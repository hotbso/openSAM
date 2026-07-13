//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023-2026 Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cassert>
#include <cstdio>
#include <fstream>
#include <map>
#include <algorithm>

#include "XPLMGraphics.h"

#include "opensam.h"
#include "autodgs_airport.h"
#include "simbrief.h"

#include "dgs/dgs.h"
#include "dgs/plane.h"

#include "log_msg.h"

static constexpr float kVdgsDefaultDist = 15.0;  // m
static constexpr float kMarshallerDefaultDist = 25.0;
static constexpr float kVdgsT2DefaultHeight = 5.0;  // m AGL
static constexpr float kVdgsXDefaultHeight = 4.5;   // m AGL
static constexpr float kStairsHeight = 2.32f;       // m AGL, must match marshaller_high.agp

static constexpr float kDgsMinDist = 8.0;
static constexpr float kDgsMaxDist = 30.0;
static constexpr float kDgsMoveDeltaMin = 1.0;  // min/max for 'move closer' cmd
static constexpr float kDgsMoveDeltaMax = 3.0;

static bool marshaller_pe_dist_updated;  // according to pilot's eye AGL
static float marshaller_pe_dist = kMarshallerDefaultDist;

std::unique_ptr<AdgsAirport> adgs_arpt;

int default_vdgs_type = kVdgsSafedock_T2_24;

OperationMode operation_mode = kAuto;
const char* const opmode_str[] = { "Automatic", "Manual" };

AdgsStand::AdgsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation, int dgs_type,
                     float dgs_dist, float dgs_height, float dgs_left_right, bool pole)
    : dgs::Stand(as, arpt_icao, elevation) {
    marshaller_max_dist_ = kDgsMaxDist;

    dgs_dist_ = dgs_dist;
    dgs_height_ = dgs_height;
    dgs_left_right_ = dgs_left_right;
    dgs_pole_ = pole;
    CalcDgsPosition();

    dgs_type_ = -1;  // invalidate to ensure that SetDgsType's code does something
    SetDgsType(dgs_type, pole);
    assert(dgs_);
    dgs_->SetPos(drawinfo_, dgs_height_);

    // LogMsg("AdgsStand '%s', is_wet: %d, type: %d, dgs_dist: %0.1f constructed", cname(),
    //      is_wet_ ? 1 : 0, dgs_type_, dgs_dist_);
}

AdgsStand::~AdgsStand() {
    LogMsg("AdgsStand '%s' destructed", cname());
}

bool AdgsStand::has_jw() const {
    return as_.has_xp12_jw;
}

void AdgsStand::SetDgsType(int dgs_type, bool pole) {
    LogMsg("AdgsStand::SetDgsType: AdgsStand '%s', type: %d, new_type: %d, pole: %d", cname(), dgs_type_, dgs_type, pole);

    if (dgs_type == kAutomatic) {
        if (as_.has_xp12_jw) {
            dgs_type = kDefaultVDGS;
            pole = true;
        } else {
            dgs_type = kMarshaller;
            pole = false;
        }
    }

    if (dgs_type_ == dgs_type && dgs_pole_ == pole)
        return;

    dgs_type_ = dgs_type;
    dgs_pole_ = pole;

    // The Marshaller is tricky as it does not have a static model.
    // If the placement position is > 2.0m it automatically adds stairs below.
    if (dgs_type == kMarshaller)    // overwrite for Marshaller
        dgs_height_ = dgs_pole_ ? kStairsHeight : 0.0f;

    bool was_vdgs = dgs_ && dgs_->isVdgs();
    dgs_ = nullptr;  // destroy old DGS instance

    if (dgs_type_ == kMarshaller) {
        if (was_vdgs)   // if we were a VDGS, we need to recalc the position for the Marshaller
            CalcDgsPosition();
        dgs_ = dgs::CreateMarshaller(cname());
    } else if (dgs_type_ == kDefaultVDGS) {
        if (!was_vdgs)  // if we were a Marshaller, we need to recalc the position for the VDGS
            CalcDgsPosition();
        if (default_vdgs_type == kVdgsSafedock_T2_24) {
            dgs_height_ = kVdgsT2DefaultHeight;
            dgs_ = dgs::CreateSafedock_T2_24(cname(), arpt_icao_, dgs_height_);
        } else {
            dgs_height_ = kVdgsXDefaultHeight;
            dgs_ = dgs::CreateSafedock_X(cname(), arpt_icao_, dgs_height_);
        }
    } else if (dgs_type_ == kVdgsSafedock_T2_24) {
        if (!was_vdgs)
            dgs_height_ = kVdgsT2DefaultHeight;
        dgs_ = dgs::CreateSafedock_T2_24(cname(), arpt_icao_, dgs_height_, /* display_only */ false, pole);
    } else if (dgs_type_ == kVdgsSafedock_X) {
        if (!was_vdgs)
            dgs_height_ = kVdgsXDefaultHeight;
        dgs_ = dgs::CreateSafedock_X(cname(), arpt_icao_, dgs_height_, /* display_only */ false, pole);
    } else
       assert(!"AdgsStand::SetDgsType: unknown type");

    SetIdle();
    // load position into DGS and force rendering
    LogMsg("AdgsStand::SetDgsType: AdgsStand '%s', type: %d, dgs_height: %0.1f, dgs_dist: %0.1f, dgs_left_right: %0.1f, pole: %d",
           cname(), dgs_type_, dgs_height_, dgs_dist_, dgs_left_right_, dgs_pole_);
    dgs_->SetPos(drawinfo_, dgs_height_);
    dgs_->UpdateInstance();
}

// tactical command to cycle between Marshaller and default VDGS during arrival
void AdgsStand::CycleDgsType() {
    int new_dgs_type = (dgs_type_ == kMarshaller ? kDefaultVDGS : kMarshaller);
    SetDgsType(new_dgs_type);
}

// compute the DGS position
void AdgsStand::CalcDgsPosition() {
    XPLMProbeInfo_t probeinfo;
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    if (dgs_type_ == kMarshaller) {
        if (!marshaller_pe_dist_updated) {
            // determine marshaller_dist_default depending on pilot eye height agl
            if (dgs::plane->pe_y_0_valid) {
                float plane_x = XPLMGetDataf(plane_x_dr);
                float plane_y = XPLMGetDataf(plane_y_dr);
                float plane_z = XPLMGetDataf(plane_z_dr);

                // get terrain y below plane y

                if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo))
                    throw std::runtime_error("XPLMProbeTerrainXYZ failed");

                // pilot eye above agl
                float pe_agl = plane_y - probeinfo.locationY + dgs::plane->pe_y_0;

                // 4.3 ~ 1 / tan(13°) -> 13° down look
                marshaller_pe_dist = std::max(kDgsMinDist, std::min(4.3f * pe_agl, kDgsMaxDist));
                marshaller_pe_dist_updated = true;
                LogMsg("setting Marshaller PE distance, pe_agl: %0.2f, dist: %0.1f", pe_agl, marshaller_pe_dist);
            }
        }

        dgs_dist_ = std::min(marshaller_pe_dist, dgs_dist_);
    }

    // add stand relative vector (dgs_left_right, -dgs_dist) to pos in local frame
    float x = x_  + cos_hdgt_ * dgs_left_right_ + -sin_hdgt_ * (-dgs_dist_);
    float z = z_  + sin_hdgt_ * dgs_left_right_ +  cos_hdgt_ * (-dgs_dist_);

    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y_, z, &probeinfo))
        throw std::runtime_error("XPLMProbeTerrainXYZ 2 failed");

    double lat, lon, elevation;
    XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &lat, &lon, &elevation);
    SetDgsPosition(lat, lon, elevation, probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, as_.hdgt);

    // may be called during initialization before the DGS instance is created, hence the check
    if (dgs_)
        dgs_->SetPos(drawinfo_, dgs_height_);
}

void AdgsStand::DgsMoveCloser() {
    float delta = std::clamp(0.1f * dgs_dist_, kDgsMoveDeltaMin, kDgsMoveDeltaMax);
    dgs_dist_ -= delta;
    if (dgs_dist_ < kDgsMinDist)  // wrap around
        dgs_dist_ = kDgsMaxDist;
    marshaller_max_dist_ = dgs_dist_;
    CalcDgsPosition();
    LogMsg("stand' '%s', new dgs_dist: %0.1f", cname(), dgs_dist_);
}

void AdgsStand::SetDistHeightLr(float dgs_dist, float dgs_height, float dgs_left_right) {
    if (std::abs(dgs_dist - dgs_dist_) < 0.05f && std::abs(dgs_height - dgs_height_) < 0.05f && std::abs(dgs_left_right - dgs_left_right_) < 0.05f)
        return;

    dgs_dist_ = dgs_dist;
    dgs_height_ = dgs_height;
    dgs_left_right_ = dgs_left_right;
    marshaller_max_dist_ = dgs_dist_;
    CalcDgsPosition();
    LogMsg("stand' '%s', new dgs_dist: %0.1f, new dgs_height: %0.1f, new dgs_left_right: %0.1f", cname(), dgs_dist_, dgs_height_, dgs_left_right_);
}

//--------------------- AdgsAirport --------------------------------------------------------------

struct DgsCfg {
    int dgs_type;
    float dgs_dist;
    float dgs_height = 5.0f;  // height of dgs (AGL) (only relevant for VDGS)
    float dgs_left_right = 0.0f;  // left/right lateral offset of the DGS from the stand centerline, in m (positive = right)
    bool pole = true;  // whether the DGS is on a pole or not (for VDGS only)
};

using DgsCfgMap = std::unordered_map<std::string, DgsCfg>;  // stand_name -> cfg

void LoadCfg(const std::string& pathname, DgsCfgMap& cfg);

AdgsAirport::AdgsAirport(const dgs::AptAirport& apt_airport) : dgs::Airport(apt_airport) {
    DgsCfgMap cfg;
    LoadCfg(user_cfg_dir + name() + ".cfg", cfg);
    if (cfg.empty())
        LoadCfg(sys_cfg_dir + name() + ".cfg", cfg);

    float arpt_elevation = XPLMGetDatad(plane_elevation_dr);  // best guess
    LogMsg("Airport '%s', elevation from plane: %0.1f", name_.c_str(), arpt_elevation);

    for (auto const& as : apt_airport.stands_) {
        int dgs_type = kAutomatic;
        float dgs_dist;
        if (as.has_xp12_jw)
            dgs_dist = kVdgsDefaultDist;
        else
            dgs_dist = kMarshallerDefaultDist;

        float dgs_left_right = 0.0f;  // default for VDGS
        float dgs_height = (dgs_type != kMarshaller) ? 5.0f : 0.0f;  // default for VDGS only
        bool pole = (dgs_type != kMarshaller);

        // override with user defined config
        if (const auto it = cfg.find(as.name); it != cfg.end()) {
            const DgsCfg& c = it->second;
            dgs_type = c.dgs_type;
            dgs_dist = c.dgs_dist;
            dgs_height = c.dgs_height;
            dgs_left_right = c.dgs_left_right;
            pole = c.pole;
            LogMsg("found in config '%s', %d, %0.1f, %0.1f, %0.1f, %d", as.name.c_str(), dgs_type, dgs_dist, dgs_height, dgs_left_right, pole);
        }

        stands_.emplace_back(std::make_unique<AdgsStand>(as, name_, arpt_elevation, dgs_type, dgs_dist, dgs_height, dgs_left_right, pole));
    }

    user_cfg_changed_ = false;

    if (dgs::ofp_destination == name_) {
        LogMsg("Now on the OFP destination '%s', looking for arrival stand %s", dgs::ofp_destination.c_str(), dgs::ofp_arrival_stand.c_str());
        for (int i = 0; i < (int)stands_.size(); i++) {
            if (stands_[i]->name() == dgs::ofp_arrival_stand) {
                selected_stand_ = i;
                LogMsg("found");
                break;
            }
        }

        if (selected_stand_ == -1)
            LogMsg("Arrival stand '%s' from OFP not found", dgs::ofp_arrival_stand.c_str());
    }
}

AdgsAirport::~AdgsAirport() {
    FlushUserCfg();
    LogMsg("AdgsAirport '%s' destructed", name().c_str());
}

void LoadCfg(const std::string& pathname, DgsCfgMap& cfg) {
    std::ifstream f(pathname);
    if (f.is_open()) {
        LogMsg("Loading config from '%s'", pathname.c_str());

        std::string line;
        bool first_line = true;
        int version = 1;

        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#' || line[0] == '\r')
                continue;

            if (line.back() == '\r')
                line.pop_back();

            if (first_line) {
                first_line = false;
                if (line.starts_with("VERSION=")) {
                    line.erase(0, 8);
                    version = std::stoi(line);
                    LogMsg("version detected: '%d'", version);
                    continue;
                }
            }

            if (version == 1) {
                // VERSION=1
                // # type, dgs_dist, stand_name
                // # type = M or V, dgs_dist = dist from parking pos in m
                // V,  15.0, 1
                // V,  15.0, 10

                int ofs;
                float dgs_dist;
                char type;
                int n = sscanf(line.c_str(), "%c,%f, %n", &type, &dgs_dist, &ofs);
                if (n != 2 || ofs >= (int)line.size()  // distrust user input
                    || !(type == 'V' || type == 'M') || dgs_dist < kDgsMinDist || dgs_dist > kDgsMaxDist) {
                    LogMsg("invalid line: '%s' %d", line.c_str(), n);
                    continue;
                }

                DgsCfg c;
                c.dgs_type = (type == 'M' ? kMarshaller : kDefaultVDGS);
                c.dgs_dist = dgs_dist;
                cfg[line.substr(ofs)] = c;
            } else if (version == 2) {
                // VERSION=2
                // # type, dgs_dist, height, lr_ofset, has_pole, stand_name
                // # type = M: Marshaller, V: default VDGS, 2: Safedock-T2, X: Safedock-X
                // # dgs_dist = dist from parking pos in m
                // # height = height of dgs (AGL) (only relevant for VDGS)
                // # lr_ofset = left/right offset from centerline
                // # has_pole = whether the DGS has a pole (or stairs for Marshaller)
                // V,  14.1,   4.5,   0.0, 1, 3
                // V,  39.3,   4.9,   0.0, 1, 4
                // M,  20.9,   5.0,   0.0, 0, 41

                int ofs, pole;
                float dgs_dist, dgs_height, dgs_left_right;
                char type;
                int n = sscanf(line.c_str(), "%c,%f,%f,%f,%d, %n", &type, &dgs_dist, &dgs_height, &dgs_left_right, &pole, &ofs);
                if (n != 5 || ofs >= (int)line.size() || dgs_height < 0.0f || dgs_height > 20.0f || dgs_left_right < -5.0f || dgs_left_right > 5.0f) { // distrust user input
                    LogMsg("invalid line: '%s' %d", line.c_str(), n);
                    continue;
                }

                DgsCfg c;

                if (type == 'M')
                    c.dgs_type = kMarshaller;
                else if (type == 'V')
                    c.dgs_type = kDefaultVDGS;
                else if (type == '2')
                    c.dgs_type = kVdgsSafedock_T2_24;
                else if (type == 'X')
                    c.dgs_type = kVdgsSafedock_X;
                else {
                    LogMsg("invalid line: '%s' %d", line.c_str(), n);
                    continue;
                }

                c.dgs_dist = dgs_dist;
                c.dgs_height = dgs_height;
                c.dgs_left_right = dgs_left_right;
                c.pole = (pole != 0);
                cfg[line.substr(ofs)] = c;
                LogMsg("found in config '%s', %d, %0.1f, %0.1f, %0.1f, %d, %d", line.substr(ofs).c_str(), c.dgs_type, c.dgs_dist, c.dgs_height, c.dgs_left_right, pole, c.pole);
             } else {
                LogMsg("unsupported version: '%d'", version);
                break;
            }
        }
    }
}

void AdgsAirport::FlushUserCfg() {
    if (!user_cfg_changed_)
        return;

    std::string fn = user_cfg_dir + name() + ".cfg";
    std::ofstream f(fn);
    if (!f.is_open()) {
        LogMsg("Can't create '%s'", fn.c_str());
        return;
    }

    // The apt.dat spec demands that the stand names must be unique but usually they are not.
    // Hence we build an ordered map first and write that out.
    // Last entry wins.
    std::map<std::string, std::string> cfg;
    for (int i = 0; i < (int)stands_.size(); i++) {
        auto p = GetStandParams(i);
        char line[200];
        char type;
        if (p.dgs_type == kMarshaller)
            type = 'M';
        else if (p.dgs_type == kDefaultVDGS)
            type = 'V';
        else if (p.dgs_type == kVdgsSafedock_T2_24)
            type = '2';
        else if (p.dgs_type == kVdgsSafedock_X)
            type = 'X';
        else
            type = '?';

        snprintf(line, sizeof(line), "%c, %5.1f, %5.1f, %5.1f, %d, %s\n", type, p.dgs_dist, p.dgs_height,
                 p.dgs_left_right, p.pole, p.name.c_str());
        cfg[p.name] = line;
    }

    f << "VERSION=2\n"
         "# type, dgs_dist, height, lr_ofset, has_pole, stand_name\n"
         "# type = M: Marshaller, V: default VDGS, 2: Safedock-T2, X: Safedock-X\n"
         "# dgs_dist = dist from parking pos in m\n"
         "# height = height of dgs (AGL) (only relevant for VDGS)\n"
         "# lr_ofset = left/right offset from centerline\n"
         "# has_pole = whether the DGS has a pole (or stairs for Marshaller)\n";

    for (auto const& kv : cfg)
        f << kv.second;

    LogMsg("cfg written to '%s'", fn.c_str());
}

AdgsStandParams AdgsAirport::GetStandParams(int idx) const {
    assert(0 <= idx && idx < (int)stands_.size());
    const AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    AdgsStandParams p;

    p.idx = idx;
    p.name = s.name();
    p.dgs_type = s.dgs_type_;
    p.dgs_dist = s.dgs_dist_;
    p.dgs_height = s.dgs_height_;
    p.dgs_left_right = s.dgs_left_right_;
    p.pole = s.dgs_pole_;
    p.has_xp12_jw = s.has_jw();
    return p;
}

void AdgsAirport::SetDgsDistance(int idx, float distance) {
    assert(0 <= idx && idx < (int)stands_.size());
    AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    s.SetDistHeightLr(distance, s.dgs_height_, s.dgs_left_right_);
    user_cfg_changed_ = true;
}

void AdgsAirport::SetDgsHeight(int idx, float height) {
    assert(0 <= idx && idx < (int)stands_.size());
    AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    s.SetDistHeightLr(s.dgs_dist_, height, s.dgs_left_right_);
    user_cfg_changed_ = true;
}

void AdgsAirport::SetDgsLeftRight(int idx, float left_right) {
    assert(0 <= idx && idx < (int)stands_.size());
    AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    s.SetDistHeightLr(s.dgs_dist_, s.dgs_height_, left_right);
    user_cfg_changed_ = true;
}

void AdgsAirport::SetDgsType(int idx, int dgs_type, bool pole) {
    assert(0 <= idx && idx < (int)stands_.size());
    AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    s.SetDgsType(dgs_type, pole);

    if (dgs_type == kMarshaller && editor_mode_)
        s.dgs_->SetMode(dgs::kArrival);

    user_cfg_changed_ = true;
}

void AdgsAirport::DgsMoveCloser() {
    if (active_stand_ >= 0) {
        dynamic_cast<AdgsStand*>(stands_[active_stand_].get())->DgsMoveCloser();
        user_cfg_changed_ = true;
    }
}

void AdgsAirport::SetDgsType(int dgs_type) {
    if (active_stand_ >= 0) {
        dynamic_cast<AdgsStand*>(stands_[active_stand_].get())->SetDgsType(dgs_type);
        user_cfg_changed_ = true;
    }
}

void AdgsAirport::Reset() {
    FlushUserCfg();
    dgs::Airport::ResetState(kIdle);

    marshaller_pe_dist_updated = false;
    marshaller_pe_dist = kMarshallerDefaultDist;
}

void AdgsAirport::CycleDgsType() {
    if (active_stand_ >= 0) {
        dynamic_cast<AdgsStand*>(stands_[active_stand_].get())->CycleDgsType();
        user_cfg_changed_ = true;
    }
}

bool AdgsAirport::auto_post_parkbrake() const {
    return operation_mode == kAuto && !dgs::plane->dont_connect_jetway_;
 }

void AdgsAirport::ConnectJetway() {
    LogMsg("toggling jetway command");
    XPLMCommandOnce(toggle_jetway_cmdr);
}

void AdgsAirport::SetEditorMode(bool on_off) {
    if (editor_mode_ == on_off)
        return;

    editor_mode_ = on_off;
    LogMsg("editor mode %s", on_off ? "enabled" : "disabled");

    if (editor_mode_) {
        // Set Marshallers to Arrival mode to make them visible in the editor.
        for (int i = 0; i < (int)stands_.size(); i++) {
            auto s = dynamic_cast<AdgsStand*>(stands_[i].get());
            if (s->dgs_type_ == kMarshaller && s->dgs_)
                s->dgs_->SetMode(dgs::kArrival);
        }
    } else
        Reset();    // the whole airport
}
