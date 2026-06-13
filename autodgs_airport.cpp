//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023, 2025 Holger Teutsch
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
#include <cmath>
#include <cstring>
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
static constexpr float kVdgsXDefaultHeight = 4.5;  // m AGL

static constexpr float kDgsMinDist = 8.0;
static constexpr float kDgsMaxDist = 30.0;
static constexpr float kDgsMoveDeltaMin = 1.0;  // min/max for 'move closer' cmd
static constexpr float kDgsMoveDeltaMax = 3.0;

static bool marshaller_pe_dist_updated;  // according to pilot's eye AGL
static float marshaller_pe_dist = kMarshallerDefaultDist;

std::unique_ptr<AdgsAirport> adgs_arpt;

opmode_t operation_mode = MODE_AUTO;
const char* const opmode_str[] = { "Automatic", "Manual" };

AdgsStand::AdgsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation, int dgs_type, float dgs_dist)
    : dgs::Stand(as, arpt_icao, elevation) {

    marshaller_max_dist_ = kDgsMaxDist;

    dgs_dist_ = dgs_dist;
    last_used_dgs_dist_ = -1.0f;  // force update
    CalcDgsPosition();

    dgs_type_ = -1;  // invalidate to ensure that SetDgsType's code does something
    SetDgsType(dgs_type);
    assert(dgs_);

    // LogMsg("AdgsStand '%s', is_wet: %d, type: %d, dgs_dist: %0.1f constructed", cname(),
    //      is_wet_ ? 1 : 0, dgs_type_, dgs_dist_);
}


AdgsStand::~AdgsStand() {
    LogMsg("AdgsStand '%s' destructed", cname());
}

bool AdgsStand::has_jw() const {
    return as_.has_xp12_jw;
}

void AdgsStand::SetDgsType(int dgs_type) {
    LogMsg("AdgsStand::SetDgsType: AdgsStand '%s', type: %d, new_type: %d", cname(), dgs_type_, dgs_type);

    if (dgs_type == kAutomatic)
        dgs_type = as_.has_xp12_jw ? kVDGS : kMarshaller;

    if (dgs_type_ == dgs_type)
        return;

    dgs_type_ = dgs_type;

    CalcDgsPosition();

    if (dgs_type_ == kMarshaller)
        dgs_ = dgs::CreateMarshaller(cname());
    else {
        if (default_vdgs_type == kVdgsSafedock_T2_24)
            dgs_ = dgs::CreateSafedock_T2_24(cname(), arpt_icao_, kVdgsT2DefaultHeight);
        else
            dgs_ = dgs::CreateSafedock_X(cname(), arpt_icao_, kVdgsXDefaultHeight);
    }

    dgs_->SetPos(drawinfo_);
    SetIdle();
}

void AdgsStand::CycleDgsType() {
    int new_dgs_type = (dgs_type_ == kMarshaller ? kVDGS : kMarshaller);
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

    // change of dgs_dist_ requires update dgs position
    if (last_used_dgs_dist_ != dgs_dist_) {
        last_used_dgs_dist_ = dgs_dist_;

        // xform vector (0, -dgs_dist) into global frame
        float x = x_ + -sin_hdgt_ * (-dgs_dist_);
        float z = z_ +  cos_hdgt_ * (-dgs_dist_);

        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y_, z, &probeinfo))
            throw std::runtime_error("XPLMProbeTerrainXYZ 2 failed");

        double lat, lon, elevation;
        XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &lat, &lon, &elevation);
        SetDgsPosition(lat, lon, elevation, probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, as_.hdgt);
    }

    // may be called during initialization before the DGS instance is created, hence the check
    if (dgs_)
        dgs_->SetPos(drawinfo_);
}

// adjust may be negative to move it closer
void AdgsStand::DgsMoveCloser() {
    float delta = std::clamp(0.1f * dgs_dist_, kDgsMoveDeltaMin, kDgsMoveDeltaMax);
    dgs_dist_ -= delta;
    if (dgs_dist_ < kDgsMinDist)  // wrap around
        dgs_dist_ = kDgsMaxDist;
    marshaller_max_dist_ = dgs_dist_;
    CalcDgsPosition();
    LogMsg("stand' '%s', new dgs_dist: %0.1f", cname(), dgs_dist_);
}

//--------------------- AdgsAirport --------------------------------------------------------------

void LoadCfg(const std::string& pathname, std::unordered_map<std::string, std::tuple<int, float>>& cfg);

AdgsAirport::AdgsAirport(const dgs::AptAirport& apt_airport) : dgs::Airport(apt_airport) {
    std::unordered_map<std::string, std::tuple<int, float>> cfg;
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

        // override with user defined config
        if (const auto it = cfg.find(as.name); it != cfg.end()) {
            std::tie(dgs_type, dgs_dist) = it->second;
            LogMsg("found in config '%s', %d, %0.1f", as.name.c_str(), dgs_type, dgs_dist);
        }

        stands_.emplace_back(std::make_unique<AdgsStand>(as, name_, arpt_elevation, dgs_type, dgs_dist));
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

void LoadCfg(const std::string& pathname, std::unordered_map<std::string, std::tuple<int, float>>& cfg) {
    std::ifstream f(pathname);
    if (f.is_open()) {
        LogMsg("Loading config from '%s'", pathname.c_str());

        std::string line;
        while (std::getline(f, line)) {
            if (line.size() == 0 || line[0] == '#')
                continue;

            if (line.back() == '\r')
                line.pop_back();

            int ofs;
            float dgs_dist;
            char type;
            int n = sscanf(line.c_str(), "%c,%f, %n", &type, &dgs_dist, &ofs);
            if (n != 2 || ofs >= (int)line.size()  // distrust user input
                || !(type == 'V' || type == 'M') || dgs_dist < kDgsMinDist || dgs_dist > kDgsMaxDist) {
                LogMsg("invalid line: '%s' %d", line.c_str(), n);
                continue;
            }

            cfg[line.substr(ofs)] = std::make_tuple((type == 'M' ? kMarshaller : kVDGS), dgs_dist);
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
    for (auto const& ss : stands_) {
        auto s = dynamic_cast<AdgsStand*>(ss.get());
        char line[200];
        float dist = s->dgs_type_ == kMarshaller ? s->marshaller_max_dist_ : s->dgs_dist_;
        snprintf(line, sizeof(line), "%c, %5.1f, %s\n", (s->dgs_type_ == kMarshaller ? 'M' : 'V'), dist,
                 s->name().c_str());
        cfg[s->name()] = line;
    }

    f << "# type, dgs_dist, stand_name\n";
    f << "# type = M or V, dgs_dist = dist from parking pos in m\n";

    for (auto const& kv : cfg)
        f << kv.second;

    LogMsg("cfg written to '%s'", fn.c_str());
}


std::tuple<int, const std::string> AdgsAirport::GetStand(int idx) const {
    assert(0 <= idx && idx < (int)stands_.size());
    const AdgsStand& s = *dynamic_cast<AdgsStand*>(stands_[idx].get());
    std::string name{s.name()};
    int dgs_type = s.dgs_type_;
    return std::make_tuple(dgs_type, name);
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

int AdgsAirport::GetDgsType() const {
    // called by the ui and the selected_stand may not already be the active stand
    if (selected_stand_ >= 0)
        return dynamic_cast<AdgsStand*>(stands_[selected_stand_].get())->dgs_type_;
    if (active_stand_ >= 0)
        return dynamic_cast<AdgsStand*>(stands_[active_stand_].get())->dgs_type_;

    return kMarshaller;
}

void AdgsAirport::ResetState(state_t new_state) {
    dgs::Airport::ResetState(new_state);
    if (new_state == INACTIVE)
        FlushUserCfg();

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
     return operation_mode == MODE_AUTO && !dgs::plane->dont_connect_jetway_;
 }

void AdgsAirport::ConnectJetway() {
    LogMsg("toggling jetway command");
    XPLMCommandOnce(toggle_jetway_cmdr);
}
