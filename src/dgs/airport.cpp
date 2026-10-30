//
//    openSAM: manage DGS and jetways for X Plane
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
#include <algorithm>
#include <numbers>
#include "airport.h"
#include "simbrief.h"

#include "dgs/dgs.h"
#include "dgs/plane.h"

#include "log_msg.h"
#include "flat_earth_math.h"

#include "XPLMGraphics.h"

namespace fem = flat_earth_math;

namespace {
// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static constexpr float kCapA = 15;   // Capture
static constexpr float kCapZ = 105;  // (50-80 in Safedock2 flier)

static constexpr float kAziA = 15;  // °, provide azimuth guidance
static constexpr float kAziZ = 65;  // m, from this distance

static constexpr float kAziCrossover = 6;  // m, switch from azimuth to xtrack guidance

static constexpr float kGoodZ_p = 0.2;   // stop position for nw / to stop
static constexpr float kGoodZ_m = -0.5;  // stop position for nw / beyond stop

static constexpr float kGoodX = 2.0;  // for mw

static constexpr float kCrZ = 12;  // Closing Rate starts here VDGS, Marshaller uses 0.5 * kCrZ

static constexpr int kTurnRight = 1;  // arrow on left side
static constexpr int kTurnLeft = 2;   // arrow on right side

static float ofp_ts;                   // timestamp of last ofp check
} // namespace

namespace dgs {

std::string ofp_destination;
std::string ofp_arrival_stand;


Stand::Stand(const AptStand& as, const std::string& arpt_icao, float elevation)
    : as_(as), arpt_icao_(arpt_icao) {
    elevation_ = elevation;
    elevation_is_estimate_ = true;
    sin_hdgt_ = std::sin(kD2R * as_.hdgt);
    cos_hdgt_ = std::cos(kD2R * as_.hdgt);
    drawinfo_.structSize = sizeof(drawinfo_);
    drawinfo_.heading = as_.hdgt;
    drawinfo_.pitch = drawinfo_.roll = 0.0;

    // determine local coords of stand, DGS position will be set once available
    ref_gen_ = -1;  // force update
    dgs_position_set_ = false;

    CheckRefFrame();

    // LogMsg("Stand '{}' constructed", name());
}

Stand::~Stand() {
    // LogMsg("Stand '{}' destructed", name());
    dgs_ = nullptr;
}

void Stand::SetDgsPosition(double lat, double lon, double elevation, double x, double y, double z, float psi) {
    dgs_lat_ = lat;
    dgs_lon_ = lon;
    dgs_altitude_ = elevation;
    dgs_position_set_ = true;
    drawinfo_.x = x;
    drawinfo_.y = y;
    drawinfo_.z = z;
    drawinfo_.heading = psi;
}

// set x_, y_, z_, drawinfo_ from as_.lon, as_.lat in the current reference frame
void Stand::CheckRefFrame() {
    if (ref_gen_ == ref_gen)
        return;

    ref_gen_ = ref_gen;
    XPLMWorldToLocal(as_.lat, as_.lon, elevation_, &x_, &y_, &z_);

    if (elevation_is_estimate_) {
        XPLMProbeInfo_t probeinfo;
        probeinfo.structSize = sizeof(XPLMProbeInfo_t);

       // refine elevation with probe, get is_wet_ info
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x_, y_, z_, &probeinfo)) {
            LogMsg("terrain probe 1 failed, name: {}, ll: ({}, {}), elevation: {}", name(), as_.lat, as_.lon, elevation_);
            throw std::runtime_error("XPLMProbeTerrainXYZ 1 failed");
        }

        // On the first pass elevation is only an estimate so we iterate.
        // It makes a difference on higher elevation airports like LOWI.
        double unused;
        XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &unused, &unused, &elevation_);
        XPLMWorldToLocal(as_.lat, as_.lon, elevation_, &x_, &y_, &z_);
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x_, y_, z_, &probeinfo)) {
            LogMsg("terrain probe 1a failed, name: {}, ll: ({}, {}), elevation: {}", name(), as_.lat, as_.lon, elevation_);
            throw std::runtime_error("XPLMProbeTerrainXYZ 1a failed");
        }

        is_wet_ = probeinfo.is_wet;
        x_ = probeinfo.locationX;
        y_ = probeinfo.locationY;
        z_ = probeinfo.locationZ;

        elevation_is_estimate_ = false;
    }

    if (dgs_position_set_) {
        XPLMWorldToLocal(dgs_lat_, dgs_lon_, dgs_altitude_, &x_, &y_, &z_);
        drawinfo_.x = x_;
        drawinfo_.y = y_;
        drawinfo_.z = z_;
        if (dgs_)
            dgs_->SetPos(drawinfo_);
    }
}

void Stand::Local2Stand(double x, double z, float& x_stand_local, float& z_stand_local) {
    CheckRefFrame();
    // transform into stand local coordinate system

    // xlate + rotate into stand frame
    // float: double precision not needed for local stand coordinates, a few 100m max
    float dx = x - x_;
    float dz = z - z_;

    x_stand_local =  cos_hdgt_ * dx + sin_hdgt_ * dz;
    z_stand_local = -sin_hdgt_ * dx + cos_hdgt_ * dz;
}

void Stand::SetIdle() {
    if (dgs_)   // for sam airports not every stand has a DGS
        dgs_->SetMode(kIdle);
}

bool Stand::isVdgs() const {
    return dgs_ != nullptr && dgs_->isVdgs();
}

//---------------------------------------------------------------------------
const char* const Airport::state_str_[] = {"IDLE",   "DEPARTURE",  "BOARDING", "ARRIVAL",       "ENGAGED",
                                           "TRACK",  "GOOD",       "BAD",      "PARKBRAKE_SET", "BEACON_OFF",
                                           "PARKED", "DEBOARDING", "DONE"};

static int seqno_base = 0;

Airport::Airport(const AptAirport& apt_airport) : seqno_(++seqno_base), apt_airport_(apt_airport) {
    CheckRefFrameShift();   // ensure ref_gen is up to date
    ref_gen_ = ref_gen;

    name_ = apt_airport.icao_;
    stands_.reserve(apt_airport.stands_.size());

    state_ = kIdle;
    active_stand_ = selected_stand_ = departure_stand_ = -1;

    state_change_ts_ = dgs_params_.distance = sin_wave_prev_ = 0.0f;
    departure_stand_ts_ = nearest_stand_ts_ = update_dgs_log_ts_ = 0.0f;
    has_xp12_jws_ = apt_airport.has_xp12_jws_;
}

Airport::~Airport() {
    stands_.clear();  // destroy all stands and DGS
    LogMsg("Airport '{}' destructed, seqno: {}, active instances: {}", name(), seqno_, ObjInstance::ActiveInstances());
}

bool Airport::active_stand_has_xp12_jw() const {
    return (active_stand_ >= 0 && stands_[active_stand_]->has_xp12_jw()) ||
           (departure_stand_ >= 0 && stands_[departure_stand_]->has_xp12_jw());
}

void Airport::SetSelectedStand(int selected_stand) {
    assert(-1 <= selected_stand && selected_stand < (int)stands_.size());
    if (selected_stand_ == selected_stand)
        return;
    selected_stand_ = selected_stand;

    if (state_ > kArrival)
        ResetState(kArrival);
}

void Airport::FindNearestStand() {
    // Check if the currently active stand is also the selected stand
    if (active_stand_ >= 0 && active_stand_ == selected_stand_)
        return;

    double dist = 1.0E10;
    int min_stand = -1;

    double plane_x = XPLMGetDatad(plane_x_dr);
    double plane_z = XPLMGetDatad(plane_z_dr);

    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    if (selected_stand_ >= 0) {
        dist = 0.0;
        min_stand = selected_stand_;
    } else {
        for (int i = 0; i < (int)stands_.size(); i++) {
            Stand& s = *stands_[i];
            if (s.is_wet_)
                continue;

            // heading in local system
            float local_hdgt = fem::RA(plane_hdgt - s.hdgt());

            if (fabsf(local_hdgt) > 90.0f)
                continue;  // not looking to stand

            // transform into gate local coordinate system
            float x_sl, z_sl;
            s.Local2Stand(plane_x, plane_z, x_sl, z_sl);

            // nose wheel
            float nw_z = z_sl - plane->nw_z_;
            float nw_x = x_sl + plane->nw_z_ * std::sin(kD2R * local_hdgt);

            float d = std::hypot(nw_x, nw_z);
            if (d > kCapZ + 50)  // fast exit
                continue;

            // LogMsg("stand: {}, z: {:2.1f}, x: {:2.1f}", s.name(), nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                // LogMsg("behind: {}", s.cname());
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / kD2R;
                // LogMsg("angle to plane: {}, {:3.1f}", s.cname(), angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = fem::RA(-angle - local_hdgt);
                // LogMsg("rel_to_stand: {}, nw_x: {:0.1f}, local_hdgt {:0.1f}, rel_to_stand: {:0.1f}", s.cname(), nw_x,
                //     local_hdgt, rel_to_stand);
                if ((nw_x > 10.0 && rel_to_stand < -60.0) || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    // LogMsg("drive by {}", s.cname());
                    continue;
                }
            }

            // for the final comparison give xtrack a higher weight + consider heading deviation
            static constexpr float xtrack_weight = 4.0;
            d = std::hypot(xtrack_weight * nw_x, nw_z) + std::abs(local_hdgt);

            if (d < dist) {
                // LogMsg("new min: {}, z: {:2.1f}, x: {:2.1f}", s.cname(), nw_z, nw_x);
                dist = d;
                min_stand = i;
            }
        }
    }

    if (min_stand >= 0 && min_stand != active_stand_) {
        // the current active stand gone regardless whether we can activate a new one or not
        if (active_stand_ >= 0)
            stands_[active_stand_]->SetIdle();
        active_stand_ = -1;

        const Stand& ms = *stands_[min_stand];
        if (!ms.dgs_position_set_) {
            LogMsg("Nearest stand '{}' found, but DGS position not yet set, ignored", ms.name());
            return;
        }
        LogMsg("active stand now: {}, ll: ({}, {}), hdgt: {}, dist: {}", ms.cname(), ms.lat(), ms.lon(), ms.hdgt(), dist);

        active_stand_ = min_stand;
        state_ = kEngaged;
    }
}

// find the stand the plane is parked on
int Airport::FindDepartureStand() {
    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);
    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    // nose wheel
    float nw_z = plane_z - plane->nw_z_ * std::cos(kD2R * plane_hdgt);
    float nw_x = plane_x + plane->nw_z_ * std::sin(kD2R * plane_hdgt);

    for (int i = 0; i < (int)stands_.size(); i++) {
        Stand& s = *stands_[i];
        if (!s.isVdgs())
            continue;

        if (std::abs(fem::RA(plane_hdgt - s.hdgt())) > 3.0f)
            continue;

        float dx = nw_x - s.x_;
        float dz = nw_z - s.z_;
        // LogMsg("stand: {}, z: {:2.1f}, x: {:2.1f}", s.cname(), dz, dx);
        if (dx * dx + dz * dz < 1.0f) {
            if (!s.dgs_position_set_) {
                LogMsg("Departure stand '{}' found, but DGS position not yet set, ignored", s.name());
                return -1;
            }
            // Return the first matching stand found
            return i;
        }
    }

    return -1;
}

float Airport::StateMachine() {
    CheckRefFrameShift();   // ensure ref_gen is up to date
    if (ref_gen_ != ref_gen) {
        ref_gen_ = ref_gen;
        LogMsg("reference frame changed, moving all instances");
        for (auto& s : stands_) {
            s->CheckRefFrame();
        }
    }

    static float last_tick_all = 0.0f;

    // update clock, brightness and other stuff for all stands at a low frequency
    if (now > last_tick_all + 2.0f) {
        last_tick_all = now;
        for (int i = 0; i < (int)stands_.size(); i++)
            if (i != active_stand_) { // dont't interfere with updates of the active stand
                if (stands_[i]->dgs_)
                    stands_[i]->dgs_->Tick();
            }
    }

    State state_prev = state_;

    // DEPARTURE and friends ...
    // that's all low freq stuff
    if (kIdle <= state_ && state_ <= kBoarding) {
        if (now > departure_stand_ts_ + 2.0f) {
            departure_stand_ts_ = now;
            // on beacon or engine or teleportation -> INACTIVE
            if (plane->BeaconOn() || plane->EnginesOn()) {
                if (departure_stand_ >= 0)
                    stands_[departure_stand_]->SetIdle();
                departure_stand_ = -1;
                state_ = kIdle;
                return 2.0f;
            }

            // check for stand (new or changed)
            int dsi = FindDepartureStand();
            // LogMsg("departure stand: {}, dsi: {}", dsi >= 0 ? stands_[dsi]->name() : "*none*", dsi);
            if (dsi != departure_stand_) {
                if (departure_stand_ >= 0)
                    stands_[departure_stand_]->SetIdle();
                LogMsg("Departure stand now '{}'", dsi >= 0 ? stands_[dsi]->name() : "*none*");
                departure_stand_ = dsi;
            }
        }

        if (departure_stand_ < 0) {
            state_ = kIdle;
            return 4.0f;
        }

        Stand& ds = *stands_[departure_stand_];

        if (plane->PaxNo() <= 0) {
            state_ = kDeparture;
            if (state_ != state_prev) {
                LogMsg("New state {}", state_str_[state_]);
                ds.dgs_->SetMode(dgs::kDeparture);
            }
            // FALLTHROUGH
        }

        if (state_ == kIdle)
            return 1.0f;            // just keep it ticking

        // cdm data may come in late during boarding
        if (state_ == kDeparture || state_ == kBoarding) {
            // although LoadIfNewer is cheap throttling it is even cheaper
            if (now > ofp_ts + 5.0f) {
                ofp_ts = now;
                if (Ofp::LoadIfNewer()) {  // (re-)fetch ofp
                    ds.dgs_->NotifyOfpUpdate();

                    // extract arrival stand from ofp remarks if any
                    if (!Ofp::ofp.dx_rmk.empty()) {
                        LogMsg("OFP Departure Remarks: '{}'", Ofp::ofp.dx_rmk);
                        auto pos = Ofp::ofp.dx_rmk.find("ARRIVAL_STAND=");
                        if (pos != std::string::npos) {
                            ofp_arrival_stand = Ofp::ofp.dx_rmk.substr(pos + 14);
                            auto endpos = ofp_arrival_stand.find_first_of(";,\n\r");
                            if (endpos != std::string::npos)
                                ofp_arrival_stand = ofp_arrival_stand.substr(0, endpos);
                            ofp_destination = Ofp::ofp.destination;
                            LogMsg("OFP Arrival Stand set to '{}@{}'", ofp_arrival_stand,
                                   ofp_destination);
                        }
                    } else {
                        ofp_arrival_stand.clear();
                        ofp_destination.clear();
                    }
                }
            }
        }

        if (state_ == kDeparture) {
            if (plane->PaxNo() > 0) {
            state_ = kBoarding;
                LogMsg("New state {}", state_str_[state_]);
                // FALLTHROUGH
            } else
                return ds.dgs_->Tick();  // just scroll the text
        }

        if (state_ == kBoarding) {
            int pax_no = plane->PaxNo();
            // LogMsg("boarding PaxNo: {}", pax_no);
            ds.dgs_->SetPaxNo(pax_no);
            return ds.dgs_->Tick();
        }
    }

    // ARRIVAL and friends ...
    // this can be high freq stuff

    // throttle costly search...
    // ... but if we have a new selected stand activate it immediately
    if (now > nearest_stand_ts_ + 2.0 || (selected_stand_ >= 0 && selected_stand_ != active_stand_)) {
        FindNearestStand();
        nearest_stand_ts_ = now;
    }

    if (active_stand_ < 0) {
        state_ = kArrival;
        return 2.0;
    }

    State new_state = state_;

    float loop_delay = 0.2;

    Stand& as = *stands_[active_stand_];

    // xform plane pos into stand local coordinate system
    float dx = XPLMGetDataf(plane_x_dr) - as.x_;
    float dz = XPLMGetDataf(plane_z_dr) - as.z_;
    float local_x =  as.cos_hdgt_ * dx + as.sin_hdgt_ * dz;
    float local_z = -as.sin_hdgt_ * dx + as.cos_hdgt_ * dz;

    // relative heading in stand local system +/ 180°
    float local_hdgt = fem::RA(XPLMGetDataf(plane_true_psi_dr) - as.hdgt());

    // nose wheel
    float nw_z = local_z - plane->nw_z_;
    float nw_x = local_x + plane->nw_z_ * std::sin(kD2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane->mw_z_;
    float mw_x = local_x + plane->mw_z_ * std::sin(kD2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = std::clamp((nw_z - kAziCrossover) / 20.0, 0.0, 1.0);
    float plane_ref_z = (1.0 - a) * plane->nw_z_ + a * plane->mw_z_;
    float ref_z = local_z - plane_ref_z;
    float ref_x = local_x + plane_ref_z * std::sin(kD2R * local_hdgt);

    float xtrack = 0.0;  // xtrack for VDGS, set later if needed

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 5.0f)) / kD2R;
    else
        azimuth_nw = 0.0;

    bool locgood = (std::abs(mw_x) <= kGoodX && kGoodZ_m <= nw_z && nw_z <= kGoodZ_p);
    bool beacon_on = plane->BeaconOn();

    memset(&dgs_params_, 0, sizeof(dgs_params_));
    dgs_params_.distance = nw_z;

    // set drefs according to *current* state
    switch (state_) {
        case kEngaged:
            if (beacon_on) {
                if ((dgs_params_.distance <= kCapZ) && (std::abs(azimuth_nw) <= kCapA))
                    new_state = kTrack;
            } else {  // not beacon_on
                new_state = kDone;
            }

            // always light up the VDGS or signal "this way" for the selected stand
            if (active_stand_ == selected_stand_) {
                dgs_params_.status = kDgsGstIdentified;  // plane id
                dgs_params_.track = 1;   // lead-in
            }
            break;

        case kTrack: {
            if (!beacon_on) {  // don't get stuck in TRACK
                new_state = kDone;
                break;
            }

            if (locgood) {
                new_state = kGood;
                break;
            }

            if (nw_z < kGoodZ_m) {
                new_state = kBad;
                break;
            }

            if ((dgs_params_.distance > kCapZ) || (std::abs(azimuth_nw) > kCapA)) {
                new_state = kEngaged;  // moving away from current gate
                break;
            }

            dgs_params_.status = kDgsGstIdentified;  // plane id
            if (dgs_params_.distance > kAziZ || std::abs(azimuth_nw) > kAziA) {
                dgs_params_.track = 1;  // lead-in only
                break;
            }

            // compute distance_ and guidance commands

            // xform xtrack distance to values required by the OBJ
            xtrack = std::clamp(ref_x, -4.0f, 4.0f);     // in m, 4 is hardcoded in the OBJ
            xtrack = std::roundf(xtrack * 2.0f) / 2.0f;  // round to 0.5 increments
            dgs_params_.xtrack = xtrack;

            // compute left/right command
            if (ref_z > kAziCrossover) {
                // far, aim to an intermediate point between ref point and stand
                float req_hdgt = std::atan2(-ref_x, 0.3f * ref_z) / kD2R;  // required hdgt
                float d_hdgt = req_hdgt - local_hdgt;  // degrees to turn
                if (d_hdgt < -1.5f)
                    dgs_params_.lr = kTurnLeft;
                else if (d_hdgt > 1.5f)
                    dgs_params_.lr = kTurnRight;

                if (now > update_dgs_log_ts_ + 2.0f)
                    LogMsg(
                        "req_hdgt: {:0.1f}, local_hdgt: {:0.1f}, d_hdgt: {:0.1f}, mw: ({:0.1f}, {:0.1f}), nw: ({:0.1f}, {:0.1f}), "
                        "ref: ({:0.1f}, {:0.1f}), "
                        "x: {:0.1f}, ",
                        req_hdgt, local_hdgt, d_hdgt, mw_x, mw_z, nw_x, nw_z, ref_x, ref_z, local_x);

            } else {
                // close, use xtrack
                if (ref_x < -0.25f)
                    dgs_params_.lr = kTurnRight;
                else if (ref_x > 0.25f)
                    dgs_params_.lr = kTurnLeft;
            }

            if (dgs_params_.distance <= kCrZ / 2) {
                dgs_params_.track = 3;
                loop_delay = 0.03;
            } else  // azimuth only
                dgs_params_.track = 2;
        } break;

        case kGood: {
            // @stop position
            dgs_params_.status = kDgsGstStop;  // stop position
            dgs_params_.lr = 3;

            int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5f);
            if (!locgood)
                new_state = kTrack;
            else if (parkbrake_set || !beacon_on)
                new_state = kParkBrakeSet;
        } break;

        case kBad:
            if (!beacon_on && (now > state_change_ts_ + 5.0f)) {
                ResetState(kIdle);
                return loop_delay;
            }

            if (nw_z >= kGoodZ_m)  // moving backwards
                new_state = kTrack;
            else {
                // Too far
                dgs_params_.status = kDgsGstTooFar;
                dgs_params_.lr = 3;
            }
            break;

        case kParkBrakeSet:
            dgs_params_.status = kDgsGstOk;
            dgs_params_.lr = 0;
            // wait for beacon off
            if (!beacon_on)
                new_state = kBeaconOff;
            break;

        case kBeaconOff:
            dgs_params_.status = kDgsGstOk;
            dgs_params_.lr = 0;

            // let the dust settle after switching off the beacon
            // e.g. for jetway selection + the guys with the chocks aren't that fast either
            if (now > state_change_ts_ + 5.0f) {
                new_state = kDone;
                parked_pax_no_ = plane->PaxNo();
                bool auto_post_bp = auto_post_parkbrake();
                LogMsg("beacon off, parked_pax_no_: {}, auto_post_parkbrake: {}", parked_pax_no_, auto_post_bp);
                if (auto_post_bp) {
                    if (as.has_jw())
                        ConnectJetway();

                    if (plane->SetChocks() && as.dgs_->HasEqStatus()) {
                        LogMsg("parked_pax_no_: {}, setting chocks", parked_pax_no_);
                        new_state = kParked;
                    }
                }
            }
            break;


        case kParked: {
            as.dgs_->SetMode(dgs::kParked);
            int pax_no = plane->PaxNo();
            // LogMsg("parked, PaxNo: {}", pax_no);
            if (pax_no < parked_pax_no_) {
                new_state = kDeboarding;
                LogMsg("Pax deboarding started, pax_no: {}", pax_no);
                break;
            }

            if (now > state_change_ts_ + 80.0f)
                new_state = kDone;
            else
                return as.dgs_->Tick();
        } break;

        case kDeboarding: {
            as.dgs_->SetMode(dgs::kDeboarding);
            int pax_no = plane->PaxNo();
            as.dgs_->SetPaxNo(pax_no);
            loop_delay = as.dgs_->Tick();
            if (pax_no <= 0)
                new_state = kDone;
        } break;

        case kDone:
            if (now > state_change_ts_ + 5.0f) {
                ResetState(kIdle);
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state_) {
        LogMsg("state transition {} -> {}, beacon: {}", state_str_[state_], state_str_[new_state], beacon_on);
        state_ = new_state;
        state_change_ts_ = now;
        return -1;  // see you on next frame
    }

    // guidance postprocessing and logging
    if (kArrival < state_ && state_ < kParked) {
        // don't flood the log
        if (now > update_dgs_log_ts_ + 2.0f) {
            update_dgs_log_ts_ = now;
            LogMsg("stand: {}, state: {}, status: {}, track: {}, lr: {}, distance: {:.2}, xtrack: {:0.1f} m",
                   as.name(), state_str_[state_], (int)dgs_params_.status, dgs_params_.track, dgs_params_.lr, dgs_params_.distance, dgs_params_.xtrack);
        }

        // xform drefs into required constraints for the OBJs
        if (dgs_params_.track == 0 || dgs_params_.track == 1) {
            dgs_params_.distance = 0.0f;
            dgs_params_.xtrack = 0.0f;
        }

        dgs_params_.distance = std::clamp(dgs_params_.distance, kGoodZ_m, kCrZ);
        dgs_params_.ref_x = ref_x;
        dgs_params_.ref_z = ref_z;

        as.dgs_->SetMode(dgs::kArrival);
        as.dgs_->SetGuidanceParams(dgs_params_);
    }

    return loop_delay;
}

void Airport::ResetState(State new_state) {
    if (state_ != new_state)
        LogMsg("setting state to {}", state_str_[new_state]);

    state_ = new_state;
    for (auto& s : stands_)
        s->SetIdle();
    active_stand_ = -1;

    if (new_state == kIdle)
        selected_stand_ = -1;
}

void Airport::SetArrival() {
    // kick off guidance when we arrive at the airport
    ResetState(plane->BeaconOn() ? Airport::kArrival : Airport::kIdle);
}

} // namespace dgs
