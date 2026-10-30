//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2025, 2026  Holger Teutsch
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

#include <cstdlib>
#include <cassert>
#include <cmath>
#include <string_view>
#include <print>

#include "my_plane.h"
#include "mpadapter_xpilot.h"
#include "log_msg.h"

namespace {

static XPLMDataRef modeS_id_dr, icao_type_dr, flight_id_dr,  // identity
    x_dr, y_dr, z_dr, psi_dr,                                // position
    on_ground_dr, lights_dr, throttle_dr;                    // state

class MpPlane_xPilot : public OsPlane {
    const int slot_;
    std::string flight_id_;

    // parking brake emulation
    float last_move_ts_{0};
    float x_last_move_{0}, z_last_move_{0};

   public:
    MpPlane_xPilot(int slot, std::string_view flight_id, std::string_view icao);
    ~MpPlane_xPilot() override {}

    void update(float x, float y, float z, float psi, float throttle, int lights);

    bool auto_mode() const override { return true; }
    bool dock_requested() override { return true; }
};

} // namespace

MpPlane_xPilot::MpPlane_xPilot(int slot, std::string_view flight_id, std::string_view icao) : slot_(slot) {
    flight_id_ = flight_id;
    icao_ = icao;

    on_ground_ = true;  // otherwise we were not here

    LogMsg("pid={:03}, constructing MpPlane {}/{}", id_, flight_id_, icao_);

    MpAdapter::LoadDoorInfo(*this);
    if (!door_info_.empty()) {
        LogMsg("pid={:03}, found door 1 in door_info_map: x: {:.2}, y: {:.2}, z: {:.2}", id_, door_info_[0].x,
               door_info_[0].y, door_info_[0].z);
    } else {
        LogMsg("pid={:03}, {}: door 1 is not defined in door_info_map, deactivating slot", id_, icao_);
        state_ = kDisabled;
        return;
    }

    state_ = kIdle;
}

void MpPlane_xPilot::update(float x, float y, float z, float psi, float throttle, int lights) {
    if (state_ == kDisabled)
        return;

    x_ = x;
    y_ = y;
    z_ = z;
    psi_ = psi;

    engines_on_ = (throttle > 0.1f);

    // 'parkbrake' detection
    if (std::abs(x_ - x_last_move_) > 0.5 || std::abs(z_ - z_last_move_) > 0.5) {
        x_last_move_ = x_;
        z_last_move_ = z_;
        last_move_ts_ = now;
    }

    beacon_on_ = ((lights & 1) == 1);
    parkbrake_set_ = ((now - last_move_ts_) > 10.0f);

    LogMsg("MP update: pid={:03}, slot: {:02d}, icao: {}, id: {}, beacon: {}, parkbrake_set: {}, engine_on: {}, state: {}",
           id_, slot_, icao_, flight_id_, beacon_on_, parkbrake_set_, engines_on_, state_str_[state_]);
}

//==============  MpAdapter_xPilot ========================================
bool MpAdapter_xPilot::probe() {
    static XPLMDataRef xpilot_status_dr;
    if (xpilot_status_dr == nullptr)
        xpilot_status_dr = XPLMFindDataRef("xpilot/login/status");

    return (xpilot_status_dr && (XPLMGetDatai(xpilot_status_dr) > 0));
}

MpAdapter_xPilot::MpAdapter_xPilot() {
    LogMsg("MpAdapter_xPilot constructor");
    static bool init_done{false};

    if (!init_done) {
        modeS_id_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/modeS_id");
        icao_type_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/icao_type");
        flight_id_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/flight_id");
        x_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/x");
        y_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/y");
        z_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/z");
        psi_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/psi");
        on_ground_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/weight_on_wheels");
        lights_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/lights");
        throttle_dr = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/throttle");
        init_done = true;
    }

    n_planes_ = XPLMGetDatavi(modeS_id_dr, NULL, 0, 0);
    LogMsg("MpPlane_xPilot drefs #: {}", n_planes_);

    modeS_id_val_ = std::make_unique_for_overwrite<int[]>(n_planes_);
    icao_type_val_ = std::make_unique_for_overwrite<char[]>(n_planes_ * 8);
    flight_id_val_ = std::make_unique_for_overwrite<char[]>(n_planes_ * 8);
    on_ground_val_ = std::make_unique_for_overwrite<int[]>(n_planes_);
    lights_val_ = std::make_unique_for_overwrite<int[]>(n_planes_);
    x_val_ = std::make_unique_for_overwrite<float[]>(n_planes_);
    y_val_ = std::make_unique_for_overwrite<float[]>(n_planes_);
    z_val_ = std::make_unique_for_overwrite<float[]>(n_planes_);
    psi_val_ = std::make_unique_for_overwrite<float[]>(n_planes_);
    throttle_val_ = std::make_unique_for_overwrite<float[]>(n_planes_);
}

MpAdapter_xPilot::~MpAdapter_xPilot() {
    LogMsg("MpAdapter_xPilot destructor");
}

#define LOAD_DR(type, name) XPLMGetDatav##type(name##_dr, name##_val_.get(), 0, n_planes_)

// static
float MpAdapter_xPilot::update() {
    LOAD_DR(i, modeS_id);
    LOAD_DR(i, on_ground);
    LOAD_DR(i, lights);
    LOAD_DR(f, x);
    LOAD_DR(f, y);
    LOAD_DR(f, z);
    LOAD_DR(f, psi);
    LOAD_DR(f, throttle);

    XPLMGetDatab(icao_type_dr, icao_type_val_.get(), 0, n_planes_ * 8);
    XPLMGetDatab(flight_id_dr, flight_id_val_.get(), 0, n_planes_ * 8);

    std::unordered_map<std::string, bool> dref_planes;
    dref_planes.reserve(n_planes_ * 2);

    for (int i = 1; i < n_planes_; i++) {
        // slot empty or not on ground or far away -> ignore
        if (0 == modeS_id_val_[i] || !on_ground_val_[i] ||
            std::hypot(x_val_[i] - my_plane->x(), z_val_[i] - my_plane->z()) > kMpMaxDist)
            continue;

        // flight_id
        char* cptr = flight_id_val_.get() + i * 8;
        cptr[7] = '\0';  // play it safe
        std::string_view flight_id(cptr);

        // icao type
        cptr = icao_type_val_.get() + i * 8;
        cptr[7] = '\0';  // play it safe
        std::string_view icao(cptr);

        std::string key = std::format("{}/{}", flight_id, icao);
        dref_planes[key] = true;

        auto it = mp_planes_.find(key);
        if (it != mp_planes_.end()) {
            MpPlane_xPilot* mp_plane = static_cast<MpPlane_xPilot*>(it->second.get());
            mp_plane->update(x_val_[i], y_val_[i], z_val_[i], psi_val_[i], throttle_val_[i], lights_val_[i]);
        } else {
            MpPlane_xPilot* mp_plane = new MpPlane_xPilot(i, flight_id, icao);
            mp_plane->update(x_val_[i], y_val_[i], z_val_[i], psi_val_[i], throttle_val_[i], lights_val_[i]);
            mp_planes_.emplace(key, mp_plane);
        }
    }

    // loop over mp_planes_ and delete the ones that are no longer in drefs
    for (auto it = mp_planes_.begin(); it != mp_planes_.end();) {
        const std::string& key = it->first;
        OsPlane& plane = *(it->second);

        if (!dref_planes.contains(key)) {
            LogMsg("pid={:03} key: {} not longer exists, deleted", plane.id_, key);
            it = mp_planes_.erase(it);
        } else
            ++it;
    }

    if (!mp_planes_.empty())
        LogMsg("------------------ MP active planes found: {} -----------------", mp_planes_.size());
    return 2.0f;
}
