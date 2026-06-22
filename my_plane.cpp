//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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
#include <cstdio>

#include "opensam.h"
#include "XPLMNavigation.h"

#include "my_plane.h"

#include "dgs/plane.h"

#include "log_msg.h"

std::shared_ptr<MyPlane> my_plane;

// opensam/jetway/status dataref
//  0 = no jetway
//  1 = can dock
//  2 = docked
// -1 = can't dock or in transit
//
// ref == 0: opensam/jetway/number
// ref == 1: opensam/jetway/status
int MyPlane::JwStatusAcc(void* ref) {
    // opensam/jetway/number
    if (ref == 0)
        return my_plane->active_jws_.size();

    // opensam/jetway/status
    if (0 == my_plane->active_jws_.size())
        return 0;

    if (kCanDock == my_plane->state_)
        return 1;

    if (kDocked == my_plane->state_)
        return 2;

    return -1;
}

// opensam/jetway/door/status array by door
//  0 = not docked
//  1 = docked
//
int MyPlane::JwDoorStatusAcc([[maybe_unused]] void* ref, int* values, int ofs, int n) {
    if (values == nullptr)
        return kMaxDoor;

    if (n <= 0 || ofs < 0 || ofs >= kMaxDoor)
        return 0;

    n = std::min(n, kMaxDoor - ofs);

    for (int i = 0; i < n; i++) {
        values[i] = 0;
    }

    for (auto ajw_idx : my_plane->active_jws_) {
        JwCtrl& ajw = my_plane->nearest_jws_[ajw_idx];
        if (ajw.state_ == JwCtrl::kDocked) {
            int i = ajw.door_ - ofs;
            if (0 <= i && i < n)
                values[i] = 1;
        }
    }

    return n;
}

static XPLMDataRef gear_fnrml_dr_,  acf_door_x_dr_, acf_door_y_dr_, acf_door_z_dr_;

MyPlane::MyPlane() {
    LogMsg("Constructing MyPlane::");
    assert(id_ == 0);  // verify that there is only one instance

    acf_icao_ = "0000";
    ResetBeacon();
    state_ = kIdle;

    plane_lat_dr_ = XPLMFindDataRef("sim/flightmodel/position/latitude");
    assert(plane_lat_dr_ != nullptr); // these are all standard SDK datarefs, just check one
    plane_lon_dr_ = XPLMFindDataRef("sim/flightmodel/position/longitude");
    plane_y_agl_dr_ = XPLMFindDataRef("sim/flightmodel2/position/y_agl");
    gear_fnrml_dr_ = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    acf_door_x_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    acf_door_y_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    acf_door_z_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_z");

    XPLMRegisterDataAccessor("opensam/jetway/number", xplmType_Int, 0, JwStatusAcc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, JwStatusAcc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, (void*)1, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/door/status", xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                             JwDoorStatusAcc, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

// Command processing is asynchronous to ui and state machine, so we set the request flag here multiple times and reset
// it when the state machine picks it up.
void MyPlane::RequestDock() {
    if (state_ == kCanDock)
        dock_requested_ = true;
}

void MyPlane::RequestUndock() {
    if (state_ == kDocked)
        undock_requested_ = true;
}

void MyPlane::RequestToggle() {
    if (state_ == kCanDock || state_ == kDocked)
        toggle_requested_ = true;
}

bool MyPlane::dock_requested() {
    bool res{dock_requested_};
    dock_requested_ = false;
    return res;
}

bool MyPlane::undock_requested() {
    bool res{undock_requested_};
    undock_requested_ = false;
    return res;
}

bool MyPlane::toggle_requested() {
    bool res{toggle_requested_};
    toggle_requested_ = false;
    return res;
}

void MyPlane::AutoModeSet(bool auto_mode) {
    if (auto_mode_ == auto_mode)
        return;

    auto_mode_ = auto_mode;

    if (state_ == kDocking || state_ == kUndocking) {
        for (auto ajw_idx : active_jws_)
            nearest_jws_[ajw_idx].ResetJw();

        state_ = kIdle;
        return;
    }

    if (auto_mode && state_ == kSelectJws)
        return;

    if (!auto_mode && state_ == kCanDock) {
        for (auto ajw_idx : active_jws_)
            nearest_jws_[ajw_idx].UnlockJw();  // unlock jetway for manual selection

        active_jws_.clear();
        state_ = kParked;
        return;
    }
}

void MyPlane::PlaneLoadedCb() {
    LogMsg("MyPlane::PlaneLoadedCb() called");
    dgs::Plane::PlaneLoadedCb();     // init parent class
    icao_ = dgs::Plane::acf_icao_;  // copy to parent class for easier access

    // reinit all that stuff as this may be a different plane now
    on_ground_ = false;
    on_ground_ts_ = 0.0f;

    // check whether door info is overridden in the config file
    door_info_.clear();
    auto it = cfg_.find("door_1");
    if (it != cfg_.end()) {
        DoorInfo di{};
        const std::string& val = it->second;
        if (3 != sscanf(val.c_str(), "%f %f %f", &di.x, &di.y, &di.z)) {
            LogMsg("invalid door_1 format in cfg_: '%s'", val.c_str());
        } else {
            door_info_.push_back(di);
            LogMsg("using pos for door 1 from cfg_: x: %0.2f, y: %0.2f, z: %0.2f", door_info_[0].x, door_info_[0].y,
                   door_info_[0].z);
        }
    }

    // always continue with 1 door, right or wrong
    if (door_info_.empty()) {
        DoorInfo di{};
        di.x = XPLMGetDataf(acf_door_x_dr_);
        di.y = XPLMGetDataf(acf_door_y_dr_);
        di.z = XPLMGetDataf(acf_door_z_dr_);
        door_info_.push_back(di);
    }

    LogMsg("plane loaded: %s, door 1: x: %1.2f, y: %1.2f, z: %1.2f",
           acf_icao_.c_str(), door_info_[0].x, door_info_[0].y, door_info_[0].z);

    // check for a second door, seems to be not available by dataref
    // data in the acf file is often bogus, so check our own config file first
    it = cfg_.find("door_2");
    if (it != cfg_.end()) {
        DoorInfo di{};
        const std::string& val = it->second;
        if (3 != sscanf(val.c_str(), "%f %f %f", &di.x, &di.y, &di.z)) {
            LogMsg("invalid door_2 format in cfg_: '%s'", val.c_str());
        } else {
            door_info_.push_back(di);
            LogMsg("found door 2 in cfg_: x: %0.2f, y: %0.2f, z: %0.2f", door_info_[1].x, door_info_[1].y,
                   door_info_[1].z);
        }
    } else {
        LogMsg("door 2 is not defined in cfg_");
    }
}

void MyPlane::Update() {
    x_ = XPLMGetDataf(plane_x_dr);
    y_ = XPLMGetDataf(plane_y_dr);
    z_ = XPLMGetDataf(plane_z_dr);
    psi_ = XPLMGetDataf(plane_true_psi_dr);

    // on ground detection
    int og = (XPLMGetDataf(gear_fnrml_dr_) != 0.0);
    if (og != on_ground_ && now > on_ground_ts_ + 10.0f) {
        on_ground_ = og;
        on_ground_ts_ = now;
        LogMsg("transition to on_ground: %d", on_ground_);
    }

    engines_on_ = dgs::Plane::EnginesOn();

    // beacon
    if (use_engines_on_)
        beacon_on_ = engines_on_;
    else {
        beacon_on_ = dgs::Plane::BeaconOn();
    }

    parkbrake_set_ = (XPLMGetDataf(parkbrake_dr) > 0.5f);
    elevation_ = XPLMGetDataf(plane_elevation_dr);
}

void MyPlane::MemorizeParkedPos() {
    parked_x_ = x_;
    parked_z_ = z_;
    parked_ngen_ = ::ref_gen;

    // find airport I'm on now to ease debugging
    float lat = this->lat();
    float lon = this->lon();

    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        char airport_id[50];
        XPLMGetNavAidInfo(ref, NULL, NULL, NULL, NULL, NULL, NULL, airport_id, NULL, NULL);
        LogMsg("parked on airport: %s, lat,lon: %0.5f,%0.5f", airport_id, lat, lon);
    }

    // due to the delayed update of datarefs it's now a good time
    // to check ToLiss' A321 door config
    if (acf_icao_ != "A321")
        return;

    LogMsg("A321 detected, checking door config");
    if (auto door_dr = XPLMFindDataRef("AirbusFBW/A321ExitConfig")) {
        if (XPLMGetDatai(door_dr) != 0) {  // 0 = CLASSIC = 2 doors, 1 = MODERN = 1 door
            door_info_.resize(1);  // only one door supported in this config, remove the second one if it was there
            LogMsg("n_door from dataref: %d", (int)door_info_.size());
        }
    }
}

bool MyPlane::CheckParkedTeleportation() {
    if (!on_ground())
        return false;

    if (parked_ngen_ != ::ref_gen || fabsf(parked_x_ - x_) > 1.0f || fabsf(parked_z_ - z_) > 1.0f) {
        LogMsg("parked_ngen: %d, ngen: %d, parked_x: %0.3f, x: %0.3f, parked_z: %0.3f, z: %0.3f", parked_ngen_,
               ::ref_gen, parked_x_, x_, parked_z_, z_);
        return true;
    }

    return false;
}
