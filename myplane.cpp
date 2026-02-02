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

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <fstream>

#include "openSAM.h"
#include "XPLMPlanes.h"
#include "XPLMNavigation.h"

#include "plane.h"
#include "samjw.h"

MyPlane my_plane;

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
        return my_plane.active_jws_.size();

    // opensam/jetway/status
    if (0 == my_plane.active_jws_.size())
        return 0;

    if (Plane::CAN_DOCK == my_plane.state_)
        return 1;

    if (Plane::DOCKED == my_plane.state_)
        return 2;

    return -1;
}

// opensam/jetway/door/status array by door
//  0 = not docked
//  1 = docked
//
int MyPlane::JwDoorStatusAcc([[maybe_unused]] XPLMDataRef ref, int* values, int ofs, int n) {
    if (values == nullptr)
        return kMaxDoor;

    if (n <= 0 || ofs < 0 || ofs >= kMaxDoor)
        return 0;

    n = std::min(n, kMaxDoor - ofs);

    for (int i = 0; i < n; i++) {
        values[i] = 0;
    }

    for (auto& ajw : my_plane.active_jws_)
        if (ajw.state_ == JwCtrl::DOCKED) {
            int i = ajw.door_ - ofs;
            if (0 <= i && i < n)
                values[i] = 1;
        }

    return n;
}

static XPLMDataRef plane_x_dr_, plane_y_dr_, plane_z_dr_, plane_elevation_dr_, plane_true_psi_dr_, beacon_dr_,
    eng_running_dr_, parkbrake_dr_, gear_fnrml_dr_, is_helicopter_dr_, acf_icao_dr_, acf_cg_y_dr_, acf_cg_z_dr_,
    acf_gear_z_dr_, acf_door_x_dr_, acf_door_y_dr_, acf_door_z_dr_;

static bool FindIcaoInFile(const std::string& acf_icao, const std::string& fn);

// constructor is called too early, no SDK calls here
MyPlane::MyPlane() {
    assert(id_ == 0);  // verify that there is only one instance

    icao_ = "0000";
    ResetBeacon();
    ui_unlocked_ = false;
    state_ = IDLE;
}

// initialize MyPlane instance from XPLMPluginStart
// register dref accessors
void MyPlane::Init() {
    static bool init_done{false};
    if (init_done)
        return;

    LogMsg("initing MyPlane::");

    plane_x_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_x");
    assert(plane_x_dr_ != nullptr);
    plane_y_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_y");
    plane_z_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    plane_lat_dr_ = XPLMFindDataRef("sim/flightmodel/position/latitude");
    plane_lon_dr_ = XPLMFindDataRef("sim/flightmodel/position/longitude");
    plane_elevation_dr_ = XPLMFindDataRef("sim/flightmodel/position/elevation");
    plane_true_psi_dr_ = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    plane_y_agl_dr_ = XPLMFindDataRef("sim/flightmodel2/position/y_agl");
    eng_running_dr_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
    beacon_dr_ = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    parkbrake_dr_ = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
    gear_fnrml_dr_ = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    is_helicopter_dr_ = XPLMFindDataRef("sim/aircraft2/metadata/is_helicopter");
    acf_icao_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    acf_cg_y_dr_ = XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    acf_cg_z_dr_ = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    acf_gear_z_dr_ = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    acf_door_x_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    acf_door_y_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    acf_door_z_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_z");
    pax_no_dr_ = nullptr;

    XPLMRegisterDataAccessor("opensam/jetway/number", xplmType_Int, 0, JwStatusAcc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, JwStatusAcc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, (void*)1, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/door/status", xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                             JwDoorStatusAcc, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    init_done = true;
}

void MyPlane::RequestDock() {
    if (state_ == CAN_DOCK) {
        dock_requested_ = true;
        LogMsg("Dock request ACCEPTED");
    } else {
        // Log detailed rejection reason
        switch (state_) {
            case DISABLED:
                LogMsg("Dock request REJECTED: plugin is disabled");
                break;
            case IDLE:
                LogMsg("Dock request REJECTED: plane not parked (state=IDLE), beacon_on=%d, on_ground=%d",
                       beacon_on_, on_ground_);
                break;
            case PARKED:
                LogMsg("Dock request REJECTED: searching for jetways (state=PARKED)");
                break;
            case SELECT_JWS:
                LogMsg("Dock request REJECTED: selecting jetways (state=SELECT_JWS)");
                break;
            case DOCKING:
                LogMsg("Dock request REJECTED: already docking");
                break;
            case DOCKED:
                LogMsg("Dock request REJECTED: already docked");
                break;
            case UNDOCKING:
                LogMsg("Dock request REJECTED: currently undocking");
                break;
            case CANT_DOCK:
                LogMsg("Dock request REJECTED: no suitable jetway found");
                break;
            default:
                LogMsg("Dock request REJECTED: unknown state %d", state_);
        }
    }
}

void MyPlane::RequestUndock() {
    if (state_ == DOCKED) {
        undock_requested_ = true;
        LogMsg("Undock request ACCEPTED");
    } else {
        LogMsg("Undock request REJECTED: not docked (state=%s)", state_str_[state_]);
    }
}

void MyPlane::RequestToggle() {
    if (state_ == CAN_DOCK || state_ == DOCKED) {
        toggle_requested_ = true;
        LogMsg("Toggle request ACCEPTED (state=%s)", state_str_[state_]);
    } else {
        LogMsg("Toggle request REJECTED: not in CAN_DOCK or DOCKED state (state=%s)", state_str_[state_]);
    }
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

    if (state_ == DOCKING || state_ == UNDOCKING) {
        for (auto& ajw : active_jws_)
            ajw.Reset();  // an animation might be ongoing
        state_ = IDLE;
        return;
    }

    if (auto_mode && state_ == SELECT_JWS)
        return;

    if (!auto_mode && state_ == CAN_DOCK) {
        for (auto& ajw : active_jws_)
            ajw.jw_->locked = false;
        active_jws_.resize(0);
        state_ = PARKED;
        return;
    }
}

void MyPlane::PlaneLoadedCb() {
    // reinit all that stuff as this may be a different plane now
    on_ground_ = 1;
    on_ground_ts_ = 0.0f;
    n_door_ = 0;
    use_engines_on_ = dont_connect_jetway_ = false;

    icao_.resize(4);
    XPLMGetDatab(acf_icao_dr_, icao_.data(), 0, 4);

    for (int i = 0; i < 4; i++)
        icao_[i] = (isupper((uint8_t)icao_[i]) || isdigit((uint8_t)icao_[i])) ? icao_[i] : ' ';

    float plane_cg_y = kF2M * XPLMGetDataf(acf_cg_y_dr_);
    float plane_cg_z = kF2M * XPLMGetDataf(acf_cg_z_dr_);

    float gear_z[2];
    if (2 == XPLMGetDatavf(acf_gear_z_dr_, gear_z, 0, 2)) {  // nose + main wheel
        nose_gear_z_ = -gear_z[0];
        main_gear_z_ = -gear_z[1];
    } else
        nose_gear_z_ = main_gear_z_ = plane_cg_z_;  // fall back to CG

    is_helicopter_ = XPLMGetDatai(is_helicopter_dr_);

    LogMsg("plane loaded: %s, is_helicopter: %d", icao_.c_str(), is_helicopter_);

    if (is_helicopter_)
        return;

    // check whether acf is listed in exception files
    use_engines_on_ = FindIcaoInFile(icao_, base_dir + "acf_use_engine_running.txt");
    dont_connect_jetway_ = FindIcaoInFile(icao_, base_dir + "acf_dont_connect_jetway.txt");

    // check whether door info is overridden in the config file
    auto it = door_info_map.find(icao_ + '1');
    if (it != door_info_map.end()) {
        door_info_[0] = it->second;
        LogMsg("using pos for door 1 from door_info_map: x: %0.2f, y: %0.2f, z: %0.2f", door_info_[0].x,
               door_info_[0].y, door_info_[0].z);
    } else {
        door_info_[0].x = XPLMGetDataf(acf_door_x_dr_);
        door_info_[0].y = XPLMGetDataf(acf_door_y_dr_);
        door_info_[0].z = XPLMGetDataf(acf_door_z_dr_);
    }

    n_door_ = 1;

    LogMsg("plane loaded: %s, plane_cg_y: %1.2f, plane_cg_z: %1.2f, door 1: x: %1.2f, y: %1.2f, z: %1.2f",
           icao_.c_str(), plane_cg_y, plane_cg_z, door_info_[0].x, door_info_[0].y, door_info_[0].z);

    // check whether single door mode is forced by config file
    bool single_door_only = FindIcaoInFile(icao_, base_dir + "acf_single_door.txt");

    // check for a second door, seems to be not available by dataref
    // data in the acf file is often bogus, so check our own config file first
    if (!single_door_only) {
        it = door_info_map.find(icao_ + '2');
        if (it != door_info_map.end()) {
            door_info_[1] = it->second;
            n_door_++;
            LogMsg("found door 2 in door_info_map: x: %0.2f, y: %0.2f, z: %0.2f", door_info_[1].x, door_info_[1].y,
                   door_info_[1].z);
        } else {
            LogMsg("door 2 is not defined in door_info_map");
        }

        // if nothing found in the config file try the acf
        if (n_door_ == 1) {
            char acf_path[512];
            char acf_file[256];

            XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
            LogMsg("acf path: '%s'", acf_path);

            FILE* acf = fopen(acf_path, "r");
            if (acf) {
                char line[200];
                int got = 0;
                int has_door2 = 0;
                // we go the simple brute force way
                while (fgets(line, sizeof(line), acf)) {
                    if (line == strstr(line, "P acf/_has_board_2 ")) {
                        if (1 != sscanf(line + 19, "%d", &has_door2))
                            break;
                    }

                    if (line == strstr(line, "P acf/_board_2/0 ")) {
                        if (1 == sscanf(line + 17, "%f", &door_info_[1].x)) {
                            door_info_[1].x *= kF2M;
                            got++;
                        }
                    }
                    if (line == strstr(line, "P acf/_board_2/1 ")) {
                        float y;
                        if (1 == sscanf(line + 17, "%f", &y)) {
                            door_info_[1].y = y * kF2M - plane_cg_y;
                            got++;
                        }
                    }
                    if (line == strstr(line, "P acf/_board_2/2 ")) {
                        float z;
                        if (1 == sscanf(line + 17, "%f", &z)) {
                            door_info_[1].z = z * kF2M - plane_cg_z;
                            got++;
                        }
                    }

                    if (has_door2 && got == 3) {
                        n_door_ = 2;
                        LogMsg("found door 2 in acf file: x: %0.2f, y: %0.2f, z: %0.2f", door_info_[1].x,
                               door_info_[1].y, door_info_[1].z);
                        break;
                    }
                }

                fclose(acf);
            }
        }
    } else {
        LogMsg("single door mode forced by acf_single_door.txt");
    }

    // SAM dgs don't like letters in pos 1-3
    if (icao_ == "A20N")
        icao_ = "A320";
    else if (icao_ == "A21N")
        icao_ = "A321";

    pax_no_dr_probed_ = false;
    pax_no_dr_ = nullptr;
    pax_no_ = 0;
}

void MyPlane::Update() {
    x_ = XPLMGetDataf(plane_x_dr_);
    y_ = XPLMGetDataf(plane_y_dr_);
    z_ = XPLMGetDataf(plane_z_dr_);
    psi_ = XPLMGetDataf(plane_true_psi_dr_);

    // on ground detection
    int og = (XPLMGetDataf(gear_fnrml_dr_) != 0.0);
    if (og != on_ground_ && now > on_ground_ts_ + 10.0f) {
        on_ground_ = og;
        on_ground_ts_ = now;
        LogMsg("transition to on_ground: %d", on_ground_);
    }

    // engines on
    engines_on_ = false;
    int er[8];
    int n = XPLMGetDatavi(eng_running_dr_, er, 0, 8);
    for (int i = 0; i < n; i++)
        if (er[i]) {
            engines_on_ = true;
            break;
        }

    // beacon
    if (use_engines_on_)
        beacon_on_ = engines_on_;
    else {
        // when checking the beacon guard against power transients when switching
        // to the APU generator (e.g. for the ToLiss fleet).
        // Report only state transitions when the new state persisted for 3 seconds

        int beacon = XPLMGetDatai(beacon_dr_);
        if (beacon) {
            if (!beacon_on_pending_) {
                beacon_on_ts_ = ::now;
                beacon_on_pending_ = true;
            } else if (now > beacon_on_ts_ + 3.0)
                beacon_on_ = true;
        } else {
            if (beacon_on_pending_) {
                beacon_off_ts_ = ::now;
                beacon_on_pending_ = false;
            } else if (now > beacon_off_ts_ + 3.0)
                beacon_on_ = false;
        }
    }

    parkbrake_set_ = (XPLMGetDataf(parkbrake_dr_) > 0.5f);
    elevation_ = XPLMGetDataf(plane_elevation_dr_);
    if (!pax_no_dr_probed_) {
        pax_no_dr_probed_ = true;
        pax_no_dr_ = XPLMFindDataRef("AirbusFBW/NoPax");  // currently only ToLiss
        if (pax_no_dr_) {
            LogMsg("ToLiss detected");
            int pax_no = XPLMGetDataf(pax_no_dr_) + 0.5f;
            if (pax_no > 0)  // warn on common user error
                LogMsg("WARNING: plane is already boarded with initial # of pax: %d", pax_no);
        }
    }

    if (pax_no_dr_)
        pax_no_ = XPLMGetDataf(pax_no_dr_) + 0.5f;
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
    if (icao_ != "A321")
        return;

    LogMsg("A321 detected, checking door config");
    if (auto door_dr = XPLMFindDataRef("AirbusFBW/A321ExitConfig")) {
        n_door_ = XPLMGetDatai(door_dr) == 0 ? 2 : 1;  // 0 = CLASSIC = 2 doors
        LogMsg("n_door from dataref: %d", n_door_);
    }
}

bool MyPlane::CheckTeleportation() {
    if (!on_ground())
        return false;

    if (parked_ngen_ != ::ref_gen || fabsf(parked_x_ - x_) > 1.0f || fabsf(parked_z_ - z_) > 1.0f) {
        LogMsg("parked_ngen: %d, ngen: %d, parked_x: %0.3f, x: %0.3f, parked_z: %0.3f, z: %0.3f", parked_ngen_,
               ::ref_gen, parked_x_, x_, parked_z_, z_);
        return true;
    }

    return false;
}

void MyPlane::ResetBeacon() {
    beacon_on_pending_ = 0;
    beacon_off_ts_ = beacon_on_ts_ = -10.0f;
}

static bool FindIcaoInFile(const std::string& acf_icao, const std::string& fn) {
    std::ifstream f(fn);
    if (f.is_open()) {
        LogMsg("check whether acf '%s' is in file %s", acf_icao.c_str(), fn.c_str());

        std::string line;
        line.reserve(50);
        while (std::getline(f, line)) {
            if (line.empty())
                continue;

            if (line.back() == '\r')
                line.pop_back();

            if (line == acf_icao) {
                LogMsg("found");
                return true;
            }
        }

        LogMsg("not found");
    }

    return false;
}
