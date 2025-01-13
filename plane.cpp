/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2025  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

*/

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <cassert>
#include <fstream>

#include "openSAM.h"
#include "XPLMPlanes.h"
#include "XPLMNavigation.h"

#include "plane.h"
#include "samjw.h"

MyPlane* my_plane;
std::vector<Plane*> mp_planes;

static const float kAnimInterval = -1;   // s for debugging or -1 for frame loop

const char * const Plane::state_str_[] = {
    "DISABLED", "IDLE", "PARKED", "SELECT_JWS", "CAN_DOCK",
    "DOCKING", "DOCKED", "UNDOCKING", "CANT_DOCK" };

// auto select active jetways
void
Plane::select_jws()
{
    if (n_door_ == 0)
        return;

    bool have_hard_match = false;
    for (auto & njw : nearest_jws_)
        if (!njw.soft_match_) {
            have_hard_match = true;
            break;
        }

    unsigned i_door = 0;
    unsigned i_jw = 0;
    while (i_jw < nearest_jws_.size()) {
        if (have_hard_match && nearest_jws_[i_jw].soft_match_)
            goto skip;

        // skip over collisions
        for (unsigned j = i_jw + 1; j < nearest_jws_.size(); j++)
            if (nearest_jws_[i_jw].collision_check(nearest_jws_[j]))
                goto skip;

        nearest_jws_[i_jw].door_ = i_door;
        active_jws_.push_back(nearest_jws_[i_jw]);
        log_msg("active jetway for door %d: %s", i_door, active_jws_.back().jw_->name);
        i_door++;
        if (i_door >= n_door_)
            break;

      skip:
        i_jw++;
    }

    if (active_jws_.size() == 0)
        log_msg("Oh no, no active jetways left in select_jws()!");
}

// the state machine called from the flight loop
float
Plane::jw_state_machine()
{
    if (state_ == DISABLED) {
        state_machine_next_ts_ = ::now + 2.0f;
        return 2.0f;
    }

    if (state_machine_next_ts_ > ::now)         // action is not due
        return state_machine_next_ts_ - ::now;

    State new_state{state_};

    if (state_ > IDLE && my_plane->check_teleportation()) {
        log_msg("teleportation detected!");
        state_ = new_state = IDLE;

        for (auto & ajw : active_jws_)
            ajw.reset();

        nearest_jws_.resize(0);
        active_jws_.resize(0);
        SamJw::reset_all();
    }

    unsigned n_done;

    switch (state_) {
        case IDLE:
            if (prev_state_ != IDLE) {
                active_jws_.resize(0);
                nearest_jws_.resize(0);
            }

            if (on_ground_ && !beacon_on_) {
                if (is_myplane()) {
                    // memorize position teleportation detection
                    memorize_parked_pos();

                    // reset stale command invocations
                    dock_requested = undock_requested = toggle_requested = 0;
                }

                new_state = PARKED;
            }
            break;

        case PARKED: {
                if (JwCtrl::find_nearest_jws(this, nearest_jws_))
                    new_state = SELECT_JWS;
                else
                    new_state = CANT_DOCK;
            }
            break;

        case SELECT_JWS:
            if (beacon_on_) {
                log_msg("SELECT_JWS and beacon goes on");
                new_state = IDLE;
                break;
            }

            // mp planes always have auto_select_jws
            if (!is_myplane() || ::auto_select_jws) {
                select_jws();
            } else if (prev_state_ != state_) {
                ui_unlocked = 1;    // allow jw selection in the ui
                update_ui(1);
            }

            // or wait for GUI selection
            if (active_jws_.size()) {
                for (auto & ajw : active_jws_) {
                    log_msg("setting up active jw for door: %d", ajw.door_);
                    ajw.setup_for_door(this, door_info_[ajw.door_]);

                    if (ajw.door_ == 0) // slightly slant towards the nose cone for door LF1
                        ajw.door_rot2_ += 3.0f;
                }

                new_state = CAN_DOCK;
            }
            break;

        case CAN_DOCK:
            if (beacon_on_) {
                log_msg("CAN_DOCK and beacon goes on");
                new_state = IDLE;
            }

            // mp planes always dock directly
            if (!is_myplane() || 1 == ::dock_requested || ::toggle_requested) {
                log_msg("docking requested");
                float start_ts = now;
                for (auto & ajw : active_jws_) {
                    // staggered start for docking low to high
                    ajw.setup_dock_undock(start_ts);
                    start_ts += 5.0f;
                }

                new_state = DOCKING;
            }
            break;

        case CANT_DOCK:
            if (!on_ground_ || beacon_on_) {
                new_state = IDLE;
                break;
            }
            break;

        case DOCKING:
            n_done = 0;
            for (auto & ajw : active_jws_) {
                if (ajw.dock_drive())
                    n_done++;
            }

            if (n_done == active_jws_.size()) {
                XPLMCommandRef cmdr = XPLMFindCommand("openSAM/post_dock");
                if (cmdr)
                    XPLMCommandOnce(cmdr);
                new_state = DOCKED;
            }
            else {
                state_machine_next_ts_ = 0.0f;
                return kAnimInterval;
            }
            break;

        case DOCKED:
            if (!on_ground_) {
                new_state = IDLE;
                break;
            }

            if (beacon_on_)
                log_msg("DOCKED and beacon goes on");

            if (beacon_on_ ||
                (is_myplane() && (1 == ::undock_requested || ::toggle_requested))) {
                log_msg("undocking requested");

                float start_ts = now + active_jws_.size() * 5.0f;
                for (auto & ajw : active_jws_) {
                    // staggered start for undocking high to low
                    start_ts -= 5.0f;
                    ajw.setup_dock_undock(start_ts);
                }

                XPLMCommandRef cmdr = XPLMFindCommand("openSAM/pre_undock");
                if (cmdr)
                    XPLMCommandOnce(cmdr);

                new_state = UNDOCKING;
            }
            break;

        case UNDOCKING:
            n_done = 0;
            for (auto & ajw : active_jws_) {
                if (ajw.undock_drive())
                    n_done++;
            }

            if (n_done == active_jws_.size())
               new_state = IDLE;
            else {
                state_machine_next_ts_ = 0.0f;
                return kAnimInterval;
            }
            break;

        default:
            log_msg("Bad state %d", state_);
            new_state = DISABLED;
            break;
    }

    // we use an extra cycle for dock/undock for better integration with the UI
    if (is_myplane()) {
        ::dock_requested--;
        ::undock_requested--;
        ::toggle_requested = 0;
    }

    prev_state_ = state_;

    if (new_state != state_) {
        log_msg("jw state transition, plane: %d, %s -> %s, beacon: %d", id_,
                state_str_[state_], state_str_[new_state], beacon_on_);
        state_ = new_state;

        // from anywhere to idle nullifies all selections
        if (state_ == IDLE) {
            active_jws_.resize(0);
            nearest_jws_.resize(0);
        }

        if (is_myplane()) {
            ui_unlocked = 0;
            update_ui(1);
        }
        state_machine_next_ts_ = 0.0f;
        return -1;  // see you on next frame
    }

    state_machine_next_ts_ = ::now + 0.5f;
    return 0.5f;
}

static bool find_icao_in_file(const std::string& acf_icao, const std::string& fn);

MyPlane::MyPlane()
{
    assert (my_plane == nullptr);
    log_msg("constructing MyPlane");
    plane_x_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_x");
    plane_y_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_y");
    plane_z_dr_ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    plane_lat_dr_ = XPLMFindDataRef("sim/flightmodel/position/latitude");
    plane_lon_dr_ = XPLMFindDataRef("sim/flightmodel/position/longitude");
    plane_elevation_dr_= XPLMFindDataRef("sim/flightmodel/position/elevation");
    plane_true_psi_dr_ = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    plane_y_agl_dr_ = XPLMFindDataRef("sim/flightmodel2/position/y_agl");
    eng_running_dr_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
    beacon_dr_ = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    parkbrake_dr_ = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
    gear_fnrml_dr_ = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    is_helicopter_dr_  = XPLMFindDataRef("sim/aircraft2/metadata/is_helicopter");
    acf_icao_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    acf_cg_y_dr_ = XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    acf_cg_z_dr_ = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    acf_gear_z_dr_ = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    acf_door_x_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    acf_door_y_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    acf_door_z_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_door_z");
    acf_livery_path_dr_ = XPLMFindDataRef("sim/aircraft/view/acf_livery_path");

    icao_ = "0000";
    reset_beacon();
    state_ = IDLE;
}

void
MyPlane::plane_loaded()
{
    on_ground_ = 1;
    on_ground_ts_ = 0.0f;

    icao_.resize(4);
    XPLMGetDatab(acf_icao_dr_, icao_.data(), 0, 4);

    for (int i=0; i < 4; i++)
        icao_[i] = (isupper((uint8_t)icao_[i]) || isdigit((uint8_t)icao_[i])) ? icao_[i] : ' ';

    float plane_cg_y = F2M * XPLMGetDataf(acf_cg_y_dr_);
    float plane_cg_z = F2M * XPLMGetDataf(acf_cg_z_dr_);

    float gear_z[2];
    if (2 == XPLMGetDatavf(acf_gear_z_dr_, gear_z, 0, 2)) {      // nose + main wheel
        nose_gear_z_ = -gear_z[0];
        main_gear_z_ = -gear_z[1];
    } else
        nose_gear_z_ = main_gear_z_ = plane_cg_z_;         // fall back to CG

    is_helicopter_ = XPLMGetDatai(is_helicopter_dr_);

    use_engines_on_ = dont_connect_jetway_ = false;

    log_msg("plane loaded: %s, is_helicopter: %d",
            icao_.c_str(), is_helicopter_);

    if (is_helicopter_)
        return;

    // check whether acf is listed in exception files
    std::string line; line.reserve(200);
    if (find_icao_in_file(icao_, base_dir + "acf_use_engine_running.txt")) {
        use_engines_on_ = true;
        log_msg("found");
    }

    if (find_icao_in_file(icao_, base_dir + "acf_dont_connect_jetway.txt")) {
        dont_connect_jetway_ = true;
        log_msg("found");
    }

    door_info_[0].x = XPLMGetDataf(acf_door_x_dr_);
    door_info_[0].y = XPLMGetDataf(acf_door_y_dr_);
    door_info_[0].z = XPLMGetDataf(acf_door_z_dr_);

    n_door_ = 1;

    log_msg("plane loaded: %s, plane_cg_y: %1.2f, plane_cg_z: %1.2f, "
            "door 1: x: %1.2f, y: %1.2f, z: %1.2f",
            icao_.c_str(), plane_cg_y, plane_cg_z,
            door_info_[0].x, door_info_[0].y, door_info_[0].z);

    // check for a second door, seems to be not available by dataref
    // data in the acf file is often bogus, so check our own config file first
    try {
        door_info_[1] = door_info_map.at(icao_ + '2');
        n_door_++;
        log_msg("found door 2 in door_info_map: x: %0.2f, y: %0.2f, z: %0.2f",
                door_info_[1].x, door_info_[1].y, door_info_[1].z);
    }
    catch(const std::out_of_range& ex) {
        log_msg("door 2 is not defined in door_info_map");
    }

    // if nothing found in the config file try the acf
    if (n_door_ == 1) {
        char acf_path[512];
        char acf_file[256];

        XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
        log_msg("acf path: '%s'", acf_path);

        FILE *acf = fopen(acf_path, "r");
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
                        door_info_[1].x *= F2M;
                        got++;
                    }
                }
                if (line == strstr(line, "P acf/_board_2/1 ")) {
                    float y;
                    if (1 == sscanf(line + 17, "%f", &y)) {
                        door_info_[1].y = y * F2M - plane_cg_y;
                        got++;
                    }
                }
                if (line == strstr(line, "P acf/_board_2/2 ")) {
                    float z;
                    if (1 == sscanf(line + 17, "%f", &z)) {
                        door_info_[1].z = z * F2M - plane_cg_z;
                        got++;
                    }
                }

                if (has_door2 && got == 3) {
                    n_door_ = 2;
                    log_msg("found door 2 in acf file: x: %0.2f, y: %0.2f, z: %0.2f",
                            door_info_[1].x, door_info_[1].y, door_info_[1].z);
                    break;
                }
            }

            fclose(acf);
        }
    }

    // SAM dgs don't like letters in pos 1-3
    if (icao_ == "A20N")
        icao_ ="A320";
}

void MyPlane::livery_loaded()
{
    // check ToLiss A321 door config
    if (icao_ != "A321")
        return;

    log_msg("A321 detected, checking door config");

    char path[512];
    strcpy(path, xp_dir.c_str());
    int len = strlen(path);
    int n = XPLMGetDatab(acf_livery_path_dr_, path + len, 0, sizeof(path) - len - 50);
    path[len + n] = '\0';
    strcat(path, "livery.tlscfg");
    log_msg("tlscfg path: '%s'", path);

    FILE *f = fopen(path, "r");
    if (f) {
        char line[150];
        line[sizeof(line) - 1] = '\0';
        while (fgets(line, sizeof(line) - 1, f)) {
            if (NULL != strstr(line, "exit_Configuration")) {
                if (NULL == strstr(line, "CLASSIC")) {
                    log_msg("door != CLASSIC, setting n_door to 1");
                    n_door_ = 1;
                }
                break;
            }
        }

        fclose(f);
    }
}

void
MyPlane::update()
{
    x_ = XPLMGetDataf(plane_x_dr_);
    y_ = XPLMGetDataf(plane_y_dr_);
    z_ = XPLMGetDataf(plane_z_dr_);
    psi_=XPLMGetDataf(plane_true_psi_dr_);

    // on ground detection
    int og = (XPLMGetDataf(gear_fnrml_dr_) != 0.0);
    if (og != on_ground_ && now > on_ground_ts_ + 10.0f) {
        on_ground_ = og;
        on_ground_ts_ = now;
        log_msg("transition to on_ground: %d", on_ground_);
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
            if (! beacon_on_pending_) {
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
}

void
MyPlane::memorize_parked_pos()
{
    parked_x_ = x_;
    parked_z_ = z_;
    parked_ngen_ = ::ref_gen;

    // find airport I'm on now to ease debugging
    float lat = this->lat();
    float lon = this->lon();

    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        char airport_id[50];
        XPLMGetNavAidInfo(ref, NULL, NULL, NULL, NULL, NULL, NULL, airport_id,
                NULL, NULL);
        log_msg("parked on airport: %s, lat,lon: %0.5f,%0.5f", airport_id, lat, lon);
    }
}

bool
MyPlane::check_teleportation()
{
	if (! on_ground())
		return false;

    if (parked_ngen_ != ::ref_gen || fabsf(parked_x_ - x_) > 1.0f || fabsf(parked_z_ - z_) > 1.0f) {
        log_msg("parked_ngen: %d, ngen: %d, parked_x: %0.3f, x: %0.3f, parked_z: %0.3f, z: %0.3f",
                parked_ngen_, ::ref_gen, parked_x_, x_, parked_z_, z_);
        return true;
    }

    return false;
}


void
MyPlane::reset_beacon()
{
    beacon_on_pending_ = 0;
    beacon_off_ts_ = beacon_on_ts_ = -10.0f;
}


// hook for the ui
void
MyPlane::auto_mode_change()
{
    if (state_ == DOCKING || state_ == UNDOCKING) {
        for (auto & ajw : active_jws_)
            ajw.reset();    // an animation might be ongoing
        state_ = IDLE;
    } else if (state_ == SELECT_JWS || state_ == CAN_DOCK) {
        state_ = PARKED;
    }
}

// static
void
MyPlane::init()
{
    if (my_plane != nullptr)
        throw OsEx("MyPlane is already initialized");

    my_plane = new MyPlane();
}

static bool
find_icao_in_file(const std::string& acf_icao, const std::string& fn)
{
    std::ifstream f(fn);
    if (f.is_open()) {
        log_msg("check whether acf '%s' is in file %s", acf_icao.c_str(), fn.c_str());

        std::string line;
        while (std::getline(f, line)) {
            size_t i = line.find('\r');
            if (i != std::string::npos)
                line.resize(i);

            if (line.find(acf_icao) == 0) {
                log_msg("found acf %s in %s", acf_icao.c_str(), fn.c_str());
                return true;
           }
        }
    }

    return false;
}
