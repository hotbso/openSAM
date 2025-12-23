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

int Plane::id_base_;

static const float kAnimInterval = -1;   // s for debugging or -1 for frame loop

const char * const Plane::state_str_[] = {
    "DISABLED", "IDLE", "PARKED", "SELECT_JWS", "CAN_DOCK",
    "DOCKING", "DOCKED", "UNDOCKING", "CANT_DOCK" };

Plane::~Plane()
{
    LogMsg("pid=%02d, Plane destructor, state: %s, active_jws: %d", id_, state_str_[state_], (int)active_jws_.size());
    if (IDLE <= state_) {
        for (auto & ajw : active_jws_)
            ajw.Reset();
        active_jws_.resize(0);
    }

    LogMsg("pid=%02d, Plane destructor finished", id_);
}

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
            if (nearest_jws_[i_jw].CollisionCheck(nearest_jws_[j]))
                goto skip;

        nearest_jws_[i_jw].door_ = i_door;
        nearest_jws_[i_jw].selected_ = true;
        active_jws_.push_back(nearest_jws_[i_jw]);
        LogMsg("active jetway for door %d: %s", i_door, active_jws_.back().jw_->name);
        i_door++;
        if (i_door >= n_door_)
            break;

      skip:
        i_jw++;
    }

    if (active_jws_.size() == 0)
        LogMsg("Oh no, no active jetways left in select_jws()!");
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

    if (state_ > IDLE && check_teleportation()) {
        LogMsg("teleportation detected!");
        state_ = new_state = IDLE;
        state_change_ts_ = now;

        for (auto & ajw : active_jws_)
            ajw.Reset();

        nearest_jws_.resize(0);
        active_jws_.resize(0);
        SamJw::ResetAll();
    }

    unsigned n_done;

    switch (state_) {
        case IDLE:
            if (prev_state_ != IDLE) {
                 for (auto & ajw : active_jws_)
                    ajw.Reset();

                active_jws_.resize(0);
                nearest_jws_.resize(0);
            }

            if (on_ground_ && !beacon_on_) {
                // memorize position teleportation detection
                memorize_parked_pos();

                // reset stale command invocations
                dock_requested();
                undock_requested();
                toggle_requested();

                new_state = PARKED;
            }
            break;

        case PARKED:
            if (JwCtrl::FindNearestJetway(*this, nearest_jws_))
                new_state = SELECT_JWS;
            else
                new_state = CANT_DOCK;
            break;

        case SELECT_JWS:
            if (beacon_on_) {
                LogMsg("SELECT_JWS and beacon goes on");
                new_state = IDLE;
                break;
            }

            if (auto_mode()) {
                select_jws();
                if (active_jws_.size() == 0) {       // e.g. collisions
                    new_state = CANT_DOCK;
                    break;
                }
            } else if (prev_state_ != state_) {
                lock_ui(false);     // allow jw selection in the ui (if the plane supports it)
                update_ui(true);
            }

            // or wait for GUI selection
            if (active_jws_.size()) {
                for (auto & ajw : active_jws_) {
                    LogMsg("pid=%d, setting up active jw for door: %d", id_, ajw.door_);
                    ajw.SetupForDoor(*this, door_info_[ajw.door_]);

                    if (ajw.door_ == 0) // slightly slant towards the nose cone for door LF1
                        ajw.door_rot2_ += 3.0f;
                }

                // unlock jws that were not selected as active jw
                for (auto & njw : nearest_jws_)
                    if (! njw.selected_)
                        njw.jw_->locked = false;

                new_state = CAN_DOCK;
            }
            break;

        case CAN_DOCK:
            if (beacon_on_) {
                LogMsg("CAN_DOCK and beacon goes on");
                new_state = IDLE;
            }

            // mp planes always dock directly
            if (dock_requested() || toggle_requested()) {
                LogMsg("pid=%02d, docking requested", id_);
                float start_ts = now;
                for (auto & ajw : active_jws_) {
                    // staggered start for docking low to high
                    ajw.SetupDockUndock(start_ts, with_alert_sound());
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
                if (ajw.DockDrive())
                    n_done++;
            }

            if (n_done == active_jws_.size()) {
                if (call_pre_post_dock_cmd()) {
                    XPLMCommandRef cmdr = XPLMFindCommand("openSAM/post_dock");
                    if (cmdr)
                        XPLMCommandOnce(cmdr);
                }
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
                LogMsg("pid=%d, DOCKED and beacon goes on", id_);

            if (beacon_on_ || undock_requested() || toggle_requested()) {
                LogMsg("undocking requested");

                float start_ts = now + active_jws_.size() * 5.0f;
                for (auto & ajw : active_jws_) {
                    // staggered start for undocking high to low
                    start_ts -= 5.0f;
                    ajw.SetupDockUndock(start_ts, with_alert_sound());
                }

                if (call_pre_post_dock_cmd()) {
                    XPLMCommandRef cmdr = XPLMFindCommand("openSAM/pre_undock");
                    if (cmdr)
                        XPLMCommandOnce(cmdr);
                }

                new_state = UNDOCKING;
            }
            break;

        case UNDOCKING:
            n_done = 0;
            for (auto & ajw : active_jws_) {
                if (ajw.UndockDrive())
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
            LogMsg("Bad state %d", state_);
            new_state = DISABLED;
            break;
    }

    prev_state_ = state_;

    if (new_state != state_) {
        state_change_ts_ = now;
        LogMsg("pid=%02d, jw state transition, %s -> %s, beacon: %d", id_,
                state_str_[state_], state_str_[new_state], beacon_on_);
        state_ = new_state;

        // from anywhere to idle nullifies all selections
        if (state_ == IDLE) {
            for (auto & ajw : active_jws_)
                ajw.Reset();
            active_jws_.resize(0);
            nearest_jws_.resize(0);
        }

        lock_ui(true);
        update_ui(true);

        state_machine_next_ts_ = 0.0f;
        return -1;  // see you on next frame
    }

    state_machine_next_ts_ = ::now + 0.5f;
    return 0.5f;
}

