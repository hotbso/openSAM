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

#ifndef _PLANE_H_
#define _PLANE_H_

#include <unordered_map>
#include <memory>

#include "jwctrl.h"

static constexpr int kNearJwLimit = 3;     // max # of jetways we consider for docking
static constexpr float kMpMaxDist = 2000;  // (m) max dist we consider MP planes

//
// Generic class that provides all plane related values for jetway animation.
//
class Plane {
   public:
    enum State { DISABLED = 0, IDLE, PARKED, SELECT_JWS, CAN_DOCK, DOCKING, DOCKED, UNDOCKING, CANT_DOCK };

    static const char* const state_str_[];

   protected:
    float state_machine_next_ts_{0};  // ts for the next run of the state machine
    State state_{DISABLED}, prev_state_{DISABLED};
    float state_change_ts_{0};

    bool beacon_on_{false}, engines_on_{false}, on_ground_{false}, parkbrake_set_{false};
    std::string icao_;
    float x_, y_, z_, psi_;

    std::vector<JwCtrl> active_jws_;
    std::vector<JwCtrl> nearest_jws_;
    int nearest_jws_seqno_{0};  // for detecting changes in the nearest jetway list by the UI

    static int id_base_;

   public:
    const int id_;  // id for logging

    // loaded for my_plane on start, updated on the fly for MP planes
    unsigned n_doors_{0};
    DoorInfo door_info_[kMaxDoor];

    Plane() : id_(id_base_++) {
        nearest_jws_.reserve(10);
        active_jws_.reserve(kMaxDoor);
    }

    virtual ~Plane() = 0;

    // general state
    State state() { return state_; }
    std::string& icao() { return icao_; }

    // position
    float x() const { return x_; }

    float y() const { return y_; }

    float z() const { return z_; }

    float psi() const { return psi_; }

    // detailed state
    bool on_ground() const { return on_ground_; }

    bool beacon_on() const { return beacon_on_; }

    bool parkbrake_set() const { return parkbrake_set_; }

    bool engines_on() const { return engines_on_; }

    // cmd support
    virtual bool auto_mode() const = 0;
    virtual bool dock_requested() { return false; }

    virtual bool undock_requested() { return false; }

    virtual bool toggle_requested() { return false; }

    virtual bool call_pre_post_dock_cmd() { return false; }

    // in general no sound on (mass-) docking
    virtual bool with_alert_sound() { return (state_ == DOCKED); }

    virtual void MemorizeParkedPos() {}  // for teleportation detection

    virtual bool CheckTeleportation() { return false; }

    // auto select jetways
    void AutoSelectJws();

    // hook into flight loop
    float JwStateMachine();
};
#endif
