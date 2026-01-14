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

#include "XPWidgets.h"

#include "jwctrl.h"

static constexpr int kNearJwLimit = 3;      // max # of jetways we consider for docking
static constexpr float kMpMaxDist = 2000;   // (m) max dist we consider MP planes

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
    static int id_base_;

   public:
    const int id_;  // id for logging

    // readonly use!
    unsigned n_door_{0};
    DoorInfo door_info_[kMaxDoor];

    Plane() : id_(id_base_++) {
        nearest_jws_.reserve(10);
        active_jws_.reserve(kMaxDoor);
    }

    virtual ~Plane();

    // general state
    State state() {
        return state_;
    }
    std::string& icao() {
        return icao_;
    }

    // position
    float x() const {
        return x_;
    }
    float y() const {
        return y_;
    }
    float z() const {
        return z_;
    }
    float psi() const {
        return psi_;
    }

    // detailed state
    bool on_ground() const {
        return on_ground_;
    }
    bool beacon_on() const {
        return beacon_on_;
    }
    bool parkbrake_set() const {
        return parkbrake_set_;
    }
    bool engines_on() const {
        return engines_on_;
    }

    // cmd support
    virtual bool auto_mode() const = 0;
    virtual bool dock_requested() {
        return false;
    }
    virtual bool undock_requested() {
        return false;
    }
    virtual bool toggle_requested() {
        return false;
    }
    virtual bool call_pre_post_dock_cmd() {
        return false;
    }

    // in general no sound on (mass-) docking
    virtual bool with_alert_sound() {
        return (state_ == DOCKED);
    }

    virtual void MemorizeParkedPos() {
    }  // for teleportation detection
    virtual bool CheckTeleportation() {
        return false;
    }

    // auto select jetways
    void SelectJws();

    // hook into flight loop
    float JwStateMachine();

    // UI support functions called from JwStateMachine()
    virtual void UpdateUI([[maybe_unused]] bool only_if_visible) {
    }
    virtual void LockUI([[maybe_unused]] bool yes_no) {
    }
};

//
// Derived class that represents the XP-pilot's plane.
// Clearly there is exactly one instance of this class.
// It is accessible through "MyPlane* my_plane" .
//
class MyPlane : public Plane {
    XPLMDataRef plane_lat_dr_, plane_lon_dr_, plane_y_agl_dr_;
    XPLMDataRef pax_no_dr_;
    bool pax_no_dr_probed_;

    bool use_engines_on_;   // instead of beacon, e.g. Zibo

    // helpers for debouncing
    int beacon_on_pending_;
    float beacon_off_ts_, beacon_on_ts_;
    float on_ground_ts_;

    // for teleportation detection
    float parked_x_, parked_z_;
    unsigned parked_ngen_;

    bool auto_mode_, dock_requested_, undock_requested_, toggle_requested_;
    bool ui_unlocked_{false}; // the ui is unlocked for jw_selection

    float elevation_;
    int pax_no_;

  public:
    static void init();         // call once

    // readonly use!
    bool dont_connect_jetway_;                       // after beacon off, e.g. Zibo
    bool is_helicopter_;
    float nose_gear_z_, main_gear_z_, plane_cg_z_;   // z value of plane's gears and cg

    MyPlane();
    ~MyPlane() {}

    void PlaneLoadedCb();        // called from XPLM_MSG_PLANE_LOADED handler

    void ResetBeacon();

    // update internal state
    void Update();

    void MemorizeParkedPos() override ; // for teleportation detection
    bool CheckTeleportation() override;

    // these 3 are called without prior update() call -> direct read from drefs
    float lat() { return XPLMGetDataf(plane_lat_dr_); }
    float lon() { return XPLMGetDataf(plane_lon_dr_); }
    float y_agl() { return XPLMGetDataf(plane_y_agl_dr_); }

    float elevation() const { return elevation_; }
    int pax_no() const { return pax_no_; }

    // UI support
    void UpdateUI(bool only_if_visible) override;
    void LockUI(bool yes_no) override { ui_unlocked_ = !yes_no; }
    static int UIWidgetCb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2);

    bool with_alert_sound() override { return true; }

    // cmd support
    void auto_mode_set(bool auto_mode);
    bool auto_mode() const override { return auto_mode_; }
    bool call_pre_post_dock_cmd() override { return true; }

    void RequestDock();
    void RequestUndock();
    void RequestToggle();

    bool dock_requested() override;
    bool undock_requested() override;
    bool toggle_requested() override;

    // dataref accessors
    static int jw_status_acc(void *ref);
    static int jw_door_status_acc(XPLMDataRef ref, int *values, int ofs, int n);

};

extern MyPlane my_plane;
#endif
