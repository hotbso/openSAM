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
#ifndef _PLANE_H_
#define _PLANE_H_

#include "XPWidgets.h"

#include "jwctrl.h"

static const int kNearJwLimit{3};   // max # of jetways we consider for docking

//
// Generic class that provides all plane related values for jetway animation.
//
class Plane {
  public:
    enum State { DISABLED=0, IDLE, PARKED, SELECT_JWS, CAN_DOCK,
                 DOCKING, DOCKED, UNDOCKING, CANT_DOCK };

    static const char * const state_str_[];

  private:
    friend class MyPlane;
    friend class MpPlaneXPMP2;

    float state_machine_next_ts_{0};    // ts for the next run of the state machine
    State state_{DISABLED}, prev_state_{DISABLED};

  protected:
    bool beacon_on_{false}, engines_on_{false}, on_ground_{false}, parkbrake_set_{false};
    std::string icao_;
    float x_, y_, z_, psi_;

    std::vector<JwCtrl> active_jws_;
    std::vector<JwCtrl> nearest_jws_;

  public:
    int id_{0};    // id for logging

    // readonly use!
    unsigned n_door_{0};
    DoorInfo door_info_[kMaxDoor];

    Plane() {
        nearest_jws_.reserve(10);
        active_jws_.reserve(kMaxDoor);
    }

    // update internal state
    virtual void update() = 0;
    virtual void memorize_parked_pos() {} // for teleportation detection

    // position
    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }
    float psi() const { return psi_; }

    // general state
    virtual bool is_myplane() = 0;
    std::string& icao() { return icao_; }

    // detailed state
    bool on_ground() const{ return on_ground_; }
    bool beacon_on() const { return beacon_on_; }
    bool parkbrake_set() const { return parkbrake_set_; }
    bool engines_on() const { return engines_on_; }

    // cmd support
    virtual bool auto_mode() const = 0;
    virtual bool dock_requested() { return false; }
    virtual bool undock_requested() { return false; }
    virtual bool toggle_requested() { return false; }

    // auto select jetways
    void select_jws();

    // hook into flight loop
    float jw_state_machine();

    // UI support functions called from jw_state_machine()
    virtual void update_ui([[maybe_unused]] bool only_if_visible) {}
    virtual void lock_ui([[maybe_unused]] bool yes_no) {}
};

//
// Derived class that represents the XP-pilot's plane.
// Clearly there is exactly one instance of this class.
// It is accessible through "MyPlane* my_plane" .
//
class MyPlane : public Plane {
    XPLMDataRef plane_x_dr_, plane_y_dr_, plane_z_dr_,
           plane_lat_dr_, plane_lon_dr_, plane_elevation_dr_,
           plane_true_psi_dr_, plane_y_agl_dr_,
           beacon_dr_, eng_running_dr_, parkbrake_dr_, gear_fnrml_dr_,
           is_helicopter_dr_,
           acf_icao_dr_, acf_cg_y_dr_, acf_cg_z_dr_, acf_gear_z_dr_,
           acf_door_x_dr_, acf_door_y_dr_, acf_door_z_dr_, acf_livery_path_dr_;

    bool use_engines_on_;   // instead of beacon, e.g. Zibo

    int beacon_on_pending_;
    float beacon_off_ts_, beacon_on_ts_;

    float on_ground_ts_;

    float parked_x_, parked_z_;
    unsigned parked_ngen_;

    bool auto_mode_, dock_requested_, undock_requested_, toggle_requested_;
    bool ui_unlocked_{false}; // the ui is unlocked for jw_selection

  public:
    static void init();         // call once

    // readonly use!
    bool dont_connect_jetway_;                       // after beacon off, e.g. Zibo
    bool is_helicopter_;
    float nose_gear_z_, main_gear_z_, plane_cg_z_;   // z value of plane's gears and cg

    MyPlane();

    void plane_loaded();        // called from XPLM_MSG_PLANE_LOADED handler
    void livery_loaded();       // called from  XPLM_MSG_LIVERY_LOADED handler

    void update() override ;    // called by flight loop, check beacon, on_ground, ....
    void reset_beacon();

    void memorize_parked_pos() override ; // for teleportation detection
    bool check_teleportation();

    bool is_myplane() override { return true; }

    float lat() { return XPLMGetDataf(plane_lat_dr_); }
    float lon() { return XPLMGetDataf(plane_lon_dr_); }
    float elevation() { return XPLMGetDataf(plane_elevation_dr_); }

    float y_agl() { return XPLMGetDataf(plane_y_agl_dr_); }

    // UI support
    void update_ui(bool only_if_visible) override;
    void lock_ui(bool yes_no) override { ui_unlocked_ = !yes_no; }
    static int ui_widget_cb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2);

    // cmd support
    void auto_mode_set(bool auto_mode);
    bool auto_mode() const override { return auto_mode_; }

    void request_dock();
    void request_undock();
    void request_toggle();

    bool dock_requested() override;
    bool undock_requested() override;
    bool toggle_requested() override;

    // dataref accessors
    static int jw_status_acc(void *ref);
    static int jw_door_status_acc(XPLMDataRef ref, int *values, int ofs, int n);

};

extern std::vector<Plane*> mp_planes;
extern MyPlane* my_plane;
#endif
