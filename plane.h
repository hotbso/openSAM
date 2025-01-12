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

#include "jwctrl.h"

static const int kNearJwLimit{3};   // max # of jetways we consider for docking
static const int kMaxNearest{10};   // max # jetways / door we consider as nearest

//
// Generic class that provides all plane related values for jetway animation.
//
class Plane {
  friend class MyPlane;
  friend class MpPlane;

  public:
    enum State { DISABLED=0, IDLE, PARKED, SELECT_JWS, CAN_DOCK,
                 DOCKING, DOCKED, UNDOCKING, CANT_DOCK };

    // readonly use!
    float x_, y_, z_, psi_;
    int n_door_;
    DoorInfo door_info_[kMaxDoor];

    std::vector<JwCtrl> active_jws_;
    int n_nearest_jws_;
    std::array<JwCtrl, kMaxNearest> nearest_jws_;

    Plane(): n_door_(0), state_(IDLE), prev_state_(DISABLED) {}

    // update internal state
    virtual void update() = 0;
    virtual void memorize_parked_pos() {} // for teleportation detection

    // general state
    virtual bool is_myplane() = 0;
    virtual std::string& icao() = 0;

    // detailed state
    virtual bool on_ground() = 0;
    virtual bool beacon_on() = 0;
    virtual bool parkbrake_set() = 0;
    virtual bool engines_on() = 0;

    State state_, prev_state_;
    float jw_state_machine();
};

//
// Derived class that represents the XP-pilot's plane.
// Clearly there is exactly one instance of this class.
// It is accessible through "MyPlane* my_plane" or as generic plane as "planes[0]".
//
class MyPlane : public Plane {
    XPLMDataRef plane_x_dr_, plane_y_dr_, plane_z_dr_,
           plane_lat_dr_, plane_lon_dr_, plane_elevation_dr_,
           plane_true_psi_dr_, plane_y_agl_dr_,
           beacon_dr_, eng_running_dr_, parkbrake_dr_, gear_fnrml_dr_,
           is_helicopter_dr_,
           acf_icao_dr_, acf_cg_y_dr_, acf_cg_z_dr_, acf_gear_z_dr_,
           acf_door_x_dr_, acf_door_y_dr_, acf_door_z_dr_, acf_livery_path_dr_;

    std::string icao_;

    bool use_engines_on_;   // instead of beacon, e.g. Zibo

    bool beacon_on_;
    int beacon_on_pending_;
    float beacon_off_ts_, beacon_on_ts_;

    int on_ground_;
    float on_ground_ts_;

    float parked_x_, parked_z_;
    unsigned parked_ngen_;

  public:
    static void init();         // call once

    // readonly use!
    bool dont_connect_jetway_;                       // after beacon off, e.g. Zibo
    bool is_helicopter_;
    float nose_gear_z_, main_gear_z_, plane_cg_z_;   // z value of plane's gears and cg

    MyPlane();

    void plane_loaded();        // called from XPLM_MSG_PLANE_LOADED handler
    void livery_loaded();       // called from  XPLM_MSG_LIVERY_LOADED handler

    void update();              // called by flight loop, check beacon, on_ground, ....
    void reset_beacon();

    void memorize_parked_pos(); // for teleportation detection
    bool check_teleportation();

    bool is_myplane() { return true; }
    std::string& icao() { return icao_; }

    float lat() { return XPLMGetDataf(plane_lat_dr_); }
    float lon() { return XPLMGetDataf(plane_lon_dr_); }
    float elevation() { return XPLMGetDataf(plane_elevation_dr_); }

    float y_agl() { return XPLMGetDataf(plane_y_agl_dr_); }

    bool on_ground() { return on_ground_; }
    bool beacon_on() { return beacon_on_; }
    bool parkbrake_set() { return (XPLMGetDataf(parkbrake_dr_) > 0.5f); }
    bool engines_on();

    void auto_mode_change(); // hook for the ui
};

extern std::vector<Plane*> planes;
extern MyPlane* my_plane;

// from os_ui.c
extern int ui_unlocked; // the ui is unlocked for jw_selection
extern void update_ui(int only_if_visible);
#endif
