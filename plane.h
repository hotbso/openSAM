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

//
// Generic class that provides all plane related values for jetway animation.
//
class Plane {
    friend class MyPlane;
    friend class MpPlaneXPMP2;

    float state_machine_next_ts_;    // ts for the next run of the state machine

  protected:
    bool beacon_on_, engines_on_, on_ground_, parkbrake_set_;
    std::string icao_;
    float x_, y_, z_, psi_;

  public:
    int id_;    // id for logging

    enum State { DISABLED=0, IDLE, PARKED, SELECT_JWS, CAN_DOCK,
                 DOCKING, DOCKED, UNDOCKING, CANT_DOCK };

    static const char * const state_str_[];

    // readonly use!
    unsigned n_door_;
    DoorInfo door_info_[kMaxDoor];

    std::vector<JwCtrl> active_jws_;
    std::vector<JwCtrl> nearest_jws_;

    Plane(): state_machine_next_ts_(0), id_(0), n_door_(0), state_(DISABLED), prev_state_(DISABLED) {
        nearest_jws_.reserve(10);
        active_jws_.resize(kMaxDoor);
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
    bool parkbrake_set() const{ return parkbrake_set_; }
    bool engines_on() const { return engines_on_; }

    State state_, prev_state_;

    // auto select jetways
    void select_jws();

    float jw_state_machine();
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

    void auto_mode_change(); // hook for the ui
};

extern std::vector<Plane*> mp_planes;
extern MyPlane* my_plane;

// from os_ui.c
extern int ui_unlocked; // the ui is unlocked for jw_selection
extern void update_ui(int only_if_visible);
#endif
