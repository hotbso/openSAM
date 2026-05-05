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

#ifndef _MY_PLANE_H_
#define _MY_PLANE_H_

#include "plane.h"
#include "dgs/plane.h"

//
// Derived class that represents the XP-pilot's plane.
// Clearly there is exactly one instance of this class.
// It is accessible through "std::unique_ptr<MyPlane> my_plane" .
//
class MyPlane : public Plane, public dgs::Plane {
    XPLMDataRef plane_lat_dr_, plane_lon_dr_, plane_y_agl_dr_;

    float on_ground_ts_;
    // for teleportation detection
    float parked_x_, parked_z_;
    unsigned parked_ngen_{};

    bool auto_mode_{}, dock_requested_{}, undock_requested_{}, toggle_requested_{};
    bool ui_unlocked_{};  // the ui is unlocked for jw_selection

    float elevation_{};

   public:
    MyPlane();
    ~MyPlane() override {}

    void PlaneLoadedCb();  // called from XPLM_MSG_PLANE_LOADED handler

    // update internal state
    void Update();

    void MemorizeParkedPos() override;  // for teleportation detection
    bool CheckTeleportation() override;

    // these 3 are called without prior update() call -> direct read from drefs
    float lat() { return XPLMGetDataf(plane_lat_dr_); }

    float lon() { return XPLMGetDataf(plane_lon_dr_); }

    float y_agl() { return XPLMGetDataf(plane_y_agl_dr_); }

    float elevation() const { return elevation_; }

    // UI support
    void UpdateUI(bool only_if_visible) override;
    void LockUI(bool yes_no) override { ui_unlocked_ = !yes_no; }
    static int UIWidgetCb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2);

    bool with_alert_sound() override { return true; }

    // cmd support
    void AutoModeSet(bool auto_mode);
    bool auto_mode() const override { return auto_mode_; }

    bool call_pre_post_dock_cmd() override { return true; }

    void RequestDock();
    void RequestUndock();
    void RequestToggle();

    bool dock_requested() override;
    bool undock_requested() override;
    bool toggle_requested() override;

    // dataref accessors
    static int JwStatusAcc(void* ref);
    static int JwDoorStatusAcc(void* ref, int* values, int ofs, int n);
};

extern std::shared_ptr<MyPlane> my_plane;
#endif
