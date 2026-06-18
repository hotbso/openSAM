//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2025  Holger Teutsch
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

#pragma once

#include <string>
#include <vector>

#include "XPLMSound.h"

struct SamJw;
struct DoorInfo;

struct JwCtrlPlaneInfo {
    int id;
    float x, y, z, psi;
    std::vector<DoorInfo>& door_info;
};

struct Sound {
    void *data;
    int size;
    int num_channels;
    int sample_rate;
};

// JwCtrl
// The jetway controller is the glue between a plane and its doors and sam jetways.
// It has support functions to find appropriate jetways for a plane and does the animation
// by updating values for the animation datarefs in the corresponding SamJw class.

class JwCtrl {
   private:
    static Sound alert_;
    static bool SoundDevInit();

   public:
    enum JwCtrlState {
        kParked,
        kToAp,
        kAtAp,
        kToDoor,
        kAtDoor,
        kDocked,  // sequence for docking

        // kToAp,                       // sequence for undocking
        kToPark
    };

    bool selected_{false};  // nearest jw was selected as an active jw
    int door_{};       // active JwCtrl associated with door #
    SamJw* jw_;
    JwCtrlState state_{kParked};

    // everything in plane local coordinates
    float x_, y_, z_, psi_;

    int soft_match_{};  // does not really fulfill matching criteria

    // parked position of jw
    float parked_x_, parked_z_;

    // target cabin position with corresponding dref values
    // docked_x/z is a cabinLength abeam of the door
    float docked_x_, docked_z_, docked_y_, docked_rot1_, docked_rot2_, docked_rot3_, docked_extent_;
    float ap_x_, ap_z_;  // intermediate alignment point abeam docked_x/z

    double cabin_x_, cabin_z_;  // current position of cabin

    bool wait_wb_rot_;  // waiting for wheel base rotation
    float wb_rot_;      // to this angle

    float start_ts_;  // actually start operation if now > start_ts_
    float last_step_ts_;
    float timeout_;  // so we don't get stuck

    FMOD_CHANNEL* alert_chn_ = nullptr;

    void SetupForDoor(const DoorInfo& door_info);

    // convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 are optional
    void XzToSamDref(float cabin_x, float cabin_z, float& rot1, float& extent, float* rot2, float* rot3);

    //
    // animation
    //
   private:
    void Rotate1Extend();

    // rotation 2/3 to angle, return true when done
    bool Rotate2(float rot2, float dt);
    bool Rotate3(float rot3, float dt);

    bool RotateWheelBase(float dt);

    void AnimateWheels(float ds);

    // sound stuff
    void AlertOn();
    void AlertOff();
    void AlertSetpos();

   public:
    JwCtrl(const JwCtrl&) = default;
    JwCtrl& operator=(const JwCtrl&) = default;

    JwCtrl(SamJw* jw, const JwCtrlPlaneInfo& plane_info);
    ~JwCtrl();

    // find nearest jetways, order by z (= door number, hopefully)
    static int FindNearestJetways(const JwCtrlPlaneInfo& plane_info, std::vector<JwCtrl>& nearest_jws);

    // check whether extended nearest njw would crash into parked njw2
    bool CollisionCheck(const JwCtrl& njw2);

    // setup for operation
    void SetupDockUndock(float start_time, bool with_sound);

    // drive jetway, return true when done
    bool DockDrive();
    bool UndockDrive();

    void UnlockJw();           // unlock the jetway controled by this controller
    void ResetJw();              // reset the jetway controlled by this controller to the initial position
    const char* name() const;  // name of jetway controlled by this controller

    friend bool operator<(const JwCtrl&, const JwCtrl&);

    // static initialization hooks, call once
    static void SoundInit();  // inits device and loads wav
    static void Init();       // registers dref accessors
};

// from ReadWav.cpp
extern void ReadWav(const std::string& fname, Sound& sound);
