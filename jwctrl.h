//
//    openSAM: open source SAM emulator for X Plane
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

#ifndef _JWCTRL_H
#define _JWCTRL_H

#include <array>

#include "XPLMSound.h"

struct Sound {
    void *data;
    int size;
    int num_channels;
    int sample_rate;
};

class Plane;

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
        PARKED,
        TO_AP, AT_AP, TO_DOOR, DOCKED,  // sequence for docking

        // TO_AP,                       // sequence for undocking
        TO_PARK };

    bool selected_;     // nearest jw was selected as an active jw
    int door_;          // active JwCtrl associated with door #
    SamJw *jw_;
    JwCtrlState state_;

    // in door local coordinates
    float x_, y_, z_, psi_;

    int soft_match_;     // does not really fulfill matching criteria

    // target cabin position with corresponding dref values
    float door_x_, door_rot1_, door_rot2_, door_rot3_, door_extent_;
    float ap_x_;      // alignment point abeam door
    float parked_x_, parked_z_;

    double cabin_x_, cabin_z_;

    bool wait_wb_rot_;    // waiting for wheel base rotation
    float wb_rot_;       // to this angle

    float start_ts_;     // actually start operation if now > start_ts_
    float last_step_ts_;
    float timeout_;      // so we don't get stuck

    FMOD_CHANNEL *alert_chn_;

    void SetupForDoor(Plane& plane, const DoorInfo& door_info);

    // convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 are optional
    void XzToSamDref(float cabin_x, float cabin_z,
                      float& rot1, float& extent, float *rot2, float *rot3);

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
    // find nearest jetways, order by z (= door number, hopefully)
    static int FindNearestJetway(Plane& plane, std::vector<JwCtrl>& nearest_jws);

    // check whether extended nearest njw would crash into parked njw2
    bool CollisionCheck(const JwCtrl &njw2);

    // check whether two jetways collide when both extend to their doors
    bool CollisionCheckExtended(const JwCtrl &njw2) const;

    // setup for operation
    void SetupDockUndock(float start_time, bool with_sound);

    // drive jetway, return true when done
    bool DockDrive();
    bool UndockDrive();

    void Reset();

    friend bool operator<(const JwCtrl&, const JwCtrl&);

    // static initialization hooks, call once
    static void SoundInit();       // inits device and loads wav
    static void Init();             // registers dref accessors
};

// from ReadWav.cpp
extern void ReadWav(const std::string& fname, Sound& sound);
#endif
