/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch

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

#include "XPLMSound.h"

typedef struct _alert_sound {
    void *data;
    int size;
    int num_channels;
    int sample_rate;
} sound_t;

typedef enum ajw_status_e {
    AJW_PARKED,
    AJW_TO_AP, AJW_AT_AP, AJW_TO_DOOR, AJW_DOCKED,  // sequence for docking

    // AJW_TO_AP,                                   // sequence for undocking
    AJW_TO_PARK
} ajw_status_t;

// jetway controller
class JwCtrl {
   public:
    SamJw *jw;           // == NULL means empty
    ajw_status_t state;

    // in door local coordinates
    float x, y, z, psi;

    int soft_match;     // does not really fulfill matching criteria

    // target cabin position with corresponding dref values
    float door_x, door_rot1, door_rot2, door_rot3, door_extent;
    float ap_x;      // alignment point abeam door
    float parked_x, parked_z;

    double cabin_x, cabin_z;

    bool wait_wb_rot;    // waiting for wheel base rotation
    float wb_rot;       // to this angle

    float start_ts;     // actually start operation if now > start_ts
    float last_step_ts;
    float timeout;      // so we don't get stuck

    FMOD_CHANNEL *alert_chn;

    auto setup_for_door(const DoorInfo& door_info) -> void;

    // convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 are optional
    auto xz_to_sam_dr(float cabin_x, float cabin_z,
                      float& rot1, float& extent, float *rot2, float *rot3) -> void;

    //
    // animation
    //
  private:
    auto rotate_1_extend() -> void;

    // rotation 2/3 to angle, return true when done
    auto rotate_2(float rot2, float dt) -> bool;
    auto rotate_3(float rot3, float dt) -> bool;

    auto rotate_wheel_base(float dt) -> bool;

    auto animate_wheels(float ds) -> void;

  public:
    // drive jetway, return true when done
    auto dock_drive() -> bool;
    auto undock_drive() -> bool;

    // sound stuff
    auto alert_on() -> void;
    auto alert_off()-> void;
    auto alert_setpos() -> void;

    friend bool operator<(const JwCtrl&, const JwCtrl&);
};

#define NEAR_JW_LIMIT 3 // max # of jetways we consider for docking
#define MAX_NEAREST 10  // max # jetways / door we consider as nearest

extern int n_active_jw;
extern JwCtrl active_jw[kMaxDoor];

extern JwCtrl nearest_jw[MAX_NEAREST];
extern int n_nearest;

extern void jw_auto_mode_change(void);

// from os_read_wav.c
extern void read_wav(const std::string& fname, sound_t *sound);

// from os_ui.c
extern int ui_unlocked; // the ui is unlocked for jw_selection
extern void update_ui(int only_if_visible);

// from os_sound.c
extern sound_t alert;
extern int sound_init(void);
