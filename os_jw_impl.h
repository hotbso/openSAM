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

typedef struct active_jw_ {
    sam_jw_t *jw;
    ajw_status_t state;

    // in door local coordinates
    float x, y, z, psi;
    float dist;         // distance to door

    // target cabin position with corresponding dref values
    float tgt_x, tgt_rot1, tgt_rot2, tgt_rot3, tgt_extent;
    float ap_x;      // alignment point abeam door
    float parked_x, parked_z;

    double cabin_x, cabin_z;

    int wait_wb_rot;    // waiting for wheel base rotation
    float wb_rot;       // to this angle

    float last_step_ts;
    float timeout;      // so we don't get stuck

    FMOD_CHANNEL *alert_chn;
} active_jw_t;

#define NEAR_JW_LIMIT 2 // max # of jetways we consider for docking
#define MAX_NEAREST 10  // max # jetways / door we consider as nearest

extern int n_active_jw;
extern active_jw_t active_jw[MAX_DOOR];

extern active_jw_t nearest_jw[MAX_DOOR][MAX_NEAREST];
extern int n_nearest[MAX_DOOR];

extern void jw_auto_mode_change(void);

// from os_read_wav.c
extern void read_wav(const char *fname, sound_t *sound);

// from os_ui.c
extern int ui_unlocked; // the ui is unlocked for jw_selection
extern void update_ui(int only_if_visible);

