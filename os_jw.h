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

struct _sam_jw  {

    // local x,z computed from the xml's lat/lon
    double xml_x, xml_y, xml_z;
    unsigned int xml_ref_gen;   // only valid if this matches the generation of the ref frame

    // values from the actually drawn object
    float x, y, z, psi;
    unsigned int obj_ref_gen;
    int library_id;

    // values fed to the datarefs
    float rotate1, rotate2, rotate3, extent, wheels,
          wheelrotatec, wheelrotater, wheelrotatel,
          warnlight;

    // these are from sam.xml
    int id;                         // only used for library jetway sets
    char name[40];
    char sound[40];

    float latitude, longitude, heading, height, wheelPos, cabinPos, cabinLength,
          wheelDiameter, wheelDistance,
          minRot1, maxRot1, minRot2, maxRot2, minRot3, maxRot3,
          minExtent, maxExtent, minWheels, maxWheels,
          initialRot1, initialRot2, initialRot3, initialExtent;
    int door; // 0 = LF1 or default, 1 = LF2

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   // bounding box for FAR_SKIP
};

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

// fortunately SAM3 is abandoned so this will never change 8-)
#define MAX_SAM3_LIB_JW 27  // index is 0..27
extern sam_jw_t sam3_lib_jw[];

extern int jw_init(void);
extern float jw_state_machine();

// from os_read_wav.c
extern void read_wav(const char *fname, sound_t *sound);

// from os_ui.c
extern int ui_unlocked; // the ui is unlocked for jw_selection
extern void update_ui(int only_if_visible);

