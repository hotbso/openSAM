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

#include <math.h>

#define XPLM200
#define XPLM210
#define XPLM300
#define XPLM301
#define XPLM400

#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#include "XPLMNavigation.h"

#define UNUSED(x) (void)x

static const float D2R = M_PI/180.0;
static const float F2M = 0.3048;	    // 1 ft [m]
static const float LAT_2_M = 111120;    // 1° lat in m
static const float FAR_SKIP = 5000;     // don't consider jetways farther than that
static const float NEAR_SKIP = 2;       // don't consider jetways farther than that

typedef struct _sam_jw sam_jw_t;
typedef struct _sam_dgs sam_dgs_t;
typedef struct _stand stand_t;

typedef struct _scenery {
    const char *name;

    sam_jw_t *sam_jws;
    int n_sam_jws;

    stand_t *stands;
    int n_stands;

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for FAR_SKIP */
} scenery_t;

extern scenery_t *sceneries;
extern int n_sceneries;

typedef struct door_info_ {
    float x, y, z;
} door_info_t;

#define MAX_DOOR 3

extern int n_door;
extern door_info_t door_info[MAX_DOOR];
extern float plane_nw_z, plane_mw_z, plane_cg_z;   // z value of plane's 0 to fw, mw and cg
extern char acf_icao[];

extern char base_dir[512];          // base directory of openSAM
extern int use_engine_running;      // instead of beacon, e.g. MD11
extern int dont_connect_jetway;     // e.g. for ZIBO with own ground service

extern int beacon_state, beacon_last_pos;   // beacon state, last switch_pos, ts of last switch actions
extern float beacon_off_ts, beacon_on_ts;
extern float parked_x, parked_y;
extern  int parked_ngen;

extern XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, plane_y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr, acf_gear_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr, acf_livery_path,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

extern uint64_t stat_sc_far_skip, stat_far_skip, stat_near_skip,
    stat_acc_called, stat_jw_match, stat_dgs_acc, stat_dgs_acc_last;

extern float now;           // current timestamp
extern int on_ground;
extern float lat_ref, lon_ref;
// generation # of reference frame
// init with 1 so jetways never seen by the accessor won't be considered in find_dockable_jws()
extern unsigned int ref_gen;

extern int dock_requested, undock_requested, toggle_requested;
extern int auto_select_jws;

// functions
extern void log_msg(const char *fmt, ...);
extern int collect_sam_xml(const char *xp_dir);
extern int check_beacon(void);
extern int check_teleportation(void);

extern void toggle_ui(void);

/* helpers */
#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define BETWEEN(x ,a ,b) ((a) <= (x) && (x) <= (b))

static inline
float RA(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        return angle - 360.0f;

    if (angle <= -180.0f)
        return angle + 360.0f;

    return angle;
}

static inline float
clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* norm-2 length */
static inline float
len2f(float x, float y)
{
    return sqrtf(x * x + y * y);
}

