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
#define XPLM300
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"

static const float D2R = M_PI/180.0;
static const float F2M = 0.3048;	/* 1 ft [m] */
static const float LAT_2_M = 111120;    /* 1° lat in m */
static const float FAR_SKIP = 5000; /* don't consider jetways farther than that */
static const float NEAR_SKIP = 2; /* don't consider jetways farther than that */

typedef struct _sam_jw  {

    /* local x,z computed from the xml's lat/lon*/
    double xml_x, xml_y, xml_z;
    unsigned int xml_ref_gen;   /* only valid if this matches the generation of the ref frame*/

    /* values from the actually drawn object */
    float x, y, z, psi;
    unsigned int obj_ref_gen;

    /* values fed to the datarefs */
    float rotate1, rotate2, rotate3, extent, wheels,
          wheelrotatec, wheelrotater, wheelrotatel;

    /* these are from sam.xml */
    char name[40];
    char sound[40];

    float latitude, longitude, heading, height, wheelPos, cabinPos, cabinLength,
          wheelDiameter, wheelDistance,
          minRot1, maxRot1, minRot2, maxRot2, minRot3, maxRot3,
          minExtent, maxExtent, minWheels, maxWheels,
          initialRot1, initialRot2, initialRot3, initialExtent;
    int door; /* 0 = LF1 or default, 1 = LF2 */

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for FAR_SKIP */
} sam_jw_t;

typedef struct _scenery {
    const char *name;
    sam_jw_t *sam_jws;
    int n_sam_jws;

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for FAR_SKIP */
} scenery_t;

extern scenery_t *sceneries;
extern int n_sceneries;

extern void log_msg(const char *fmt, ...);
extern int collect_sam_xml(const char *xp_dir);

extern XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, plane_y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

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

