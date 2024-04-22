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

static const float D2R = M_PI/180.0;
static const float F2M = 0.3048;	/* 1 ft [m] */
static const float LON_D2M = 111120;    /* 1° lon in m */
static const float FAR_SKIP = 4000; /* don't consider jetways farther than that */
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
          initialRot1, initialRot2, initialRot3, initialExtent,
          door; /* 0 = LF1 or default, 1 = LF2 */
} sam_jw_t;

typedef struct _scenery {
    sam_jw_t *sam_jws;
    int n_sam_jws;
    float NE_lat, NE_lon, SW_lat, SW_lon;   /* bounding box + FAR_SKIP */
} scenery_t;

extern scenery_t *sceneries;
extern int n_sceneries;

extern void log_msg(const char *fmt, ...);
extern int collect_sam_xml(const char *xp_dir);

