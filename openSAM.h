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
typedef struct _sam_jw  {

    /* local x,y,z */
    double x, y ,z, psi;
    unsigned int ref_gen;   /* only valid if this matches the generation of the ref frame*/

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
} sam_jw_t;

extern sam_jw_t *sam_jws;
extern int n_sam_jws, max_sam_jws;

extern void log_msg(const char *fmt, ...);
extern int collect_sam_xml(const char *xp_dir);

