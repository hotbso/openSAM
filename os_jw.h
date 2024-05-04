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

// fortunately SAM3 is abandoned so this will never change 8-)
#define MAX_SAM3_LIB_JW 27  // index is 0..27
extern sam_jw_t sam3_lib_jw[];

extern int jw_init(void);
extern float jw_state_machine();
