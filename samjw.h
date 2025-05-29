//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
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

#ifndef _SAMJW_H_
#define _SAMJW_H_

static constexpr float kFarSkip = 5000;     // (m) don't consider jetways farther away

struct SamJw  {
  public:
    bool bad;       // marked bad, e.g. terrain probe failed
    int is_zc_jw;   // is a zero config jw
    Stand* stand;   // back pointer to stand for zc jetways
    bool locked;	// locked by a plane

    // local x,z computed from the xml's lat/lon
    float xml_x, xml_y, xml_z;
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

    // set wheels height
    void SetWheels() {
        wheels = tanf(rotate3 * kD2R) * (wheelPos + extent);
    }

    void Reset() {
        locked = false;
        rotate1 = initialRot1;
        rotate2 = initialRot2;
        rotate3 = initialRot3;
        extent = initialExtent;
        SetWheels();
        warnlight = 0;
    }

    void FillLibraryValues(int id);
    Stand* FindStand();

    static void ResetAll();
};

extern std::vector<SamJw *>zc_jws;

// fortunately SAM3 is abandoned so this will never change 8-)
#define MAX_SAM3_LIB_JW 27  // index is 0..27
extern SamJw sam3_lib_jw[];

extern void JwInit(void);
void CheckRefFrameShift();
#endif
