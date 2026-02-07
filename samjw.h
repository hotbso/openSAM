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

#include <string>
#include <vector>

static constexpr float kFarSkip = 5000;     // (m) don't consider jetways farther away

// Context of an instantiated jetway, either in sam.xml or zero config per WED within the scenery
struct SamJw {
    bool bad{};       // marked bad, e.g. terrain probe failed
    bool is_zc_jw{};  // is a zero config jw
    bool locked{};    // locked by a plane
    Stand* stand{};   // back pointer to stand for zc jetways

    // local x,z computed from the xml's lat/lon
    unsigned int xml_ref_gen{};  // only valid if this matches the generation of the ref frame
    float xml_x, xml_y, xml_z;

    // values from the actually drawn object
    unsigned int obj_ref_gen{};  // only valid if this matches the generation of the ref frame
    float x, y, z, psi;

    unsigned int library_id{};  // id of the library jetway this one is configured from, 0 = none

    // values fed to the datarefs
    float rotate1, rotate2, rotate3, extent, wheels, wheelrotatec{}, wheelrotater{}, wheelrotatel{}, warnlight;

    // geometry values from sam.xml or filled in from library jetway
    std::string name;
    std::string sound;

    float latitude{}, longitude{}, heading{}, height{}, wheelPos{}, cabinPos{}, cabinLength{}, wheelDiameter{},
        wheelDistance{}, minRot1{}, maxRot1{}, minRot2{}, maxRot2{}, minRot3{}, maxRot3{}, minExtent{}, maxExtent{},
        minWheels{}, maxWheels{}, initialRot1{}, initialRot2{}, initialRot3{}, initialExtent{};
    int door{};  // 0 = LF1 or default, 1 = LF2

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

    void FillLibraryValues(unsigned int id);
    Stand* FindStand();

    static void ResetAll(); // called from various places
};

// Geometry information of a library jetway
struct SamLibJw {
    std::string id;
    std::string name;
    float height{}, wheelPos{}, cabinPos{}, cabinLength{}, wheelDiameter{}, wheelDistance{}, minRot1{}, maxRot1{}, minRot2{}, maxRot2{},
        minRot3{}, maxRot3{}, minExtent{}, maxExtent{}, minWheels{}, maxWheels{};
};

// currently active zero config jetways
extern std::vector<SamJw*> zc_jws;

// library jetways information from all collected libraryjetways.xml files
extern std::vector<SamLibJw*> lib_jw;

extern void JwInit(void);
void CheckRefFrameShift();
#endif
