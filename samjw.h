//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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

#pragma once

#include <numbers>
#include <string>
#include <vector>
#include <cmath>

#include "quadtree.h"

// Context of an instantiated jetway, either in sam.xml or zero config per WED within the scenery
struct SamJw {
   private:
    int locked{};          // locked by a plane
    int lock_pid{-1};        // id of the plane that has locked this jetway, for logging purposes

   public:
    static constexpr float kD2R = std::numbers::pi/180.0;
    static constexpr float kSam2ObjMax = 2.5;     // m, max delta between coords in sam.xml and object
    static constexpr float kSam2ObjHdgMax = 5;    // °, likewise for heading

    bool bad{};            // marked bad, e.g. terrain probe failed
    bool is_zc_jw{};       // is a zero config jw
    bool zc_stand_done{};  // for zero config jetways, whether looking for a stand has been attempted
    // values from the actually drawn object
    unsigned int obj_ref_gen{};  // only valid if this matches the generation of the ref frame
    float x, y, z, psi;

    int library_id{};  // id of the library jetway this one is configured from, 0 = none

    // values fed to the datarefs
    float rotate1, rotate2, rotate3, extent, wheels, wheelrotatec{}, wheelrotater{}, wheelrotatel{},
        warnlight, canopy;

    // geometry values from sam.xml or filled in from library jetway
    std::string base_name;  // from sam.xml, e.g. "jetway1", "jetway2" or "jetway3"
    std::string name;       // == base_name for sam.xml jetways or fabricated for zero config jetways
    std::string sound;

    double latitude{}, longitude{}, altitude{}; // altitude is determined by terrain probe or XPLMLocalToWorld for zero config jetways
    float heading{}, height{}, wheelPos{}, cabinPos{}, cabinLength{}, wheelDiameter{},
        wheelDistance{}, minRot1{}, maxRot1{}, minRot2{}, maxRot2{}, minRot3{}, maxRot3{}, minExtent{}, maxExtent{},
        minWheels{}, maxWheels{}, initialRot1{}, initialRot2{}, initialRot3{}, initialExtent{};
    int door{};  // 0 = LF1 or default, 1 = LF2

    // bounding box around the anchor point for quick lookup in quadtree, computed from lat/lon and kSam2ObjMax
    quadtree::Box<double> bbox;

    bool is_locked() const { return locked > 0; }
    bool Lock(int pid); // -> whether lock could be aquired
    void Unlock();

    // set wheels height
    void SetWheels() {
        wheels = std::tan(rotate3 * kD2R) * (wheelPos + extent);
    }

    void Reset() {
        locked = 0;
        lock_pid = -1;
        rotate1 = initialRot1;
        rotate2 = initialRot2;
        rotate3 = initialRot3;
        extent = initialExtent;
        SetWheels();
        warnlight = 0;
        canopy = 0;
    }

    void FillLibraryValues(unsigned int id);

    // for the quadtree...

    void ComputeBbox() {
        static constexpr float kLat2m = 111120;  // 1° lat in m
        double dlat = SamJw::kSam2ObjMax / kLat2m;
        double dlon = SamJw::kSam2ObjMax / (kLat2m * std::cos(latitude * kD2R));
        bbox = {longitude - dlon, latitude - dlat, longitude + dlon, latitude + dlat};
    }

    double lon() const { return longitude; }
    double lat() const { return latitude; }
    quadtree::Box<double> bounds() const { return bbox; }
    std::string repr() const { return name; }

    // Initializer and Finaliser
    static void Init(int max_sam_stands);
    static void Finalize();
};

// Geometry information of a library jetway
struct SamLibJw {
    std::string id;
    std::string name;
    float height{}, wheelPos{}, cabinPos{}, cabinLength{}, wheelDiameter{}, wheelDistance{}, minRot1{}, maxRot1{}, minRot2{}, maxRot2{},
        minRot3{}, maxRot3{}, minExtent{}, maxExtent{}, minWheels{}, maxWheels{};
};

// the global quadtree for all sam jetways collected from sam.xml files and zero config jetways
static constexpr int kMaxJwPerNode = 10;
extern quadtree::LLQuadTree<double, SamJw, kMaxJwPerNode> jw_quadtree;  // for fast lookup by position
extern std::vector<SamJw*> sam_jw_list;  // for iterating over all jetways, e.g. for resetting them

// library jetways information from all collected libraryjetways.xml files
extern std::vector<SamLibJw*> lib_jw;
