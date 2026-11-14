//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
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

#include "jetway.h"

// Geometry information of openSAM jetway model
struct OsJwModel {
    std::string model_id;
    std::string name;
    float height{}, wheel_pos{}, cabin_pos{}, cabin_length{}, wheel_diameter{}, wheel_distance{}, min_rot2{},
        max_rot2{}, min_rot3{}, max_rot3{}, min_extent{}, max_extent{}, stair_pos{}, stair_length{};
};

// an openSAM specific jetway implementation
class OsJetway : public Jetway {
   public:
    // max delta for heading between opensam.xml and object
    static constexpr float kSam2ObjHdgMax = 5;  // °, likewise for heading

    // library_id is configured when the jetway comes into view, so that may be delayed
    int library_id_{};        // id of the library jetway this one is configured from, 0 = none
    bool is_lib_jw_inst_{};   // is an instance of a library jetway
    bool is_zc_jw_{};         // is a zero config jw
    bool stand_retrieved_{};  // whether looking for a matching stand for this jw has been attempted
    bool is_deleted_{};       // whether this jetway instance was deleted by the editor

    std::string model_id_;  // geometry model id for this jetway
    std::string sound_;

    OsJetway() : Jetway(kOsJw) {};
    OsJetway(const OsJetway&) = delete;
    OsJetway& operator=(const OsJetway&) = delete;
    OsJetway(OsJetway&&) = delete;
    OsJetway& operator=(OsJetway&&) = delete;

    ~OsJetway() = default;

    // Fill values from a library jetway with numeric id, e.g. from a <library_jw> element in the scenery file
    void FillLibraryValues(unsigned int id);

    // Fill geometry values from a model jetway, e.g. from a <local_jw> element in the scenery file
    void FillModelValues(const OsJwModel& ljw);

    // configure a zc library jetway and add it to the quadtree and the global list of jetways.
    static OsJetway* AddZeroConfigJetway(int id, float obj_x, float obj_z, float obj_y, float obj_psi);

    // Animate the object based on its current state
    void Animate() override {}; // the DrefAccessor takes care

    bool deleted() const override { return is_deleted_; }

    // dataref accessor OsJetways
    static float DrefAccessor(void* ref);

    // Initializer
    static void Initialize(int max_sam_stands);
};

// library jetways information from all collected libraryjetways.xml files
// and local <models> <library_jw> elements in the scenery files
extern std::vector<OsJwModel*> os_jw_model_list;
