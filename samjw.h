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

#include "XPLMSound.h"

#include "quadtree.h"

struct Sound {
    void *data;
    int size;
    int num_channels;
    int sample_rate;
};

// Context of an instantiated jetway, either in sam.xml or zero config per WED within the scenery
class SamJw {
   private:
    FMOD_CHANNEL* alert_chn_ = nullptr;
    static Sound alert_;
    static void AlertComplete(void* ref, FMOD_RESULT status);

    int locked_{};      // locked by a plane
    int lock_pid_{-1};  // id of the plane that has locked this jetway, for logging purposes

   public:
    static constexpr float kD2R = std::numbers::pi / 180.0;
    static constexpr float kSam2ObjMax = 2.5;   // m, max delta between coords in sam.xml and object
    static constexpr float kSam2ObjHdgMax = 5;  // °, likewise for heading

    // values fed to the datarefs in the dataref accessor, computed by a JwCtrl
    float rotate1_, rotate2_, rotate3_, extent_, wheels_, wheelrotatec_{}, wheelrotater_{}, wheelrotatel_{}, warnlight_, canopy_;

    // These describe placement and use of the jw instance.
    // They are from opensam.xml or generated for zero config library jetways from scenery objects.
    double latitude_{}, longitude_{};
    float heading_{}, min_rot1_{}, max_rot1_{}, initial_rot1_{}, initial_rot2_{}, initial_rot3_{}, initial_extent_{};
    int door_{};  // 0 = LF1 or default, 1 = LF2

    std::string model_id_;   // geometry model id for this jetway
    std::string base_name_;  // from sam.xml, e.g. "jetway1", "jetway2" or "jetway3"
    std::string name_;       // == base_name_ for sam.xml jetways or fabricated for zero config jetways
    std::string sound_;

    // These geometry values are filled in from the jetways model for the instance
    float height_{}, wheel_pos_{}, cabin_pos_{}, cabin_length_{}, wheel_diameter_{}, wheel_distance_{},
        min_rot2_{}, max_rot2_{}, min_rot3_{}, max_rot3_{}, min_extent_{}, max_extent_{};
    double altitude_{};  // altitude is determined by terrain probe or XPLMLocalToWorld for zero config jetways

    // local coordinate values of the actually drawn object
    unsigned int obj_ref_gen_{};  // only valid if this matches the generation # of the ref frame
    float x_, y_, z_, psi_;

    bool bad_{};             // marked bad, e.g. terrain probe failed
    bool is_lib_jw_inst_{};  // is an instance of a library jetway
    // library_id is configured when the jetway comes into view, so that may be delayed
    int library_id_{};  // id of the library jetway this one is configured from, 0 = none

    bool is_zc_jw_{};         // is a zero config jw
    bool stand_retrieved_{};  // whether looking for a matching stand for this jw has been attempted
    bool is_undefined_{};     // whether this jetway is present in the scenery but undefined in opensam.xml
    bool is_deleted_{};       // whether this jetway instance was deleted by the editor

    // bounding box around the anchor point for quick lookup in quadtree, computed from lat/lon and kSam2ObjMax
    quadtree::Box<double> bbox_;

    bool is_locked() const noexcept { return locked_ > 0; }
    bool Lock(int pid) noexcept;  // -> whether lock could be aquired
    void Unlock() noexcept;

    // set wheels height
    void SetWheels() { wheels_ = std::tan(rotate3_ * kD2R) * (wheel_pos_ + extent_); }

    void Reset() {
        AlertOff();
        locked_ = 0;
        lock_pid_ = -1;
        rotate1_ = initial_rot1_;
        rotate2_ = initial_rot2_;
        rotate3_ = initial_rot3_;
        extent_ = initial_extent_;
        SetWheels();
        warnlight_ = 0;
        canopy_ = 0;
    }

    void FillLibraryValues(unsigned int id);

    // for the quadtree...
    void ComputeBbox() {
        static constexpr float kLat2m = 111120;  // 1° lat in m
        double dlat = SamJw::kSam2ObjMax / kLat2m;
        double dlon = SamJw::kSam2ObjMax / (kLat2m * std::cos(latitude_ * kD2R));
        bbox_ = {longitude_ - dlon, latitude_ - dlat, longitude_ + dlon, latitude_ + dlat};
    }

    double lon() const { return longitude_; }
    double lat() const { return latitude_; }
    quadtree::Box<double> bounds() const { return bbox_; }
    std::string repr() const { return name_; }
    bool hidden() const {return is_undefined_; }
    bool deleted() const { return is_deleted_; }  // never shows up

    // sound stuff
    void AlertOn();
    void AlertOff();
    void AlertSetpos();

    // Initializers and Finaliser
    static void SoundInit();  // inits device and loads wav
    static void Init(int max_sam_stands);
    static void Finalize();
};

// Geometry information of jetway model
struct SamJwModel {
    std::string model_id;
    std::string name;
    float height{}, wheel_pos{}, cabin_pos{}, cabin_length{}, wheel_diameter{}, wheel_distance{}, min_rot2{}, max_rot2{},
        min_rot3{}, max_rot3{}, min_extent{}, max_extent{};
};

// the global quadtree for all sam jetways collected from sam.xml files and zero config jetways
static constexpr int kMaxJwPerNode = 10;
extern quadtree::LLQuadTree<double, SamJw, kMaxJwPerNode> jw_quadtree;  // for fast lookup by position
extern std::vector<SamJw*> sam_jw_list;  // for iterating over all jetways, e.g. for resetting them

// library jetways information from all collected libraryjetways.xml files
// and local <models> <library_jw> elements in the scenery files
extern std::vector<SamJwModel*> lib_jw;

// from ReadWav.cpp
extern void ReadWav(const std::string& fname, Sound& sound);
