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

// A jetway living somewhere in the X-Plane scenery.
// These are collected as OsJetway from opensam.xml or configured on the fly from a zero config jetway placed with WED somewhere in
// the scenery or as XP12Jetway from apt.dat files.
// .. and once configured jetways never die so we can use raw pointers.
class Jetway {
   public:
    // everything beyond a simple ClassCode pattern seems to be overengineered here
    enum ClassCode { kOsJw, kXP12Jw };

   private:
    const ClassCode class_code_;
    FMOD_CHANNEL* alert_chn_ = nullptr;
    static Sound alert_;
    static void AlertComplete(void* ref, FMOD_RESULT status);

    int locked_{};      // locked by a plane
    int lock_pid_{-1};  // id of the plane that has locked this jetway, for logging purposes

   protected:
    float wheels_adjust_{};      // due to terrain slope

    explicit Jetway(ClassCode class_code) : class_code_(class_code) {}

   public:
    ClassCode class_code() const { return class_code_; }

    virtual ~Jetway() = default;

    static constexpr float kD2R = std::numbers::pi / 180.0;
    static constexpr float kJw2ObjMax = 2.5;   // m, max delta between jw coordinates and actual object

    // values fed to the datarefs in the dataref accessor, computed by a JwCtrl
    float rotate1_, rotate2_, rotate3_, extent_, wheels_, wheelrotatec_{}, wheelrotater_{}, wheelrotatel_{}, warnlight_, canopy_, stairrotate_{};

    // Position and initial placement of the jetway in the scenery
    // They are from opensam.xml or aptr.dat or generated for zero config library jetways from scenery objects
    double latitude_{}, longitude_{};
    double altitude_{};  // altitude is determined by terrain probe or XPLMLocalToWorld

    float heading_{}, min_rot1_{}, max_rot1_{}, initial_rot1_{}, initial_rot2_{}, initial_rot3_{}, initial_extent_{};
    int door_{};  // 0 = LF1 or default, 1 = LF2

    std::string base_name_;  // from sam.xml, e.g. "jetway1", "jetway2" or "jetway3"
    std::string name_;       // == base_name_ for sam.xml jetways or fabricated for zero config jetways

    // These geometry values are filled in from the jetways model for the instance
    float height_{}, wheel_pos_{}, cabin_pos_{}, cabin_length_{}, wheel_diameter_{}, wheel_distance_{},
        min_rot2_{}, max_rot2_{}, min_rot3_{}, max_rot3_{}, min_extent_{}, max_extent_{}, stair_pos_{}, stair_length_{};

    // local coordinate values of the actually drawn object
    unsigned int obj_ref_gen_{};  // only valid if this matches the generation # of the ref frame
    double x_, y_, z_;            // Position in meters in local coords
    float psi_;

    bool is_undefined_{};     // present in the scenery but undefined geometry

    // bounding box around the anchor point for quick lookup in quadtree, computed from lat/lon and kSam2ObjMax
    quadtree::Box<double> bbox_;

    bool is_locked() const noexcept { return locked_ > 0; }
    bool Lock(int pid) noexcept;  // -> whether lock could be aquired
    void Unlock() noexcept;

    // set wheels height + stairs angles
    void SetWheelsStairs(bool force_high_precision = false);

    // Animate the object based on its current state
    virtual void Animate() = 0;

    // reset to initial position, alert off, ...
    void Reset();

    // sound stuff
    void AlertOn();
    void AlertOff();
    void AlertSetpos();

    // for the quadtree...
    void ComputeBbox() {
        static constexpr float kLat2m = 111120;  // 1° lat in m
        double dlat = kJw2ObjMax / kLat2m;
        double dlon = kJw2ObjMax / (kLat2m * std::cos(latitude_ * kD2R));
        bbox_ = {longitude_ - dlon, latitude_ - dlat, longitude_ + dlon, latitude_ + dlat};
    }

    double lon() const { return longitude_; }
    double lat() const { return latitude_; }
    quadtree::Box<double> bounds() const { return bbox_; }
    virtual std::string repr() const { return name_; }
    virtual bool deleted() const { return false; }  // never shows up

    // Initializers and Finaliser
    static void SoundInit();  // inits device and loads wav
    static void Initialize();
    static void Finalize();
};

// the global quadtree for all jetways in the system
static constexpr int kMaxJwPerNode = 10;
extern quadtree::LLQuadTree<double, Jetway, kMaxJwPerNode> jw_quadtree;  // for fast lookup by position
extern std::vector<Jetway*> jw_list;  // for iterating over all jetways, e.g. for resetting them
