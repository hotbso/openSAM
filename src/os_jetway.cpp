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

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <string>
#include <array>
#include <format>

#include "XPLMGraphics.h"

#include "opensam.h"
#include "os_jetway.h"
#include "os_airport.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "position_cache_key.h"
#include "log_msg.h"

#include "flat_earth_math.h"
namespace fem = flat_earth_math;

// keep in sync with array below !
enum DrCode {
    kRotate1, kRotate2, kRotate3, kExtent,
    kWheels, kWheelRotateC, kWheelRotateR, kWheelRotateL,
    kWarnLight,
    kCanopy,
    kStairRotate,
    kNumDrCodes
};

static const char *dr_name_jw[] = {
    "rotate1",
    "rotate2",
    "rotate3",
    "extent",
    "wheels",
    "wheelrotatec",
    "wheelrotater",
    "wheelrotatel",
    "warnlight",
    "canopy",
    "stairrotate"
};

// The global cache for os jetways keyed by (x,z)
static std::unordered_map<PositionCacheKey, OsJetway*, PositionCacheKeyHasher> jw_cache;
static unsigned int jwc_ref_gen = 0;  // generation # of the reference frame for which the cache is valid

std::vector<OsJwModel*> os_jw_model_list;

//
// fill in values for a library jetway
//
void OsJetway::FillLibraryValues(unsigned int id) {
    if (library_id_)
        return;

    if (id == 0 || id >= os_jw_model_list.size()) {
        LogMsg("sanity check failed for jw: '{}', id: {}", name_, id);
        return;
    }

    library_id_ = id;
    const OsJwModel* ljw = os_jw_model_list[id];
    if (ljw == nullptr) {
        LogMsg("Unconfigured library jw for '{}', id: {}", name_, id);
        return;
    }

    LogMsg("filling in library data for '{}', id: {}", name_, id);
    FillModelValues(*ljw);
    // That's for a repair of missing data from a previous version of this plugin
    if (min_rot1_ == 0.0f && max_rot1_ == 0.0f) {
        min_rot1_ = -90.0f;
        max_rot1_ = 90.0f;
    }
}

void OsJetway::FillModelValues(const OsJwModel& jwm) {
    height_ = jwm.height;
    wheel_pos_ = jwm.wheel_pos;
    cabin_pos_ = jwm.cabin_pos;
    cabin_length_ = jwm.cabin_length;
    wheel_diameter_ = jwm.wheel_diameter;
    wheel_distance_ = jwm.wheel_distance;
    min_rot2_ = jwm.min_rot2;
    max_rot2_ = jwm.max_rot2;
    min_rot3_ = jwm.min_rot3;
    max_rot3_ = jwm.max_rot3;
    min_extent_ = jwm.min_extent;
    max_extent_ = jwm.max_extent;
    stair_pos_ = jwm.stair_pos;
    stair_length_ = jwm.stair_length;
}

//
// configure a zc library jetway and add it to the quadtree and the global list of jetways.
//
OsJetway* OsJetway::AddZeroConfigJetway(int id, float obj_x, float obj_z, float obj_y, float obj_psi) {
    OsJetway* jw = new OsJetway();
    jw->obj_ref_gen_ = ref_gen;
    jw->x_ = obj_x;
    jw->z_ = obj_z;
    jw->y_ = obj_y;
    jw->psi_ = obj_psi;
    jw->is_zc_jw_ = true;
    jw->is_lib_jw_inst_ = true;  // zc jetways are always library instances
    jw->min_rot1_ = -90.0f;
    jw->max_rot1_ = 90.0f;

    // fill the 'opensam.xml' related position values
    XPLMLocalToWorld(obj_x, obj_y, obj_z, &jw->latitude_,  &jw->longitude_, &jw->altitude_);
    jw->heading_ = obj_psi;

    jw->ComputeBbox();

    jw->base_name_ = "zcjw";    // hopefully will be replaced by stand name in DrefAccessor
    jw->initial_extent_ = 0.3f;
    jw->initial_rot1_ = 0.0f;
    jw->initial_rot2_ = -5.0f;  // will be refined later in DrefAccessor
    jw->initial_rot3_ = -3.0f * 0.01f * (rand() % 100);

    jw->rotate1_ = jw->initial_rot1_;
    jw->rotate2_ = jw->initial_rot2_;
    jw->rotate3_ = jw->initial_rot3_;
    jw->extent_ = jw->initial_extent_;
    jw->FillLibraryValues(id);

    // add to the global stores
    jw_list.push_back(jw);
    jw_quadtree.Insert(jw);

    LogMsg("added zc jetway, stand: '{}', global: x: {:5.3f}, z: {:5.3f}, y: {:5.3f}, psi: {:4.1f}, initial_rot2: {:0.1f}",
           jw->base_name_, jw->x_, jw->z_, jw->y_, jw->psi_, jw->initial_rot2_);

    return jw;
}

//
// Accessor for the "sam/jetway/..." datarefs
//
// This function is called from draw loops, efficient coding required.
// It's called with a frequency of at least fps * <# of visible jetways> * 9 .
//
// ref is uint64_t and has the library id in the high 32bit and the dataref id in low 32bit.
// e.g.
// sam/jetways/rotate1     -> ( 0, kRotate1)
// sam/jetways/15/rotate2  -> (15, kRotate2)
//
// static
float OsJetway::DrefAccessor(void* ref) {
    // hopefully one day we get these as double
    const float obj_x = XPLMGetDataf(draw_object_x_dr);
    const float obj_z = XPLMGetDataf(draw_object_z_dr);
    const float obj_y = XPLMGetDataf(draw_object_y_dr);

    if (obj_x == 0.0f && obj_y == 0.0f && obj_z == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    stat_jw_acc_called++;

    CheckRefFrameShift();

    uint64_t ctx = reinterpret_cast<uint64_t>(ref);
    DrCode drc = static_cast<DrCode>(ctx & 0xffffffff);
    unsigned int id = ctx >> 32;

    OsJetway* jw = nullptr;

    // We cache jetway pointers for quick lookup by position in the local coordinate system.
    if (jwc_ref_gen != ref_gen) [[unlikely]] {
        jwc_ref_gen = ref_gen;
        LogMsg("jw cache invalidated by reference frame shift, load factor: {:.2}", jw_cache.load_factor());
        jw_cache.clear();
    }

    PositionCacheKey key{obj_x, obj_z};
    auto it = jw_cache.find(key);
    if (it != jw_cache.end()) [[likely]] {
        stat_jw_cache_hit++;
        jw = it->second;
        assert(jw);
        if (jw->is_undefined_ || jw->is_deleted_)
            return 0.0f;  // negative cache entry, object at this position is not a recognized jetway
    } else {
        const float obj_psi = XPLMGetDataf(draw_object_psi_dr);

        double obj_lat, obj_lon, obj_alt;
        XPLMLocalToWorld(obj_x, obj_y, obj_z, &obj_lat, &obj_lon, &obj_alt);

        // Add a jetway that came just into view into the cache
        auto CacheJw = [&](OsJetway* jw) {
            if (id > 0)
                jw->FillLibraryValues(id);
            jw->obj_ref_gen_ = ref_gen;
            jw->x_ = obj_x;
            jw->y_ = obj_y;
            jw->z_ = obj_z;
            jw->psi_ = obj_psi;
            jw_cache[key] = jw;
        };

        // create an undefined jw at the current object position
        auto AddUndefinedJw = [&]() -> OsJetway* {
            LogMsg("creating undefined jetway for position: ll: ({:.6}, {:.6}), x: {:5.3f}, z: {:5.3f}", obj_lat, obj_lon, key.x, key.z);
            OsJetway* jw = new OsJetway();
            jw->is_undefined_ = true;
            jw->latitude_ = obj_lat;
            jw->longitude_ = obj_lon;
            jw->altitude_ = obj_alt;
            jw->heading_ = obj_psi;
            jw->ComputeBbox();
            jw_list.push_back(jw);
            jw_quadtree.Insert(jw);
            return jw;
        };

        // filter out undefined jetways and ensure the class code matches for SAM jetways
        auto FilterNotUndefined = [](const Jetway* jw) { return jw->class_code() == Jetway::kOsJw && !jw->is_undefined_; };

        std::array<Jetway*, kMaxJwPerNode> candidates;
        int n_candidates = jw_quadtree.Find(obj_lon, obj_lat, candidates, FilterNotUndefined);

        if (n_candidates == 1) [[likely]] {
            jw = static_cast<OsJetway*>(candidates[0]);
            LogMsg("quadtree candidate: '{}', ll: ({}, {})", jw->name_, jw->latitude_, jw->longitude_);

            if (std::abs(fem::RA(jw->heading_ - obj_psi)) > kSam2ObjHdgMax) {
                LogMsg("candidate '{}' rejected by heading, candidate heading: {:0.1f}, obj_psi: {:0.1f}", jw->name_,
                       jw->heading_, obj_psi);
                LogMsg("negative cached: obj: ll({}, {}), candidate: ll({}, {})", obj_lat, obj_lon, jw->latitude_, jw->longitude_);
                // Create a new undefined jetway
                // -> negative caching for this object position and heading
                jw = AddUndefinedJw();
                CacheJw(jw);
                return 0.0f;
            }

            // Now with position and terrain available adjust wheels + stairs
            jw->SetWheelsStairs(/* high_precision = */ true);
            CacheJw(jw);
        } else if (n_candidates > 1) [[unlikely]] {
            float min_dist = 100000.0f;
            Jetway* nearest = nullptr;
            fem::LLPos obj_pos(obj_lat, obj_lon);

            for (int i = 0; i < n_candidates; i++) {
                Jetway* candidate = candidates[i];
                LogMsg("candidate {}: '{}', ll: ({}, {})", i, candidate->name_, candidate->latitude_,
                       candidate->longitude_);
                if (std::abs(fem::RA(candidate->heading_ - obj_psi)) > kSam2ObjHdgMax) {
                    LogMsg("candidate '{}' rejected by heading, candidate heading: {:0.1f}, obj_psi: {:0.1f}", candidate->name_,
                        candidate->heading_, obj_psi);
                    continue;
                }

                float dist = fem::len(fem::LLPos(candidate->latitude_, candidate->longitude_) - obj_pos);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = candidate;
                }
            }

            if (nearest == nullptr) {
                LogMsg("all candidates rejected by heading");
                OsJetway* jw = AddUndefinedJw();
                CacheJw(jw);
                return 0.0f;
            }

            LogMsg("nearest candidate: '{}', ll: ({}, {}), dist: {:.2}m", nearest->name_,
                   nearest->latitude_, nearest->longitude_, min_dist);
            jw = static_cast<OsJetway*>(nearest);
            jw->SetWheelsStairs(/* high_precision = */ true);
            CacheJw(jw);
        } else if (n_candidates == 0 && id == 0) [[unlikely]] {
            LogMsg("quadtree lookup found no candidates for obj at x: {:5.3f}, z: {:5.3f}, ll: ({}, {})", obj_x,
                   obj_z, obj_lat, obj_lon);

            // some support for sloppy configured scenery, e.g. sam.xml values quite off
            quadtree::Box<double> search_box(obj_lon, obj_lat, 2 * Jetway::kJw2ObjMax);  // 2 * max delta between sam.xml and object coords
            std::unordered_map<Jetway*, bool>  around = jw_quadtree.FindInBox(search_box, FilterNotUndefined);
            if (around.empty()) {
                LogMsg("FindInBox found no candidates either");
                OsJetway* jw = AddUndefinedJw();   // negative caching for this object position and heading
                CacheJw(jw);
                return 0.0f;
            }

            LogMsg("FindInBox found {} candidates:", around.size());
            float min_dist = 100000.0f;
            Jetway* nearest = nullptr;
            fem::LLPos obj_pos(obj_lat, obj_lon);

            for (auto& [ajw, _] : around) {
                LogMsg("candidate: '{}', ll: ({}, {})", ajw->name_, ajw->latitude_, ajw->longitude_);
                if (std::abs(fem::RA(ajw->heading_ - obj_psi)) > 2.0f * kSam2ObjHdgMax) { // be generous as well
                    LogMsg("candidate '{}' rejected by heading, candidate heading: {:0.1f}, obj_psi: {:0.1f}", ajw->name_,
                        ajw->heading_, obj_psi);
                    continue;
                }

                float dist = fem::len(fem::LLPos(ajw->latitude_, ajw->longitude_) - obj_pos);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = ajw;
                }
            }

            if (nearest == nullptr) {
                LogMsg("FindInBox found no nearest candidate");
                OsJetway* jw = AddUndefinedJw();   // negative caching for this object position and heading
                CacheJw(jw);
                return 0.0f;
            }

            LogMsg("nearest candidate: '{}', ll: ({}, {}), dist: {:.2}m", nearest->name_,
                   nearest->latitude_, nearest->longitude_, min_dist);
            jw = static_cast<OsJetway*>(nearest);
            jw->SetWheelsStairs(/* high_precision = */ true);
            CacheJw(jw);
        }

        // obj was not found in the scenery's configured jetways, but maybe it's a zero config library jetway
        if (nullptr == jw && 0 < id && id < os_jw_model_list.size()) {  // unconfigured library jetway
            jw = AddZeroConfigJetway(id, obj_x, obj_z, obj_y, obj_psi);
            jw->SetWheelsStairs(/* high_precision = */ true);
            jw_cache[key] = jw;
        }

        if (nullptr == jw) {          // still unconfigured -> bad luck
            LogMsg("could not configure jw");
            OsJetway* jw = AddUndefinedJw();   // negative caching for this object position and heading
            CacheJw(jw);
            return 0.0f;
        }
    }  // no cache hit

    if (jw->is_undefined_ || jw->is_deleted_)       // we have hit an undefined or deleted jetway
        return 0.0f;

    switch (drc) {
        case kRotate1:
            return jw->rotate1_;
            break;
        case kRotate2:
            // one shot attempt to update stand related parameters for zc jetways once os_arpt is available
            if (jw->is_zc_jw_ && !jw->stand_retrieved_ && os_arpt) [[unlikely]] {
                jw->stand_retrieved_ = true;  // one shot only
                const OsStand* stand = os_arpt->FindStandForJw(jw->x_, jw->z_);
                if (stand) {
                    jw->base_name_ = stand->name();
                    // delta = cabin points perpendicular to stand
                    float delta = fem::RA((stand->hdgt() + 90.0f) - jw->psi_);
                    float delta_r = delta + 0.1f * ((std::rand() % 100) - 50);  // +- 5° randomization
                    jw->initial_rot2_ = delta_r;
                    jw->rotate2_ = jw->initial_rot2_;
                    LogMsg("zc {}, jw->psi: {:0.1f}, stand->hdgt: {:0.1f}, delta: {:0.1f}, initial_rot2: {:0.1f}", jw->base_name_, jw->psi_,
                           stand->hdgt(), delta, jw->initial_rot2_);
                }
            }
            return jw->rotate2_;
            break;
        case kRotate3:
            return jw->rotate3_;
            break;
        case kExtent:
            return jw->extent_;
            break;
        case kWheels:
            return jw->wheels_;
            break;
        case kWheelRotateC:
            return jw->wheelrotatec_;
            break;
        case kWheelRotateR:
            return jw->wheelrotater_;
            break;
        case kWheelRotateL:
            return jw->wheelrotatel_;
            break;
        case kWarnLight:
            return jw->warnlight_;
            break;
        case kCanopy:
            return jw->canopy_;
            break;
        case kStairRotate:
            return jw->stairrotate_;
            break;
        default:
            LogMsg("Accessor got invalid DR code: {}", (int)drc);
            return 0.0f;
    }

    return 0.0f;
}

// static
void OsJetway::Initialize(int max_sam_stands) {
    jw_cache.reserve(max_sam_stands);  // usually there are much more stands than jetways

    // create the jetway animation datarefs
    for (int drc = kRotate1; drc < kNumDrCodes; drc++) {
        std::string name = std::format("sam/jetway/{}", dr_name_jw[drc]);
        XPLMRegisterDataAccessor(name.c_str(), xplmType_Float, 0, NULL, NULL, DrefAccessor, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void*)(uint64_t)drc, NULL);

        for (unsigned int i = 1; i < os_jw_model_list.size(); i++) {
            std::string name = std::format("sam/jetway/{}/{}", os_jw_model_list[i]->model_id, dr_name_jw[drc]);
            uint64_t ctx = (uint64_t)i << 32 | (uint64_t)drc;
            XPLMRegisterDataAccessor(name.c_str(), xplmType_Float, 0, NULL, NULL, DrefAccessor, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void*)ctx, NULL);
        }
    }

    for (auto jw : jw_list)
        jw->Reset();

    srand(time(NULL));  // for random initial values for zero config jetways
}
