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
#include <array>

#include "XPLMGraphics.h"

#include "opensam.h"
#include "samjw.h"
#include "opensam_airport.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "log_msg.h"

#include "flat_earth_math.h"
namespace fem = flat_earth_math;

// keep in sync with array below !
enum DrCode {
    kRotate1, kRotate2, kRotate3, kExtent,
    kWheels, kWheelRotateC, kWheelRotateR, kWheelRotateL,
    kWarnLight,
    kCanopy,
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
    "canopy"
};

// The global cache for jetways keyed by (x,z)
static std::unordered_map<PositionCacheKey, SamJw*, PositionCacheKeyHasher> jw_cache;
static unsigned int jwc_ref_gen = 0;  // generation # of the reference frame for which the cache is valid

quadtree::LLQuadTree<double, SamJw, kMaxJwPerNode> jw_quadtree;
std::vector<SamJw*> sam_jw_list;
std::vector<SamLibJw*> lib_jw;

//
// fill in values for a library jetway
//
void SamJw::FillLibraryValues(unsigned int id) {
    if (library_id)
        return;

    if (id == 0 || id >= lib_jw.size()) {
        LogMsg("sanity check failed for jw: '%s', id: %d", name.c_str(), id);
        return;
    }

    library_id = id;
    const SamLibJw* ljw = lib_jw[id];
    if (ljw == nullptr) {
        LogMsg("Unconfigured library jw for '%s', id: %d", name.c_str(), id);
        return;
    }

    LogMsg("filling in library data for '%s', id: %d", name.c_str(), id);
    height = ljw->height;
    wheelPos = ljw->wheelPos;
    cabinPos = ljw->cabinPos;
    cabinLength = ljw->cabinLength;

    wheelDiameter = ljw->wheelDiameter;
    wheelDistance = ljw->wheelDistance;

    minRot1 = ljw->minRot1;
    maxRot1 = ljw->maxRot1;

    minRot2 = ljw->minRot2;
    maxRot2 = ljw->maxRot2;

    minRot3 = ljw->minRot3;
    maxRot3 = ljw->maxRot3;

    minExtent = ljw->minExtent;
    maxExtent = ljw->maxExtent;

    minWheels = ljw->minWheels;
    maxWheels = ljw->maxWheels;
}

//
// configure a zc library jetway and add it to the scenery
// As all jetways zc jetways should live forever as there can be complex interactions between
// frame shifts and multiplayer planes with active jetways.
//
static SamJw* AddZeroConfigJetway(int id, float obj_x, float obj_z, float obj_y, float obj_psi) {
    SamJw* jw = new SamJw();
    jw->obj_ref_gen = ref_gen;
    jw->x = obj_x;
    jw->z = obj_z;
    jw->y = obj_y;
    jw->psi = obj_psi;
    jw->is_zc_jw = true;

    // fill the 'sam.xml' related position values
    XPLMLocalToWorld(obj_x, obj_y, obj_z, &jw->latitude,  &jw->longitude, &jw->altitude);
    jw->heading = obj_psi;

    jw->ComputeBbox();

    // try to update stand related parameters or delay that until os_arpt is available
    const OsStand* stand = nullptr;
    if (os_arpt) {
        jw->zc_stand_done = true;   // one shot only
        stand = os_arpt->FindStandForJw(jw->x, jw->z);
    }

    if (stand) {
        jw->base_name = stand->name();
        // delta = cabin points perpendicular to stand
        float delta = fem::RA((stand->hdgt() + 90.0f) - jw->psi);
        // randomize
        float delta_r = (0.2f + 0.8f * (0.01f * (rand() % 100))) * delta;
        jw->initialRot2 = delta_r;
        LogMsg("jw->psi: %0.1f, stand->hdgt: %0.1f, delta: %0.1f, initialRot2: %0.1f", jw->psi, stand->hdgt(), delta,
               jw->initialRot2);
    } else {
        jw->base_name = "zc_";
        jw->initialRot2 = 5.0f;
    }

    jw->initialExtent = 0.3f;
    jw->initialRot3 = -3.0f * 0.01f * (rand() % 100);

    jw->rotate2 = jw->initialRot2;
    jw->rotate3 = jw->initialRot3;
    jw->extent = jw->initialExtent;
    jw->FillLibraryValues(id);
    jw->SetWheels();

    // add to the global stores
    sam_jw_list.push_back(jw);
    jw_quadtree.Insert(jw);

    LogMsg("added zc jetway, stand: '%s', global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, initialRot2: %0.1f",
           jw->base_name.c_str(), jw->x, jw->z, jw->y, jw->psi, jw->initialRot2);

    return jw;
}

//
// Accessor for the "sam/jetway/..." datarefs
//
// This function is called from draw loops, efficient coding required.
// It's called with a frequency of at least fps * <# of visible jetways> * 9 .
//
// ref is uint64_t and has the library id in the high long and the dataref id in low long.
// e.g.
// sam/jetways/rotate1     -> ( 0, kRotate1)
// sam/jetways/15/rotate2  -> (15, kRotate2)
//
static float JwAnimAcc(void* ref) {
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

    SamJw* jw = nullptr;

    // We cache jetway pointers for quick lookup by position in the local coordinate system.
    if (jwc_ref_gen != ref_gen) [[unlikely]] {
        jwc_ref_gen = ref_gen;
        LogMsg("jw cache invalidated by reference frame shift, load factor: %0.2f", jw_cache.load_factor());
        jw_cache.clear();
    }

    PositionCacheKey key{obj_x, obj_z};
    auto it = jw_cache.find(key);
    if (it != jw_cache.end()) [[likely]] {
        stat_jw_cache_hit++;
        jw = it->second;
        if (jw == nullptr)
            return 0.0f;  // negative cache entry, object at this position is not a jetway
    } else {
        const float obj_psi = XPLMGetDataf(draw_object_psi_dr);

        double obj_lat, obj_lon, obj_alt;
        XPLMLocalToWorld(obj_x, obj_y, obj_z, &obj_lat, &obj_lon, &obj_alt);

        std::array<SamJw*, kMaxJwPerNode> candidates;
        int n_candidates = jw_quadtree.Find(obj_lon, obj_lat, candidates);
        LogMsg("quadtree lookup for obj at x: %5.3f, z: %5.3f, lat: %0.6f, lon: %0.6f, found %d candidates", obj_x,
               obj_z, obj_lat, obj_lon, n_candidates);

        if (n_candidates == 1) [[likely]] {
            jw = candidates[0];
            LogMsg("quadtree candidate: '%s', lat: %0.6f, lon: %0.6f", jw->name.c_str(), jw->latitude, jw->longitude);
            if (std::abs(fem::RA(jw->heading - obj_psi)) > SamJw::kSam2ObjHdgMax) {
                LogMsg("candidate '%s' rejected by heading, candidate heading: %0.1f, obj_psi: %0.1f", jw->name.c_str(),
                       jw->heading, obj_psi);
                jw_cache[key] = nullptr;  // negative cache entry
                return 0.0f;
            }

            jw->obj_ref_gen = ref_gen;
            jw->x = obj_x;
            jw->z = obj_z;
            jw->y = obj_y;
            jw->psi = obj_psi;

            jw_cache[key] = jw;
        } else if (n_candidates > 1) [[unlikely]] {
            for (int i = 0; i < n_candidates; i++) {
                SamJw* candidate = candidates[i];
                LogMsg("candidate %d: '%s', lat: %0.6f, lon: %0.6f", i, candidate->name.c_str(), candidate->latitude,
                       candidate->longitude);
            }
            // TODO: find nearest candidate by distance, but for now just reject all if there are multiple candidates to
            // avoid wrong matches
            LogMsg("multiple candidates found for obj at x: %5.3f, z: %5.3f, lat: %0.6f, lon: %0.6f, rejecting all",
                   obj_x, obj_z, obj_lat, obj_lon);
            jw_cache[key] = nullptr;  // negative cache entry
            return 0.0f;              // multiple candidates, can't decide which one is correct, reject all
        } else if (n_candidates == 0 && id == 0) [[unlikely]] {
            // TODO: try FindAround and search nearest
            LogMsg("quadtree lookup found no candidates for obj at x: %5.3f, z: %5.3f, lat: %0.6f, lon: %0.6f", obj_x,
                   obj_z, obj_lat, obj_lon);
            jw_cache[key] = nullptr;  // negative cache entry
            return 0.0f;
        }

        // obj was not found in the scenery's configured jetways, but maybe it's a zero config library jetway
        if (nullptr == jw && 0 < id && id < lib_jw.size()) {  // unconfigured library jetway
            jw = AddZeroConfigJetway(id, obj_x, obj_z, obj_y, obj_psi);
            jw_cache[key] = jw;
        }

        if (nullptr == jw) {          // still unconfigured -> bad luck
            jw_cache[key] = nullptr;  // negative cache entry
            return 0.0f;
        }
    }  // no cache hit

    switch (drc) {
        case kRotate1:
            // a one shot event on first access
            if (id > 0) {
                jw->FillLibraryValues(id);
            }
            return jw->rotate1;
            break;
        case kRotate2:
            return jw->rotate2;
            break;
        case kRotate3:
            return jw->rotate3;
            break;
        case kExtent:
            return jw->extent;
            break;
        case kWheels:
            return jw->wheels;
            break;
        case kWheelRotateC:
            return jw->wheelrotatec;
            break;
        case kWheelRotateR:
            return jw->wheelrotater;
            break;
        case kWheelRotateL:
            return jw->wheelrotatel;
            break;
        case kWarnLight:
            return jw->warnlight;
            break;
        case kCanopy:
            return jw->canopy;
            break;
        default:
            LogMsg("Accessor got invalid DR code: %d", drc);
            return 0.0f;
    }

    return 0.0f;
}

// static method, reset all jetways
void SamJw::ResetAll() {
    for (auto jw : sam_jw_list)
        jw->Reset();
}

void JwInit(int max_sam_stands) {
    jw_cache.reserve(max_sam_stands);   // usually there are much more stands than jetways

    // create the jetway animation datarefs
    for (int drc = kRotate1; drc < kNumDrCodes; drc++) {
        char name[100];
        name[99] = '\0';
        snprintf(name, sizeof(name) - 1, "sam/jetway/%s", dr_name_jw[drc]);
        XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL, NULL, JwAnimAcc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void*)(uint64_t)drc, NULL);

        for (unsigned int i = 1; i < lib_jw.size(); i++) {
            snprintf(name, sizeof(name) - 1, "sam/jetway/%s/%s", lib_jw[i]->id.c_str(), dr_name_jw[drc]);
            uint64_t ctx = (uint64_t)i << 32 | (uint64_t)drc;
            XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL, NULL, JwAnimAcc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void*)ctx, NULL);
        }
    }

    SamJw::ResetAll();
    srand(time(NULL));  // for random initial values for zero config jetways
}
