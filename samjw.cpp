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

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <cassert>

#include "opensam.h"
#include "samjw.h"
#include "jwctrl.h"

#include "my_plane.h"
#include "opensam_airport.h"

static constexpr float kSam2ObjMax = 2.5;     // m, max delta between coords in sam.xml and object
static constexpr float kSam2ObjHdgMax = 5;    // °, likewise for heading

// keep in sync with array below !
typedef enum dr_code_e {
    DR_ROTATE1, DR_ROTATE2, DR_ROTATE3, DR_EXTENT,
    DR_WHEELS, DR_WHEELROTATEC, DR_WHEELROTATER, DR_WHEELROTATEL,
    DR_WARNLIGHT,
    DR_CANOPY,
    N_JW_DR
} dr_code_t;

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

static std::unordered_map<PositionCacheKey, SamJw*, PositionCacheKeyHasher> jw_cache;
static unsigned int jwc_ref_gen = 0;  // generation # of the reference frame for which the cache is valid

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
// As all jetways zc jetways ahould live forever as there can be complex interactions between
// frame shifts and multiplayer planes with active jetways
//
SamJw* Scenery::AddZeroConfigJetway(int id, float obj_x, float obj_z, float obj_y, float obj_psi) {
    SamJw* jw = new SamJw();
    jw->obj_ref_gen = ref_gen;
    jw->x = obj_x;
    jw->z = obj_z;
    jw->y = obj_y;
    jw->psi = obj_psi;
    jw->is_zc_jw = true;

    XPLMLocalToWorld(obj_x, obj_y, obj_z, &jw->latitude,  &jw->longitude, &jw->altitude);
    jw->xml_ref_gen = ref_gen;
    jw->xml_x = obj_x;
    jw->xml_y = obj_y;
    jw->xml_z = obj_z;
    jw->heading = obj_psi;

    const OsStand* stand = os_arpt->FindStandForJw(jw->x, jw->z);

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

    if (sam_jws_.size() == 0)
        sam_jws_.reserve(os_arpt->n_stands());  // should be a good intial estimate

    sam_jws_.push_back(jw);

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
// sam/jetways/rotate1     -> ( 0, DR_ROTATE1)
// sam/jetways/15/rotate2  -> (15, DR_ROTATE2)
//
static float JwAnimAcc(void* ref) {
    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_y = XPLMGetDataf(draw_object_y_dr);

    if (obj_x == 0.0f && obj_y == 0.0f && obj_z == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    stat_jw_acc_called++;

    CheckRefFrameShift();

    uint64_t ctx = (uint64_t)ref;
    dr_code_t drc = (dr_code_t)(ctx & 0xffffffff);
    unsigned int id = ctx >> 32;

    SamJw* jw = nullptr;

    // We cache jetway pointers for quick lookup by position in the local coordinate system.
    if (jwc_ref_gen != ref_gen) {
        jwc_ref_gen = ref_gen;
        LogMsg("jw cache invalidated by reference frame shift, load factor: %0.2f", jw_cache.load_factor());
        jw_cache.clear();
    }

    PositionCacheKey key{obj_x, obj_z};
    auto it = jw_cache.find(key);
    if (it != jw_cache.end()) {
        stat_jw_cache_hit++;
        jw = it->second;
        goto have_jw;
    }

    {
        float lat = my_plane->lat();
        float lon = my_plane->lon();
        float obj_psi = XPLMGetDataf(draw_object_psi_dr);

        for (auto sc : sceneries) {
            // cheap check against bounding box
            if (!sc->InBbox(lat, lon)) {
                stat_sc_far_skip++;
                continue;
            }

            for (auto tjw : sc->sam_jws_) {
                if (tjw->bad)
                    continue;

                if (tjw->xml_ref_gen < ref_gen) {
                    // we must iterate to get the elevation of the jetway
                    //
                    // this stuff runs once when a jw in a scenery comes in sight
                    // so it should not be too costly
                    //
                    double x, y, z;
                    XPLMWorldToLocal(tjw->latitude, tjw->longitude, tjw->altitude, &x, &y, &z);
                    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                        LogMsg("terrain probe 1 failed, jw: '%s', lat,lon: %0.6f, %0.6f, x,y,z: %0.5f, %0.5f, %0.5f",
                               tjw->name.c_str(), tjw->latitude, tjw->longitude, x, y, z);
                        LogMsg("jw: '%s' marked BAD", tjw->name.c_str());
                        tjw->bad = true;
                        return 0.0f;
                    }

                    // xform back to world to get an approximation for the elevation
                    double lat, lon;
                    XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &lat, &lon,
                                     &tjw->altitude);
                    // LogMsg("elevation: %0.2f", tjw->altitude);

                    // and again to local with SAM's lat/lon and the approx elevation
                    XPLMWorldToLocal(tjw->latitude, tjw->longitude, tjw->altitude, &x, &y, &z);
                    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                        LogMsg("terrain probe 2 failed???");
                        return 0.0f;
                    }

                    tjw->xml_x = probeinfo.locationX;
                    tjw->xml_z = probeinfo.locationZ;
                    tjw->xml_ref_gen = ref_gen;
                }

                if (std::abs(obj_x - tjw->xml_x) <= kSam2ObjMax && std::abs(obj_z - tjw->xml_z) <= kSam2ObjMax) {
                    // Heading is likely to match.
                    // We check position first as it's more likely to rule out other jetways
                    // letting us perform less checks to find the right one.
                    if (std::abs(fem::RA(tjw->heading - obj_psi)) > kSam2ObjHdgMax)
                        continue;

                    // have a match
                    if (tjw->obj_ref_gen < ref_gen) {
                        // use higher precision values of the actually drawn object
                        tjw->obj_ref_gen = ref_gen;
                        tjw->x = obj_x;
                        tjw->z = obj_z;
                        tjw->y = obj_y;
                        tjw->psi = obj_psi;
                    }

                    jw_cache[key] = jw = tjw;
                    goto have_jw;  // out of nested loops
                }

                stat_near_skip++;
            }

            // obj was not found in the scenery's configured jetways, but maybe it's a zero config library jetway
            if (nullptr == jw && 0 < id && id < lib_jw.size()) {  // unconfigured library jetway
                if (os_arpt == nullptr)
                    return 0.0f;  // airport not loaded yet, can't do anything

                jw = sc->AddZeroConfigJetway(id, obj_x, obj_z, obj_y, obj_psi);
                goto have_jw;
            }

            break;  // only check the first matching scenery, we should not have multiple sceneries with jetways in
                    // sight at the same time
        }

        if (nullptr == jw)  // still unconfigured -> bad luck
            return 0.0f;
    }

have_jw:
    switch (drc) {
        case DR_ROTATE1:
            // a one shot event on first access
            if (id > 0) {
                jw->FillLibraryValues(id);
            }
            return jw->rotate1;
            break;
        case DR_ROTATE2:
            return jw->rotate2;
            break;
        case DR_ROTATE3:
            return jw->rotate3;
            break;
        case DR_EXTENT:
            return jw->extent;
            break;
        case DR_WHEELS:
            return jw->wheels;
            break;
        case DR_WHEELROTATEC:
            return jw->wheelrotatec;
            break;
        case DR_WHEELROTATER:
            return jw->wheelrotater;
            break;
        case DR_WHEELROTATEL:
            return jw->wheelrotatel;
            break;
        case DR_WARNLIGHT:
            return jw->warnlight;
            break;
        case DR_CANOPY:
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
    for (auto sc : sceneries)
        for (auto jw : sc->sam_jws_)
            jw->Reset();
}

void JwInit(int max_sam_stands) {
    jw_cache.reserve(max_sam_stands);   // usually there are much more stands than jetways

    // create the jetway animation datarefs
    for (int drc = DR_ROTATE1; drc < N_JW_DR; drc++) {
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
    srand(time(NULL));
}
