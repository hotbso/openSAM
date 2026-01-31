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

#include "openSAM.h"
#include "samjw.h"
#include "jwctrl.h"

#include "os_dgs.h"
#include "plane.h"

static constexpr float kSam2ObjMax = 2.5;     // m, max delta between coords in sam.xml and object
static constexpr float kSam2ObjHdgMax = 5;    // °, likewise for heading
static constexpr int kHashBits = 13;          // size of cache

// keep in sync with array below !
typedef enum dr_code_e {
    DR_ROTATE1, DR_ROTATE2, DR_ROTATE3, DR_EXTENT,
    DR_WHEELS, DR_WHEELROTATEC, DR_WHEELROTATER, DR_WHEELROTATEL,
    DR_WARNLIGHT,
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
    "warnlight"
};


// zero config jw structures
std::vector<SamJw *>zc_jws;
static unsigned int zc_ref_gen;  // change of ref_gen invalidates the whole list

static std::array<SamJw*, (1 << kHashBits)>jw_cache;

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
// find the stand the jetway belongs to
//
Stand* SamJw::FindStand() {
    float dist = 1.0E10;
    Stand* min_stand = nullptr;

    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (!sc->in_bbox(plane_lat, plane_lon))
            continue;

        for (auto s : sc->stands) {
            s->Xform2RefFrame();

            float local_x, local_z;
            s->Global2Stand(x, z, local_x, local_z);
            if (local_x > 2.0f)  // on the right
                continue;

            float d = len2f(local_x, local_z);

            if (d < dist) {
                // LogMsg("new min: %s, z: %2.1f, x: %2.1f",stand->id, local_z, local_x);
                dist = d;
                min_stand = s;
            }
        }
    }

    stand = min_stand;
    return stand;
}

//
// configure a zc library jetway
//
static SamJw* ConfigureZcJw(int id, float obj_x, float obj_z, float obj_y, float obj_psi) {
    // library jetways may be in view from very far away when stand information is not
    // yet available. We won't see details anyway.
    if (len2f(obj_x - my_plane.x(), obj_z - my_plane.z()) > 0.5f * kFarSkip || fabsf(obj_y - my_plane.y()) > 1000.0f)
        return nullptr;

    SamJw* jw = new SamJw();
    jw->obj_ref_gen = ref_gen;
    jw->x = obj_x;
    jw->z = obj_z;
    jw->y = obj_y;
    jw->psi = obj_psi;
    jw->is_zc_jw = true;
    jw->name = "zc_";
    jw->FillLibraryValues(id);

    Stand* stand = jw->FindStand();
    if (stand) {
        // delta = cabin points perpendicular to stand
        float delta = RA((stand->hdgt + 90.0f) - jw->psi);
        // randomize
        float delta_r = (0.2f + 0.8f * (0.01f * (rand() % 100))) * delta;
        jw->initialRot2 = delta_r;
        LogMsg("jw->psi: %0.1f, stand->hdgt: %0.1f, delta: %0.1f, initialRot2: %0.1f", jw->psi, stand->hdgt, delta,
               jw->initialRot2);
    } else
        jw->initialRot2 = 5.0f;

    jw->initialExtent = 0.3f;
    jw->initialRot3 = -3.0f * 0.01f * (rand() % 100);

    jw->rotate2 = jw->initialRot2;
    jw->rotate3 = jw->initialRot3;
    jw->extent = jw->initialExtent;
    jw->SetWheels();

    zc_jws.push_back(jw);

    LogMsg("added to zc table stand: '%s', global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, initialRot2: %0.1f",
           stand ? stand->id.c_str() : "<NULL>", jw->x, jw->z, jw->y, jw->psi, jw->initialRot2);
    return jw;
}

// check for shift of reference frame
void CheckRefFrameShift() {
    // check for shift of reference frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        jw_cache = {};
        LogMsg("reference frame shift");
    }

    if (zc_ref_gen < ref_gen) {
        // from a different frame = stale data
        LogMsg("zc_jws deleted");
        for (auto jw : zc_jws)
            delete (jw);

        zc_jws.resize(0);  // keep the allocation
        zc_ref_gen = ref_gen;
    }
}

//
// Accessor for the "sam/jetway/..." datarefs
//
// This function is called from draw loops, efficient coding required.
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

    stat_acc_called++;

    CheckRefFrameShift();

    uint64_t ctx = (uint64_t)ref;
    dr_code_t drc = (dr_code_t)(ctx & 0xffffffff);
    unsigned int id = ctx >> 32;

    SamJw* jw = nullptr;

    // We cache jetway pointers in a "1-way associative cache" 8-) .
    // The tag is jw->(x, y, z).
    // For the mapping we use the x coordinate in 0.5 m resolution as base.
    // Unless the airport is extremely large the high bits are mostly the the same
    // hence we merge in the z coordinate as high bit.
    // Results in a hit rate of ~99% for SFD KLAX.
    unsigned ci_lo = (int)(obj_x * 2.0f) & ((1 << (kHashBits - 1)) - 1);
    unsigned ci_hi = (int)(obj_z) << (kHashBits - 1);
    unsigned cache_idx = (ci_hi | ci_lo) & ((1 << kHashBits) - 1);
    SamJw* cjw = jw_cache[cache_idx];
    if (cjw && cjw->x == obj_x && cjw->y == obj_y && cjw->z == obj_z) {
        stat_jw_cache_hit++;
        jw = cjw;
        goto have_jw;
    }

    {
        float lat = my_plane.lat();
        float lon = my_plane.lon();
        float obj_psi = XPLMGetDataf(draw_object_psi_dr);

        for (auto sc : sceneries) {
            // cheap check against bounding box
            if (!sc->in_bbox(lat, lon)) {
                stat_sc_far_skip++;
                continue;
            }

            for (auto tjw : sc->sam_jws) {
                if (tjw->bad)
                    continue;

                if (tjw->xml_ref_gen < ref_gen) {
                    // we must iterate to get the elevation of the jetway
                    //
                    // this stuff runs once when a jw in a scenery comes in sight
                    // so it should not be too costly
                    //
                    double x, y, z;
                    XPLMWorldToLocal(tjw->latitude, tjw->longitude, 0.0, &x, &y, &z);
                    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                        LogMsg("terrain probe 1 failed, jw: '%s', lat,lon: %0.6f, %0.6f, x,y,z: %0.5f, %0.5f, %0.5f",
                               tjw->name.c_str(), tjw->latitude, tjw->longitude, x, y, z);
                        LogMsg("jw: '%s' marked BAD", tjw->name.c_str());
                        tjw->bad = true;
                        return 0.0f;
                    }

                    // xform back to world to get an approximation for the elevation
                    double lat, lon, elevation;
                    XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &lat, &lon,
                                     &elevation);
                    // LogMsg("elevation: %0.2f", elevation);

                    // and again to local with SAM's lat/lon and the approx elevation
                    XPLMWorldToLocal(tjw->latitude, tjw->longitude, elevation, &x, &y, &z);
                    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                        LogMsg("terrain probe 2 failed???");
                        return 0.0f;
                    }

                    tjw->xml_x = probeinfo.locationX;
                    tjw->xml_z = probeinfo.locationZ;
                    tjw->xml_ref_gen = ref_gen;
                }

                if (fabsf(obj_x - tjw->xml_x) <= kSam2ObjMax && fabsf(obj_z - tjw->xml_z) <= kSam2ObjMax) {
                    // Heading is likely to match.
                    // We check position first as it's more likely to rule out other jetways
                    // letting us perform less checks to find the right one.
                    if (fabsf(RA(tjw->heading - obj_psi)) > kSam2ObjHdgMax)
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

                    stat_jw_match++;
                    jw_cache[cache_idx] = jw = tjw;
                    goto have_jw;  // of nested loops
                }

                stat_near_skip++;
            }
        }

        // no match of custom jw
        // check against the zero config table
        for (auto tjw : zc_jws) {
            if (obj_x == tjw->x && obj_z == tjw->z && obj_y == tjw->y) {
                stat_jw_match++;
                jw_cache[cache_idx] = jw = tjw;
                goto have_jw;
            }

            stat_near_skip++;
        }

        if (nullptr == jw && 0 < id && id < lib_jw.size())  // unconfigured library jetway
            jw = ConfigureZcJw(id, obj_x, obj_z, obj_y, obj_psi);

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
        default:
            LogMsg("Accessor got invalid DR code: %d", drc);
            return 0.0f;
    }

    return 0.0f;
}

// static method, reset all jetways
void SamJw::ResetAll() {
    for (auto sc : sceneries)
        for (auto jw : sc->sam_jws)
            jw->Reset();

    for (auto jw : zc_jws)
        jw->Reset();
}

void JwInit() {
    zc_jws.reserve(150);

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
