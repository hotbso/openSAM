/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

*/

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>

#include "openSAM.h"
#include "samjw.h"
#include "jwctrl.h"

#include "os_dgs.h"
#include "plane.h"

static const float SAM_2_OBJ_MAX = 2.5;     // m, max delta between coords in sam.xml and object
static const float SAM_2_OBJ_HDG_MAX = 5;   // °, likewise for heading

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

//
// fill in values for a library jetway
//
void
SamJw::fill_library_values(int id)
{
    if (library_id)
        return;

    if (!BETWEEN(id, 1, MAX_SAM3_LIB_JW)) {
        log_msg("sanity check failed for jw: '%s', id: %d", name, id);
        return;
    }

    library_id = id;

    log_msg("filling in library data for '%s', id: %d", name, id);

    const SamJw *ljw = &sam3_lib_jw[id];

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
Stand*
SamJw::find_stand()
{
    float dist = 1.0E10;
    Stand *min_stand = nullptr;

    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (! sc->in_bbox(plane_lat, plane_lon))
            continue;

        for (auto s : sc->stands) {
            s->xform_to_ref_frame();

            float local_x, local_z;
            s->global_2_stand(x, z, local_x, local_z);
            if (local_x > 2.0f)     // on the right
                continue;

            float d = len2f(local_x, local_z);

            if (d < dist) {
                //log_msg("new min: %s, z: %2.1f, x: %2.1f",stand->id, local_z, local_x);
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
static SamJw*
configure_zc_jw(int id, float obj_x, float obj_z, float obj_y, float obj_psi)
{
    // library jetways may be in view from very far away when stand information is not
    // yet available. We won't see details anyway.
    if (len2f(obj_x - my_plane.x(), obj_z - my_plane.z()) > 0.5f * FAR_SKIP
        || fabsf(obj_y - my_plane.y()) > 1000.0f)
        return nullptr;

    SamJw *jw = new SamJw();
    jw->obj_ref_gen = ref_gen;
    jw->x = obj_x;
    jw->z = obj_z;
    jw->y = obj_y;
    jw->psi = obj_psi;
    jw->is_zc_jw = 1;
    strcpy(jw->name, "zc_");
    jw->fill_library_values(id);

    Stand *stand = jw->find_stand();
    if (stand) {
        // delta = cabin points perpendicular to stand
        float delta = RA((stand->hdgt + 90.0f) - jw->psi);
        // randomize
        float delta_r = (0.2f + 0.8f * (0.01f * (rand() % 100))) * delta;
        jw->initialRot2 = delta_r;
        log_msg("jw->psi: %0.1f, stand->hdgt: %0.1f, delta: %0.1f, initialRot2: %0.1f",
                jw->psi, stand->hdgt, delta, jw->initialRot2);
    } else
        jw->initialRot2 = 5.0f;

    jw->initialExtent = 0.3f;
    jw->initialRot3 = -3.0f * 0.01f * (rand() % 100);

    jw->rotate2 = jw->initialRot2;
    jw->rotate3 = jw->initialRot3;
    jw->extent = jw->initialExtent;
    jw->set_wheels();

    zc_jws.push_back(jw);

    log_msg("added to zc table stand: '%s', global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, initialRot2: %0.1f",
            stand ? stand->id : "<NULL>", jw->x, jw->z, jw->y, jw->psi, jw->initialRot2);
    return jw;
}

// check for shift of reference frame
void
check_ref_frame_shift()
{
    // check for shift of reference frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        log_msg("reference frame shift");
    }

    if (zc_ref_gen < ref_gen) {
        // from a different frame = stale data
        log_msg("zc_jws deleted");
        for (auto jw : zc_jws)
            delete(jw);

        zc_jws.resize(0);           // keep the allocation
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
static float
jw_anim_acc(void *ref)
{
    stat_acc_called++;

    float lat = my_plane.lat();
    float lon = my_plane.lon();

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_y = XPLMGetDataf(draw_object_y_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    check_ref_frame_shift();

    uint64_t ctx = (uint64_t)ref;
    dr_code_t drc = (dr_code_t)(ctx & 0xffffffff);
    int id = ctx >> 32;

    SamJw *jw = nullptr;

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (! sc->in_bbox(lat, lon)) {
            stat_sc_far_skip++;
            continue;
        }

        for (auto jw_ : sc->sam_jws) {
            // cheap check against bounding box
            if (lat < jw_->bb_lat_min || lat > jw_->bb_lat_max
                || RA(lon - jw_->bb_lon_min) < 0 || RA(lon - jw_->bb_lon_max) > 0) {
                stat_far_skip++;
                continue;
            }

            if (fabsf(RA(jw_->heading - obj_psi)) > SAM_2_OBJ_HDG_MAX)
                continue;

            if (jw_->xml_ref_gen < ref_gen) {
                // we must iterate to get the elevation of the jetway
                //
                // this stuff runs once when a jw in a scenery comes in sight
                // so it should not be too costly
                //
                double  x, y ,z;
                XPLMWorldToLocal(jw_->latitude, jw_->longitude, 0.0, &x, &y, &z);
                if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                    log_msg("terrain probe failed???");
                    return 0.0f;
                }

                // xform back to world to get an approximation for the elevation
                double lat, lon, elevation;
                XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ,
                                 &lat, &lon, &elevation);
                //log_msg("elevation: %0.2f", elevation);

                // and again to local with SAM's lat/lon and the approx elevation
                XPLMWorldToLocal(jw_->latitude, jw_->longitude, elevation, &x, &y, &z);
                if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
                    log_msg("terrain probe 2 failed???");
                    return 0.0f;
                }

                jw_->xml_x = probeinfo.locationX;
                jw_->xml_z = probeinfo.locationZ;
                jw_->xml_ref_gen = ref_gen;
            }

            if (fabs(obj_x - jw_->xml_x) <= SAM_2_OBJ_MAX && fabs(obj_z - jw_->xml_z) <= SAM_2_OBJ_MAX) {
                // have a match
                if (jw_->obj_ref_gen < ref_gen) {
                    // use higher precision values of the actually drawn object
                    jw_->obj_ref_gen = ref_gen;
                    jw_->x = obj_x;
                    jw_->z = obj_z;
                    jw_->y = obj_y;
                    jw_->psi = obj_psi;
                }

                stat_jw_match++;
                jw = jw_;
                goto out;   // of nested loops
            }

            stat_near_skip++;
        }
    }

    // no match of custom jw
    // check against the zero config table
    for (auto jw_ : zc_jws) {
        if (obj_x == jw_->x && obj_z == jw_->z && obj_y == jw_->y) {
            stat_jw_match++;
            jw = jw_;
            goto out;
        }

        stat_near_skip++;
    }

    if (nullptr == jw && BETWEEN(id, 1, MAX_SAM3_LIB_JW))   // unconfigured library jetway
        jw = configure_zc_jw(id, obj_x, obj_z, obj_y, obj_psi);

    if (nullptr == jw)    // still unconfigured -> bad luck
        return 0.0f;

   out:
    switch (drc) {
        case DR_ROTATE1:
            // a one shot event on first access
            if (id > 0) {
                jw->fill_library_values(id);
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
            log_msg("Accessor got invalid DR code: %d", drc);
            return 0.0f;
    }

    return 0.0f;
}

// static method, reset all jetways
void
SamJw::reset_all()
{
    for (auto sc : sceneries)
        for (auto jw : sc->sam_jws)
            jw->reset();

    for (auto jw : zc_jws)
        jw->reset();
}

void
jw_init()
{
    zc_jws.reserve(150);

    // create the jetway animation datarefs
    for (int drc = DR_ROTATE1; drc < N_JW_DR; drc++) {
        char name[100];
        name[99] = '\0';
        snprintf(name, sizeof(name) - 1, "sam/jetway/%s", dr_name_jw[drc]);
        XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL,
                                 NULL, jw_anim_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(uint64_t)drc, NULL);

        for (int i = 1; i <= MAX_SAM3_LIB_JW; i++) {
            snprintf(name, sizeof(name) - 1, "sam/jetway/%02d/%s", i, dr_name_jw[drc]);
            uint64_t ctx = (uint64_t)i << 32|(uint64_t)drc;
            XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL,
                                     NULL, jw_anim_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void *)ctx, NULL);
        }

    }


    SamJw::reset_all();
    srand(time(NULL));
}
