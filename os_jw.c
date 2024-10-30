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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>

#include "openSAM.h"
#include "os_jw.h"
#include "os_jw_impl.h"

#include "os_dgs.h"

static const float SAM_2_OBJ_MAX = 2.5;     // m, max delta between coords in sam.xml and object
static const float SAM_2_OBJ_HDG_MAX = 5;   // °, likewise for heading

static const float JW_DRIVE_SPEED = 1.0;    // m/s
static const float JW_TURN_SPEED = 10.0;    // °/s
static const float JW_HEIGHT_SPEED = 0.1;   // m/s
static const float JW_ANIM_INTERVAL = -1;   // s for debugging or -1 for frame loop
static const float JW_ANIM_TIMEOUT  = 50;   // s
static const float JW_ALIGN_DIST = 1.0;     // m abeam door

typedef enum
{
    DISABLED=0, IDLE, PARKED, SELECT_JWS, CAN_DOCK,
    DOCKING, DOCKED, UNDOCKING, CANT_DOCK
} state_t;

static const char * const state_str[] = {
    "DISABLED", "IDLE", "PARKED", "SELECT_JWS", "CAN_DOCK",
    "DOCKING", "DOCKED", "UNDOCKING", "CANT_DOCK" };

static state_t state = IDLE;
static state_t prev_state = DISABLED;

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

int n_active_jw;
jw_ctx_t active_jw[MAX_DOOR];

jw_ctx_t nearest_jw[MAX_NEAREST];
int n_nearest;

// zero config jw structures
static sam_jw_t zc_jws[150];    // static for now
static int zc_max_jws = 150;
static int zc_n_jws;
static unsigned int zc_ref_gen;  // change of ref_gen invalidates the whole list

static int dock_requested, undock_requested, toggle_requested;
static float plane_x, plane_y, plane_z, plane_psi, sin_psi, cos_psi;

XPLMCommandRef dock_cmdr, undock_cmdr, toggle_cmdr, toggle_ui_cmdr;


// set wheels height
static inline void
jw_set_wheels(sam_jw_t *jw)
{
    jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);
}

//
// fill in values for a library jetway
//
static void
fill_library_values(sam_jw_t *jw)
{
    int id = jw->library_id;
    if (!BETWEEN(id, 1, MAX_SAM3_LIB_JW)) {
        log_msg("sanity check failed for jw: '%s', id: %d", jw->name, id);
        return;
    }

    log_msg("filling in library data for '%s', id: %d", jw->name, id);

    sam_jw_t *ljw = &sam3_lib_jw[id];

    jw->height = ljw->height;
    jw->wheelPos = ljw->wheelPos;
    jw->cabinPos = ljw->cabinPos;
    jw->cabinLength = ljw->cabinLength;

    jw->wheelDiameter = ljw->wheelDiameter;
    jw->wheelDistance = ljw->wheelDistance;

    jw->minRot1 = ljw->minRot1;
    jw->maxRot1 = ljw->maxRot1;

    jw->minRot2 = ljw->minRot2;
    jw->maxRot2 = ljw->maxRot2;

    jw->minRot3 = ljw->minRot3;
    jw->maxRot3 = ljw->maxRot3;

    jw->minExtent = ljw->minExtent;
    jw->maxExtent = ljw->maxExtent;

    jw->minWheels = ljw->minWheels;
    jw->maxWheels = ljw->maxWheels;
}

//
// find the stand the jetway belongs to
//
static stand_t *
find_stand_for_jw(sam_jw_t *jw)
{
    float dist = 1.0E10;
    stand_t *min_stand = NULL;

    float plane_lat = XPLMGetDataf(plane_lat_dr);
    float plane_lon = XPLMGetDataf(plane_lon_dr);

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        // cheap check against bounding box
        if (plane_lat < sc->bb_lat_min || plane_lat > sc->bb_lat_max
            || RA(plane_lon - sc->bb_lon_min) < 0 || RA(plane_lon - sc->bb_lon_max) > 0) {
            continue;
        }

        for (stand_t *stand = sc->stands; stand < sc->stands + sc->n_stands; stand++) {
            xform_to_ref_frame(stand);

            float local_x, local_z;
            global_2_stand(stand, jw->x, jw->z, &local_x, &local_z);
            if (local_x > 2.0f)     // on the right
                continue;

            float d = len2f(local_x, local_z);

            if (d < dist) {
                //log_msg("new min: %s, z: %2.1f, x: %2.1f",stand->id, local_z, local_x);
                dist = d;
                min_stand = stand;
            }
        }
    }

    return min_stand;
}

//
// configure a zc library jetway
//
static sam_jw_t *
configure_zc_jw(int id, float obj_x, float obj_z, float obj_y, float obj_psi)
{
    if (zc_n_jws == zc_max_jws)
        return NULL;

    // library jetways may be in view from very far away when stand information is not
    // yet available. We won't see details anyway.
    if (len2f(obj_x - XPLMGetDataf(plane_x_dr), obj_z - XPLMGetDataf(plane_z_dr)) > 0.5f * FAR_SKIP
        || fabsf(obj_y - XPLMGetDataf(plane_y_dr)) > 1000.0f)
        return NULL;

    sam_jw_t *jw = &zc_jws[zc_n_jws++];
    *jw = (sam_jw_t){0};
    jw->obj_ref_gen = ref_gen;
    jw->x = obj_x;
    jw->z = obj_z;
    jw->y = obj_y;
    jw->psi = obj_psi;
    jw->is_zc_jw = 1;
    strcpy(jw->name, "zc_");
    jw->library_id = id;
    fill_library_values(jw);

    stand_t *stand = jw->stand = find_stand_for_jw(jw);
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
    jw_set_wheels(jw);
    log_msg("added to zc table stand: '%s', global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, initialRot2: %0.1f",
            stand ? stand->id : "<NULL>", jw->x, jw->z, jw->y, jw->psi, jw->initialRot2);
    return jw;
}

// check for shift of reference frame
static inline void
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
        zc_n_jws = 0;
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

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_y = XPLMGetDataf(draw_object_y_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    check_ref_frame_shift();

    uint64_t ctx = (uint64_t)ref;
    dr_code_t drc = ctx & 0xffffffff;
    int id = ctx >> 32;

    sam_jw_t *jw = NULL;

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        // cheap check against bounding box
        if (lat < sc->bb_lat_min || lat > sc->bb_lat_max
            || RA(lon - sc->bb_lon_min) < 0 || RA(lon - sc->bb_lon_max) > 0) {
            stat_sc_far_skip++;
            continue;
        }

        for (sam_jw_t *jw_ = sc->sam_jws; jw_ < sc->sam_jws + sc->n_sam_jws; jw_++) {
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
    for (sam_jw_t *jw_ = zc_jws; jw_ < zc_jws + zc_n_jws; jw_++) {
        if (obj_x == jw_->x && obj_z == jw_->z && obj_y == jw_->y) {
            stat_jw_match++;
            jw = jw_;
            goto out;
        }

        stat_near_skip++;
    }

    if (NULL == jw && BETWEEN(id, 1, MAX_SAM3_LIB_JW))   // unconfigured library jetway
        jw = configure_zc_jw(id, obj_x, obj_z, obj_y, obj_psi);

    if (NULL == jw)    // still unconfigured -> bad luck
        return 0.0f;

   out:
    switch (drc) {
        case DR_ROTATE1:
            // a one shot event on first access
            if (id > 0 && 0 == jw->library_id) {
                jw->library_id = id;
                fill_library_values(jw);
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

// opensam/jetway/status dataref
//  0 = no jetway
//  1 = can dock
//  2 = docked
// -1 = can't dock or in transit
static int
jw_status_acc(void *ref)
{
    // opensam/jetway/number
    if (ref == &n_active_jw)
        return n_active_jw;

    // opensam/jetway/status
    if (0 == n_active_jw)
        return 0;

    if (CAN_DOCK == state)
        return 1;

    if (DOCKED == state)
        return 2;

    return -1;
}

// opensam/jetway/door/status array by door
//  0 = not docked
//  1 = docked
//
static int
jw_door_status_acc(XPLMDataRef ref, int *values, int ofs, int n)
{
    UNUSED(ref);

    if (values == NULL)
        return MAX_DOOR;

    if (n <= 0 || ofs < 0 || ofs >= MAX_DOOR)
        return 0;

    n = MIN(n, MAX_DOOR - ofs);

    for (int i = 0; i < n; i++) {
        jw_ctx_t *ajw = &active_jw[ofs + i];
        values[i] = (ajw->jw && ajw->state == AJW_DOCKED) ? 1 : 0;
    }

    return n;
}

static void
reset_jetways()
{
    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            jw->rotate1 = jw->initialRot1;
            jw->rotate2 = jw->initialRot2;
            jw->rotate3 = jw->initialRot3;
            jw->extent = jw->initialExtent;
            jw_set_wheels(jw);
            jw->warnlight = 0;
       }

    for (sam_jw_t *jw = zc_jws; jw < zc_jws + zc_n_jws; jw++) {
        jw->rotate1 = jw->initialRot1;
        jw->rotate2 = jw->initialRot2;
        jw->rotate3 = jw->initialRot3;
        jw->extent = jw->initialExtent;
        jw_set_wheels(jw);
        jw->warnlight = 0;
    }

    for (int i = 0; i < n_door; i++) {
        jw_ctx_t *ajw = &active_jw[i];
        if (ajw->jw)
            alert_off(ajw);
    }

    state = IDLE;
}

// hook for the ui
void
jw_auto_mode_change()
{
    if (state == SELECT_JWS)
        state = IDLE;
    else
        reset_jetways();    // an animation might be ongoing
}

// convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 can be NULL
static inline void
jw_xy_to_sam_dr(const jw_ctx_t *ajw, float cabin_x, float cabin_z,
                float *rot1, float *extent, float *rot2, float *rot3)
{
    const sam_jw_t *jw = ajw->jw;

    float dist = len2f(cabin_x - ajw->x, cabin_z - ajw->z);

    float rot1_d = atan2(cabin_z - ajw->z, cabin_x - ajw->x) / D2R;   // door frame
    *rot1 =  RA(rot1_d + 90.0f - ajw->psi);
    *extent = dist - jw->cabinPos;

    // angle 0° door frame  -> hdgt -> jw frame -> diff to rot1
    float r2 = RA(0.0f + 90.0f - ajw->psi - *rot1);
    if (rot2)
        *rot2 = r2;

    if (rot3) {
        float net_length = dist + jw->cabinLength * cosf(r2 * D2R);
        *rot3 = -atan2f(ajw->y, net_length) / D2R;
    }
}

//
// fill in geometry data related to specific door
//
static void
jw_ctx_for_door(jw_ctx_t *ajw, const door_info_t *door_info)
{
    sam_jw_t *jw = ajw->jw;

    // rotate into plane local frame
    float dx = jw->x - plane_x;
    float dz = jw->z - plane_z;
    ajw->x =  cos_psi * dx + sin_psi * dz;
    ajw->z = -sin_psi * dx + cos_psi * dz;
    ajw->psi = RA(jw->psi - plane_psi);

    // xlate into door local frame
    ajw->x -= door_info->x;
    ajw->z -= door_info->z;

    float rot1_d = RA((jw->initialRot1 + ajw->psi) - 90.0f);    // door frame
    ajw->cabin_x = ajw->x + (jw->extent + jw->cabinPos) * cosf(rot1_d * D2R);
    ajw->cabin_z = ajw->z + (jw->extent + jw->cabinPos) * sinf(rot1_d * D2R);

    ajw->tgt_x = -jw->cabinLength;
    // tgt z = 0.0
    ajw->y = (jw->y + jw->height) - (plane_y + door_info->y);

    jw_xy_to_sam_dr(ajw, ajw->tgt_x, 0.0f, &ajw->tgt_rot1, &ajw->tgt_extent, &ajw->tgt_rot2, &ajw->tgt_rot3);

    float r = jw->initialExtent + jw->cabinPos;
    ajw->parked_x = ajw->x + r * cosf(rot1_d * D2R);
    ajw->parked_z = ajw->z + r * sinf(rot1_d * D2R);

    ajw->ap_x = ajw->tgt_x - JW_ALIGN_DIST;

    jw_set_wheels(jw);
}

// a fuzzy comparator for jetway by door number
static int
njw_compar(const void *a_, const void *b_)
{
    const jw_ctx_t *a = a_;
    const jw_ctx_t *b = b_;

    // height goes first
    if (a->jw->height < b->jw->height - 1.0f)
        return -1;

    if (a->jw->height > b->jw->height + 1.0f)
        return 1;

    // then z
    if (a->z < b->z - 0.5f)
        return -1;

    if (a->z > b->z + 0.5f)
        return 1;

    // then x, further left (= towards -x) is higher
    if (a->x < b->x)
        return 1;

    if (a->x > b->x)
        return -1;

    return 0;
}

// filter list of jetways jw[] for candidates and add them to nearest_jw[]
static void
filter_candidates(sam_jw_t *jw, int n_jw, const door_info_t *door_info, float *dist_threshold)
{
    // Unfortunately maxExtent in sam.xml can be bogus (e.g. FlyTampa EKCH)
    // So we find the nearest jetways on the left and do some heuristics

    for (;n_jw-- > 0; jw++) {
        if (jw->obj_ref_gen < ref_gen)  // not visible -> not dockable
            continue;

        log_msg("%s door %d, global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f",
                jw->name, jw->door, jw->x, jw->z, jw->y, jw->psi);

        jw_ctx_t tentative_njw = {0};
        jw_ctx_t *njw = &tentative_njw;
        njw->jw = jw;
        jw_ctx_for_door(njw, door_info);

        if (njw->x > 1.0f || BETWEEN(RA(njw->psi + jw->initialRot1), -130.0f, 20.0f) ||   // on the right side or pointing away
            njw->x < -80.0f || fabsf(njw->z) > 80.0f) {             // or far away
            if (fabsf(njw->x) < 120.0f && fabsf(njw->z) < 120.0f)   // don't pollute the log with jws VERY far away
                log_msg("too far or pointing away: %s, x: %0.2f, z: %0.2f, (njw->psi + jw->initialRot1): %0.1f",
                        jw->name, njw->x, njw->z, njw->psi + jw->initialRot1);
            continue;
        }

        if (njw->z > *dist_threshold)
            continue;

        if (!(BETWEEN(njw->tgt_rot1, jw->minRot1, jw->maxRot1) && BETWEEN(njw->tgt_rot2, jw->minRot2, jw->maxRot2)
            && BETWEEN(njw->tgt_extent, jw->minExtent, jw->maxExtent))) {
            log_msg("jw: %s for door %d, rot1: %0.1f, rot2: %0.1f, rot3: %0.1f, extent: %0.1f",
                     jw->name, jw->door, njw->tgt_rot1, njw->tgt_rot2, njw->tgt_rot3, njw->tgt_extent);
            log_msg("  does not fulfil min max criteria in sam.xml");
            float extra_extent = njw->tgt_extent - jw->maxExtent;
            if (extra_extent < 10.0f) {
                log_msg("  as extra extent of %0.1f m < 10.0 m we take it as a soft match", extra_extent);
                njw->soft_match = 1;
            } else
                continue;
        }

        // add to list
        log_msg("--> candidate %s, lib_id: %d, door %d, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, "
                "rot1: %0.1f, extent: %.1f",
                jw->name, jw->library_id, jw->door,
                njw->x, njw->z, njw->y, njw->psi, njw->tgt_rot1, njw->tgt_extent);
        nearest_jw[n_nearest] = *njw;
        n_nearest++;

        // if full, sort by dist and trim down to NEAR_JW_LIMIT
        if (n_nearest == MAX_NEAREST) {
            qsort(nearest_jw, MAX_NEAREST, sizeof(jw_ctx_t), njw_compar);
            n_nearest = NEAR_JW_LIMIT;
            *dist_threshold = nearest_jw[NEAR_JW_LIMIT - 1].z;
        }
    }
}

// find nearest jetways, order by z (= door number, hopefully)
static int
find_nearest_jws()
{
    if (n_door == 0) {
        log_msg("acf has no doors!");
        return 0;
    }

    // in case we move from a SAM airport to one with XP12 default
    // or autogate jetways this test never executes in the data accessors
    // so we may end up with a stale zc_jws table here
    check_ref_frame_shift();

    // compute the 'average' door location
    door_info_t avg_di;
    avg_di.x = 0.0f;
    avg_di.z = 0.0f;
    for (int i = 0; i < n_door; i++) {
        avg_di.x += door_info[i].x;
        avg_di.z += door_info[i].z;
    }

    avg_di.x /= n_door;
    avg_di.z /= n_door;
    avg_di.y = door_info[0].y;

    n_nearest = 0;
    float dist_threshold = 1.0E10f;

    // custom jws
    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
        filter_candidates(sc->sam_jws, sc->n_sam_jws, &avg_di, &dist_threshold);

    // and zero config jetways
    filter_candidates(zc_jws, zc_n_jws, &avg_di, &dist_threshold);

    if (n_nearest > 1) { // final sort + trim down to limit
        qsort(nearest_jw, n_nearest, sizeof(jw_ctx_t), njw_compar);
        n_nearest = MIN(n_nearest, NEAR_JW_LIMIT);
    }

    // fake names for zc jetways
    for (int i = 0; i < n_nearest; i++) {
        sam_jw_t *jw = nearest_jw[i].jw;
        if (jw->is_zc_jw) {
            stand_t *stand = jw->stand;
            if (stand) {
                // stand->id can be eveything from "A11" to "A11 - Terminal 1 (cat C)"
                char buf[sizeof(stand->id)];
                strcpy(buf, stand->id);
                // truncate at ' ' or after 10 chars max
                char *cptr = strchr(buf, ' ');
                if (cptr)
                    *cptr = '\0';
                int len = strlen(buf);
                if (len > 10)
                    buf[10] = '\0';
                snprintf(jw->name, sizeof(jw->name) -1, "%s_%c", buf, i + 'A');
            } else
                snprintf(jw->name, sizeof(jw->name) -1, "zc_%c", i + 'A');
        }
    }

    return n_nearest;
}

// det of 2 column vectors x,y
static inline float
det(float x1, float x2, float y1, float y2)
{
    return x1 * y2 - x2 * y1;
}

// check whether extended nearest jw i would crash into parked jw j
static
int jw_collision_check(int i, int j)
{
    // S = start, E = extended, P = parked; all (x, z) vectors
    // we solve
    //  S1 + s * (E1 - S1) = S2 + t * (P2 - S2)
    //  s * (E1 - S1) + t * -(P2 - S2) = S2 - S1
    //          A                B          C
    // if the solutions for s, t are in [0,1] there is collision

    const jw_ctx_t *njw1 = &nearest_jw[i];
    const jw_ctx_t *njw2 = &nearest_jw[j];

    // x, z in the door frame
    float A1 = njw1->tgt_x - njw1->x;
    float A2 =             - njw1->z;   // tgt_z is 0 in the door frame

    float B1 = -(njw2->parked_x - njw2->x);
    float B2 = -(njw2->parked_z - njw2->z);

    float C1 = njw2->x - njw1->x;
    float C2 = njw2->z - njw1->z;

    float d = det(A1, A2, B1, B2);
    if (fabsf(d) < 0.2f)
        return 0;

    float s = det(C1, C2, B1, B2) / d;
    float t = det(A1, A2, C1, C2) / d;
    log_msg("check between jw %d and %d, s = %0.2f, t = %0.2f", i, j, s, t);

    if (BETWEEN(t, 0.0f, 1.0f) && BETWEEN(s, 0.0f, 1.0f)) {
        log_msg("collision between jw %d and %d, s = %0.2f, t = %0.2f", i, j, s, t);
        return 1;
    }

    return 0;
}

// auto select active jetways
static void
select_jws()
{
    if (n_door == 0)
        return;

    int have_hard_match = 0;
    for (int i = 0; i < n_nearest; i++)
        if (!nearest_jw[i].soft_match) {
            have_hard_match = 1;
            break;
        }

    int i_door = 0;
    int i_jw = 0;
    while (i_jw < n_nearest) {
        if (have_hard_match && nearest_jw[i_jw].soft_match)
            goto skip;

        // skip over collisions
        for (int j = i_jw + 1; j < n_nearest; j++)
            if (jw_collision_check(i_jw, j))
                goto skip;

        active_jw[i_door] = nearest_jw[i_jw];
        log_msg("active jetway for door %d: %s", i_door, active_jw[i_door].jw->name);
        i_door++;
        if (i_door >= n_door)
            break;

      skip:
        i_jw++;
    }

    n_active_jw = i_door;   // for the auto select case
    if (n_active_jw == 0)
        log_msg("Oh no, no active jetways left in select_jws()!");
}

static int
rotate_wheel_base(jw_ctx_t *ajw, float dt)
{
    sam_jw_t *jw = ajw->jw;

    float delta_rot = RA(ajw->wb_rot - jw->wheelrotatec);

    // optimize rotation
    if (delta_rot > 90.0f)
        delta_rot -= 180.0f;
    else if (delta_rot < -90.0f)
        delta_rot += 180.0f;

    //log_msg("wb_rot: %0.2f, delta_rot: %0.2f, wheelrotatec: %0.2f",
    //        ajw->wb_rot, delta_rot, jw->wheelrotatec);


    // wheel base rotation
    int must_wait = 0;
    float d_rot;
    if (fabsf(delta_rot) > 2.0f) {
        d_rot = dt * JW_TURN_SPEED;
        //log_msg("turning wheel base by %0.2f°", d_rot);
        if (delta_rot < 0.0f)
            d_rot = -d_rot;

        jw->wheelrotatec += d_rot;

        must_wait = 1;    // must wait
    } else {
        d_rot = delta_rot;
        jw->wheelrotatec += delta_rot;
    }

    float da_rot = d_rot * (jw->wheelDistance / jw->wheelDiameter);

    jw->wheelrotatel += da_rot;
    jw->wheelrotater -= da_rot;
    return must_wait;
}

// rotation1 + extend
static void
rotate_1_extend(jw_ctx_t *ajw, float cabin_x, float cabin_z)
{
    sam_jw_t *jw = ajw->jw;

    jw_xy_to_sam_dr(ajw, cabin_x, cabin_z, &jw->rotate1, &jw->extent, NULL, NULL);
    jw_set_wheels(jw);
}

// rotation 3
// return 1 when done
static int
rotate_3(jw_ctx_t *ajw, float tgt_rot3, float dt)
{
    sam_jw_t *jw = ajw->jw;

    if (fabsf(jw->rotate3 - tgt_rot3) > 0.1) {
        float d_rot3 = (dt * JW_HEIGHT_SPEED / (jw->cabinPos + jw->extent)) / D2R;  // strictly it's atan
        if (jw->rotate3 >= tgt_rot3)
            jw->rotate3 = MAX(jw->rotate3 - d_rot3, tgt_rot3);
        else
            jw->rotate3 = MIN(jw->rotate3 + d_rot3, tgt_rot3);
    }

    jw_set_wheels(jw);

    if (fabsf(jw->rotate3 - tgt_rot3) > 0.1f)
        return 0;

    jw->rotate3 = tgt_rot3;
    return 1;
}

// rotation 2
// return 1 when done
static int
rotate_2(jw_ctx_t *ajw, float tgt_rot2, float dt) {
    sam_jw_t *jw = ajw->jw;

    if (fabsf(jw->rotate2 - tgt_rot2) > 0.5) {
        float d_rot2 = dt * JW_TURN_SPEED;
        if (jw->rotate2 >= tgt_rot2)
            jw->rotate2 = MAX(jw->rotate2 - d_rot2, tgt_rot2);
        else
            jw->rotate2 = MIN(jw->rotate2 + d_rot2, tgt_rot2);
        return fabsf(jw->rotate2 - tgt_rot2) <= 0.5;
    }

    jw->rotate2 = tgt_rot2;
    return 1;
}

// animate wheels for straight driving
static void
animate_wheels(jw_ctx_t *ajw, float ds)
{
    sam_jw_t *jw = ajw->jw;

    if (fabsf(RA(ajw->wb_rot - jw->wheelrotatec)) > 90.0f)
        ds = -ds;
    //log_msg("wb_rot: %0.2f, wheelrotatec: %0.2f, ds: 0.3f", ajw->wb_rot, jw->wheelrotatec, ds);

    float da_ds = (ds / jw->wheelDiameter) / D2R;

    jw->wheelrotatel += da_ds;
    jw->wheelrotater += da_ds;
}

// drive jetway to the door
// return 1 when done
static int
dock_drive(jw_ctx_t *ajw)
{
    sam_jw_t *jw = ajw->jw;

    if (ajw->state == AJW_DOCKED)
        return 1;

    if (now < ajw->start_ts)
        return 0;

    // guard against a hung animation
    if (now > ajw->timeout) {
        log_msg("dock_drive() timeout!");
        ajw->state = AJW_DOCKED;
        jw->rotate1 = ajw->tgt_rot1;
        jw->rotate2 = ajw->tgt_rot2;
        jw->rotate3 = ajw->tgt_rot3;
        jw->extent = ajw->tgt_extent;
        jw->warnlight = 0;
        alert_off(ajw);
        return 1;   // -> done
    }

    float dt = now - ajw->last_step_ts;
    ajw->last_step_ts = now;

    float rot1_d = RA((jw->rotate1 + ajw->psi) - 90.0f);    // door frame

    //float wheel_x = ajw->x + (jw->extent + jw->wheelPos) * cosf(rot1_d * D2R);
    //float wheel_z = ajw->z + (jw->extent + jw->wheelPos) * sinf(rot1_d * D2R);

    if (ajw->state == AJW_TO_AP) {
        if (ajw->wait_wb_rot) {
            //log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(ajw, dt))
                return 0;
            ajw->wait_wb_rot = 0;
        }

        float tgt_x = ajw->ap_x;

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - ajw->cabin_x), fabs(ajw->cabin_z));
        if (fabs(tgt_x - ajw->cabin_x) < eps && fabs(ajw->cabin_z) < eps)  {
            ajw->state = AJW_AT_AP;
            log_msg("align point reached reached");
            return 0;
        }

        double ds = dt * JW_DRIVE_SPEED;

        // Well, the wheels are somewhat behind the cabin so this is only approximate
        // but doesn't make much of a difference.
        double drive_angle = atan2(-ajw->cabin_z, tgt_x - ajw->cabin_x) / D2R;

        // wb_rot is drive_angle in the 'tunnel frame'
        float wb_rot = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw->extent <= jw->minExtent && wb_rot < -90.0f) {
            wb_rot = -90.0f;
            drive_angle = RA(rot1_d + -90.0f);
        }

        ajw->wb_rot = wb_rot;
        ajw->cabin_x += cos(drive_angle * D2R) * ds;
        ajw->cabin_z += sin(drive_angle * D2R) * ds;


        //log_msg("to ap: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, drive_angle: %0.2f, wb_rot: %0.2f",
        //        rot1_d, ajw->cabin_x, ajw->cabin_z, drive_angle, ajw->wb_rot);

        if (rotate_wheel_base(ajw, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        // rotation2
        float tgt_rot2 = ajw->tgt_rot2;
        if (ajw->cabin_x < (tgt_x - 1.0f) || ajw->cabin_z < -2.0f) {
            float angle_to_door = atan2f(-ajw->cabin_z, ajw->tgt_x - ajw->cabin_x) / D2R;
            tgt_rot2 = RA(angle_to_door + 90.0f - ajw->psi - jw->rotate1); // point to door
        }
        //log_msg("jw->rotate2: %0.1f, ajw->tgt_rot2: %0.1f, tgt_rot2: %0.1f", jw->rotate2, ajw->tgt_rot2, tgt_rot2);

        rotate_2(ajw, tgt_rot2, dt);
        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        rotate_3(ajw, ajw->tgt_rot3, dt);
        animate_wheels(ajw, ds);
    }

    if (ajw->state == AJW_AT_AP) {
        // use the time to rotate the wheel base towards the door
        ajw->wb_rot = RA(-rot1_d);
        rotate_wheel_base(ajw, dt);

        // rotation 2 + 3 must be at target now
        if (rotate_2(ajw, ajw->tgt_rot2, dt) && rotate_3(ajw, ajw->tgt_rot3, dt))
            ajw->state = AJW_TO_DOOR;
    }

    if (ajw->state == AJW_TO_DOOR) {
        if (ajw->wait_wb_rot) {
            // log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(ajw, dt))
                return 0;
            ajw->wait_wb_rot = 0;
        }

        float tgt_x = ajw->tgt_x;

        ajw->cabin_x = MIN(ajw->cabin_x, tgt_x); // dont drive beyond the target point

        //log_msg("to door: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f", rot1_d, ajw->cabin_x, ajw->cabin_z);

        // ramp down speed when approaching the plane
        float drive_speed = JW_DRIVE_SPEED;
        if (ajw->cabin_x >= (ajw->tgt_x - 0.8f))
            drive_speed = JW_DRIVE_SPEED * (0.1f + 0.9f * MAX(0.0f, (ajw->tgt_x - ajw->cabin_x) / 0.8f));

        float ds = dt * drive_speed;

        ajw->cabin_x += ds;
        //log_msg("ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f", ajw->cabin_x, ajw->cabin_z);

        ajw->wb_rot = RA(-rot1_d);
        if (rotate_wheel_base(ajw, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        animate_wheels(ajw, ds);

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.05f);
        //log_msg("eps: %0.3f, d_x: %0.3f", eps, fabs(tgt_x - ajw->cabin_x));
        if (fabs(tgt_x - ajw->cabin_x) < eps) {
            ajw->state = AJW_DOCKED;
            log_msg("door reached");
            jw->warnlight = 0;
            alert_off(ajw);
            return 1;   // done
        }
    }

    alert_setpos(ajw);
    return 0;
}


// drive jetway to parked position
// return 1 when done
static float
undock_drive(jw_ctx_t *ajw)
{
    sam_jw_t *jw = ajw->jw;

    if (ajw->state == AJW_PARKED)
        return 1;

    if (now < ajw->start_ts)
        return 0;

    // guard against a hung animation
    if (now > ajw->timeout) {
        log_msg("undock_drive() timeout!");
        ajw->state = AJW_PARKED;
        jw->rotate1 = jw->initialRot1;
        jw->rotate2 = jw->initialRot2;
        jw->rotate3 = jw->initialRot3;
        jw->extent = jw->initialExtent;
        jw->warnlight = 0;
        alert_off(ajw);
        return 1;   // -> done
    }

    float dt = now - ajw->last_step_ts;
    ajw->last_step_ts = now;

    float rot1_d = RA((jw->rotate1 + ajw->psi) - 90.0f);    // door frame

    //float wheel_x = ajw->x + (jw->extent + jw->wheelPos) * cosf(rot1_d * D2R);
    //float wheel_z = ajw->z + (jw->extent + jw->wheelPos) * sinf(rot1_d * D2R);

    if (ajw->state == AJW_TO_AP) {
        if (ajw->wait_wb_rot) {
            //log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(ajw, dt)) {
                return 0;
            }
            ajw->wait_wb_rot = 0;
        }

        float tgt_x = ajw->ap_x;

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - ajw->cabin_x), fabs(ajw->cabin_z));
        if (fabs(tgt_x - ajw->cabin_x) < eps && fabs(ajw->cabin_z) < eps)  {
            ajw->state = AJW_AT_AP;
            log_msg("align point reached reached");
            return 0;
        }

        double ds = dt * 0.5 * JW_DRIVE_SPEED;
        double drive_angle = atan2(-ajw->cabin_z, tgt_x - ajw->cabin_x) / D2R;

        ajw->cabin_x += cos(drive_angle * D2R) * ds;
        ajw->cabin_z += sin(drive_angle * D2R) * ds;
        //log_msg("to ap: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle: %0.2f",
        //        rot1_d, ajw->cabin_x, ajw->cabin_z, wheel_x, wheel_z, drive_angle);

        ajw->wb_rot = RA(drive_angle - rot1_d);
        if (rotate_wheel_base(ajw, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        animate_wheels(ajw, ds);
    }

    if (ajw->state == AJW_AT_AP) {
        // nothing for now
        ajw->state = AJW_TO_PARK;
    }

    if (ajw->state == AJW_TO_PARK) {
        if (ajw->wait_wb_rot) {
            // log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(ajw, dt)) {
                return 0;
            }
            ajw->wait_wb_rot = 0;
        }

        float tgt_x = ajw->parked_x;
        float tgt_z = ajw->parked_z;

        //log_msg("to park: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f",
        //        rot1_d, ajw->cabin_x, ajw->cabin_z, wheel_x, wheel_z);

        double ds = dt * JW_DRIVE_SPEED;
        double drive_angle = atan2(tgt_z - ajw->cabin_z, tgt_x - ajw->cabin_x) / D2R;

        // wb_rot is drive_angle in the 'tunnel frame'
        float wb_rot = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw->extent <= jw->minExtent && wb_rot > 90.0f) {
            wb_rot = 90.0f;
            drive_angle = RA(rot1_d + 90.0f);
        }

        ajw->wb_rot = wb_rot;

        ajw->cabin_x += cos(drive_angle * D2R) * ds;
        ajw->cabin_z += sin(drive_angle * D2R) * ds;
        //log_msg("to parked: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle: %0.2f",
        //       rot1_d, ajw->cabin_x, ajw->cabin_z, wheel_x, wheel_z, drive_angle);

        if (rotate_wheel_base(ajw, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        rotate_2(ajw, jw->initialRot2, dt);
        rotate_3(ajw, jw->initialRot3, dt);
        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        animate_wheels(ajw, ds);

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - ajw->cabin_x), fabs(tgt_z - ajw->cabin_z));
        if (fabs(tgt_x - ajw->cabin_x) < eps && fabs(tgt_z -ajw->cabin_z) < eps)  {
            ajw->state = AJW_PARKED;
            jw->warnlight = 0;
            alert_off(ajw);
            log_msg("park position reached");
            return 1;   // done
        }
    }

    alert_setpos(ajw);
    return 0;
}

// the state machine called from the flight loop
float
jw_state_machine()
{
    if (state == DISABLED)
        return 2.0;

    state_t new_state = state;

    int beacon_on = check_beacon();

    if (state > IDLE && check_teleportation()) {
        log_msg("teleportation detected!");
        state = new_state = IDLE;
        reset_jetways();
    }

    int n_done;
    char airport_id[50];
    float lat, lon;

    switch (state) {
        case IDLE:
            if (prev_state != IDLE) {
                n_active_jw = 0;
                memset(active_jw, 0, sizeof(active_jw));
                n_nearest = 0;
                memset(nearest_jw, 0, sizeof(nearest_jw));
            }

            if (on_ground && !beacon_on) {
                plane_x = XPLMGetDataf(plane_x_dr);
                plane_y = XPLMGetDataf(plane_y_dr);
                plane_z = XPLMGetDataf(plane_z_dr);
                plane_psi = XPLMGetDataf(plane_true_psi_dr);

                sin_psi = sinf(D2R * plane_psi);
                cos_psi = cosf(D2R * plane_psi);

                parked_x = plane_x;
                parked_z = plane_z;
                parked_ngen = ref_gen;

                // reset stale command invocations
                dock_requested = undock_requested = toggle_requested = 0;

                new_state = PARKED;
            }
            break;

        case PARKED:
            // find airport I'm on now to ease debugging
            lat = XPLMGetDataf(plane_lat_dr);
            lon = XPLMGetDataf(plane_lon_dr);
            XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
            if (XPLM_NAV_NOT_FOUND != ref) {
                XPLMGetNavAidInfo(ref, NULL, NULL, NULL, NULL, NULL, NULL, airport_id,
                        NULL, NULL);
                log_msg("parked on airport: %s, lat,lon: %0.5f,%0.5f", airport_id, lat, lon);
            }

            if (find_nearest_jws())
                new_state = SELECT_JWS;
            else
                new_state = CANT_DOCK;
            break;

        case SELECT_JWS:
            if (beacon_on) {
                log_msg("SELECT_JWS and beacon goes on");
                new_state = IDLE;
                break;
            }

            if (auto_select_jws) {
                select_jws();
            } else if (prev_state != state) {
                ui_unlocked = 1;    // allow jw selection in the ui
                update_ui(1);
            }

            // or wait for GUI selection
            if (n_active_jw) {
                for (int i = 0; i < n_door; i++) {
                    jw_ctx_t *ajw = &active_jw[i];
                    sam_jw_t *jw = ajw->jw;

                    if (NULL == jw)
                        continue;

                    log_msg("setting up active jw for door: %d", i);
                    jw_ctx_for_door(ajw, &door_info[i]);
                    if (i == 0) // slightly slant towards the nose cone for door LF1
                        ajw->tgt_rot2 += 3.0f;
                }

                new_state = CAN_DOCK;
            }
            break;

        case CAN_DOCK:
            if (beacon_on) {
                log_msg("CAN_DOCK and beacon goes on");
                new_state = IDLE;
            }

            if (1 == dock_requested || toggle_requested) {
                log_msg("docking requested");
                int active_door = 0;
                for (int i = 0; i < n_door; i++) {
                    jw_ctx_t *ajw = &active_jw[i];
                    if (NULL == ajw->jw)
                        continue;

                    ajw->state = AJW_TO_AP;
                    // staggered start for docking low to high
                    ajw->start_ts = now + active_door * 5.0f;
                    ajw->last_step_ts = ajw->start_ts;
                    ajw->timeout = ajw->start_ts + JW_ANIM_TIMEOUT;
                    alert_on(ajw);
                    ajw->jw->warnlight = 1;
                    active_door++;
                }

                new_state = DOCKING;
            }
            break;

        case CANT_DOCK:
            if (!on_ground || beacon_on) {
                new_state = IDLE;
                break;
            }
            break;

        case DOCKING:
            n_done = 0;
            for (int i = 0; i < n_door; i++) {
                jw_ctx_t *ajw = &active_jw[i];
                if (NULL == ajw->jw)
                    continue;

                n_done += dock_drive(ajw);
            }

            if (n_done == n_active_jw) {
                XPLMCommandRef cmdr = XPLMFindCommand("openSAM/post_dock");
                if (cmdr)
                    XPLMCommandOnce(cmdr);
                new_state = DOCKED;
            }
            else
                return JW_ANIM_INTERVAL;
            break;

        case DOCKED:
            if (!on_ground) {
                new_state = IDLE;
                break;
            }

            if (beacon_on) {
                log_msg("DOCKED and beacon goes on");
                undock_requested = 1;
            }

            if (1 == undock_requested || toggle_requested) {
                log_msg("undocking requested");
                int active_door = n_door;
                for (int i = 0; i < n_door; i++) {
                    jw_ctx_t *ajw = &active_jw[i];
                    if (NULL == ajw->jw)
                        continue;

                    active_door--;
                    ajw->state = AJW_TO_AP;
                    // staggered start for undocking high to low
                    ajw->start_ts = now + active_door * 5.0f;
                    ajw->last_step_ts = ajw->start_ts;
                    ajw->timeout = ajw->start_ts + JW_ANIM_TIMEOUT;
                    alert_on(ajw);
                    ajw->jw->warnlight = 1;
                }

                XPLMCommandRef cmdr = XPLMFindCommand("openSAM/pre_undock");
                if (cmdr)
                    XPLMCommandOnce(cmdr);

                new_state = UNDOCKING;
            }
            break;

        case UNDOCKING:
            n_done = 0;
            for (int i = 0; i < n_door; i++) {
                jw_ctx_t *ajw = &active_jw[i];
                if (NULL == ajw->jw)
                    continue;
                n_done += undock_drive(&active_jw[i]);
            }

            if (n_done == n_active_jw)
               new_state = IDLE;
            else
                return JW_ANIM_INTERVAL;
            break;

        default:
            log_msg("Bad state %d", state);
            new_state = DISABLED;
            break;
    }

    // we use an extra cycle for dock/undock for better integration with the UI
    dock_requested--;
    undock_requested--;
    toggle_requested = 0;

    prev_state = state;

    if (new_state != state) {
        log_msg("jw state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;

        // from anywhere to idle nullifies all selections
        if (state == IDLE) {
            n_active_jw = 0;
            memset(active_jw, 0, sizeof(active_jw));
            n_nearest = 0;
        }

        ui_unlocked = 0;
        update_ui(1);
        return -1;  // see you on next frame
    }

    return 0.5;
}

static int
cmd_dock_jw_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(cmdr);
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd_dock_jw_cb called");

    *(int *)ref = 2;
     return 0;
}

// intercept XP12's standard cmd
static int
cmd_xp12_dock_jw_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(cmdr);
    UNUSED(ref);

    if (xplm_CommandBegin != phase)
        return 1;

    log_msg("cmd_xp12_dock_jw_cb called");

    if (CAN_DOCK == state || DOCKED == state)
        toggle_requested = 2;

    return 1;       // pass on to XP12, likely there is no XP12 jw here 8-)
}

int
jw_init()
{
    // load alert sound
    char fn[sizeof(base_dir) + 100];
    strcpy(fn, base_dir);
    strcat(fn, "sound/alert.wav");
    read_wav(fn, &alert);
    if (alert.data)
        log_msg("alert sound loaded, channels: %d, bit_rate: %d, size: %d",
                alert.num_channels, alert.sample_rate, alert.size);
    else {
        log_msg("Could not load sound");
        return 0;
    }

    if (!sound_init())
        return 0;

    dock_cmdr = XPLMCreateCommand("openSAM/dock_jwy", "Dock jetway");
    XPLMRegisterCommandHandler(dock_cmdr, cmd_dock_jw_cb, 0, &dock_requested);

    undock_cmdr = XPLMCreateCommand("openSAM/undock_jwy", "Undock jetway");
    XPLMRegisterCommandHandler(undock_cmdr, cmd_dock_jw_cb, 0, &undock_requested);

    toggle_cmdr = XPLMCreateCommand("openSAM/toggle_jwy", "Toggle jetway");
    XPLMRegisterCommandHandler(toggle_cmdr, cmd_dock_jw_cb, 0, &toggle_requested);

    // augment XP12's standard cmd
    XPLMCommandRef xp12_toggle_cmdr = XPLMFindCommand("sim/ground_ops/jetway");
    if (xp12_toggle_cmdr)
        XPLMRegisterCommandHandler(xp12_toggle_cmdr, cmd_xp12_dock_jw_cb, 1, NULL);

    // create the jetway animation datarefs
    for (dr_code_t drc = DR_ROTATE1; drc < N_JW_DR; drc++) {
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

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, jw_status_acc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/number", xplmType_Int, 0, jw_status_acc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, &n_active_jw, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/door/status", xplmType_IntArray, 0, NULL, NULL,
                             NULL, NULL, NULL, NULL, jw_door_status_acc, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL);

    reset_jetways();
    srand(time(NULL));
    return 1;
}
