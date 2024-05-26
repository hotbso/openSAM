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
#include <sys/types.h>

#include "openSAM.h"
#include "os_jw.h"

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
static state_t prev_state = IDLE;

// keep in sync!
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
active_jw_t active_jw[MAX_DOOR];

active_jw_t nearest_jw[MAX_DOOR][MAX_NEAREST];
int n_nearest[MAX_DOOR];

static sound_t alert;

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
read_jw_acc(void *ref)
{
    stat_acc_called++;

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);
    float elevation = XPLMGetDataf(plane_elevation_dr);

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);

    // check for shift of reference frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        log_msg("reference frame shift");
    }

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        // cheap check against bounding box
        if (lat < sc->bb_lat_min || lat > sc->bb_lat_max
            || RA(lon - sc->bb_lon_min) < 0 || RA(lon - sc->bb_lon_max) > 0) {
            stat_sc_far_skip++;
            continue;
        }

        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            // cheap check against bounding box
            if (lat < jw->bb_lat_min || lat > jw->bb_lat_max
                || RA(lon - jw->bb_lon_min) < 0 || RA(lon - jw->bb_lon_max) > 0) {
                stat_far_skip++;
                continue;
            }

            if (jw->xml_ref_gen < ref_gen) {
                XPLMWorldToLocal(jw->latitude, jw->longitude, elevation, &jw->xml_x, &jw->xml_y, &jw->xml_z);
                jw->xml_ref_gen = ref_gen;
            }

            if (fabs(obj_x - jw->xml_x) > NEAR_SKIP || fabs(obj_z - jw->xml_z) > NEAR_SKIP) {
                stat_near_skip++;
                continue;
            }

            // have a match
            if (jw->obj_ref_gen < ref_gen) {
                // use higher precision values of the actually drawn object
                jw->obj_ref_gen = ref_gen;
                jw->x = obj_x;
                jw->z = obj_z;
                jw->y = XPLMGetDataf(draw_object_y_dr);
                jw->psi = XPLMGetDataf(draw_object_psi_dr);
            }

            stat_jw_match++;

            uint64_t ctx = (uint64_t)ref;
            dr_code_t drc = ctx & 0xffffffff;
            int id = ctx >> 32;

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

            return 0.0;
        }
    }

    return 0.0;
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

static void
alert_complete(void *ref, FMOD_RESULT status)
{
    log_msg("fmod callback: %d", status);

    active_jw_t *ajw = ref;
    ajw->alert_chn = NULL;
}


static void
alert_on(active_jw_t *ajw)
{
    if (ajw->alert_chn)
        return;
    ajw->alert_chn = XPLMPlayPCMOnBus(alert.data, alert.size, FMOD_SOUND_FORMAT_PCM16,
                                      alert.sample_rate, alert.num_channels, 1,
                                      xplm_AudioExteriorUnprocessed,
                                      alert_complete, ajw);
#if 0
    sam_jw_t *jw = ajw->jw;

    static FMOD_VECTOR velocity = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR position = {jw->xml_x, jw->xml_y + 1.5f, jw->xml_z};

    XPLMSetAudioPosition(ajw->alert_chn, &position, &velocity);
#endif
}

static void
alert_off(active_jw_t *ajw)
{
    if (ajw->alert_chn)
        XPLMStopAudio(ajw->alert_chn);
    ajw->alert_chn = NULL;
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
            jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);
            jw->warnlight = 0;
            //log_msg("%s %5.6f %5.6f", jw->name, jw->latitude, jw->longitude);
       }

    for (int i = 0; i < n_door; i++) {
        active_jw_t *ajw = &active_jw[i];
        if (ajw->jw)
            alert_off(ajw);
    }

    n_active_jw = 0;
    memset(active_jw, 0, sizeof(active_jw));
}

// convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 can be NULL
static inline void
jw_xy_to_sam_dr(const active_jw_t *ajw, float cabin_x, float cabin_z,
                float *rot1, float *extent, float *rot2, float *rot3)
{
    const sam_jw_t *jw = ajw->jw;

    float dist = len2f(cabin_x - ajw->x, cabin_z - ajw->z);

    //float rot1_d = asinf((cabin_z - ajw->z)/ dist) / D2R;   // door frame
    float rot1_d = atan2(cabin_z - ajw->z, cabin_x - ajw->x) / D2R;   // door frame
    *rot1 =  RA(rot1_d + 90.0f - ajw->psi);
    *extent = dist - jw->cabinPos;

    // angle 0° door frame  -> hdgt -> jw frame -> diff to rot1
    float r2 = RA(0.0f + 90.0f - ajw->psi - *rot1);
    if (rot2)
        *rot2 = r2;

    if (rot3) {
        float net_length = dist + jw->cabinLength * cosf(r2 * D2R);
        *rot3 = RA(-asinf(ajw->y / net_length) / D2R);
    }
}

static int
ajw_compar(const void *a, const void *b)
{
    const active_jw_t *ajw_a = a;
    const active_jw_t *ajw_b = b;

    if (ajw_a->dist < ajw_b->dist)
        return -1;

    if (ajw_a->dist == ajw_b->dist)
        return 0;

    return 1;
}

// find nearest jetways per door and save their info
static int
find_nearest_jws()
{
    if (n_door == 0) {
        log_msg("acf has no doors!");
        return 0;
    }

    int jws_found = 0;
    n_active_jw = 0;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_y = XPLMGetDataf(plane_y_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);
    float plane_psi = XPLMGetDataf(plane_true_psi_dr);

    float sin_psi = sinf(D2R * plane_psi);
    float cos_psi = cosf(D2R * plane_psi);

    n_active_jw = 0;
    memset(active_jw, 0, sizeof(active_jw));

    for (int i = 0; i < MAX_DOOR; i++)
        active_jw[i].dist = 1.0E10;

    memset(nearest_jw, 0, sizeof(nearest_jw));

    for (int idoor = 0; idoor < MAX_DOOR; idoor++) {
        n_nearest[idoor] = 0;
        float dist_threshold = 1.0E10f;

        // Unfortunately maxExtent in sam.xml can be bogus (e.g. FlyTampa EKCH)
        // So we find the nearest jetways on the left and do some heuristics
        for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
            for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
                if (jw->obj_ref_gen < ref_gen)  // not visible -> not dockable
                    continue;

                //log_msg("%s door %d, global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f",
                //        jw->name, jw->door, jw->x, jw->z, jw->y, jw->psi);

                active_jw_t tentative_ajw;
                active_jw_t *ajw = &tentative_ajw;
                memset(ajw, 0, sizeof(active_jw_t));
                ajw->jw = jw;

                // rotate into plane local frame
                float dx = jw->x - plane_x;
                float dz = jw->z - plane_z;
                ajw->x =  cos_psi * dx + sin_psi * dz;
                ajw->z = -sin_psi * dx + cos_psi * dz;
                ajw->psi = RA(jw->psi - plane_psi);

                // xlate into door local frame
                ajw->x -= door_info[idoor].x;
                ajw->z -= door_info[idoor].z;

                float rot1_d = RA((jw->initialRot1 + ajw->psi) - 90.0f);    // door frame
                ajw->cabin_x = ajw->x + (jw->extent + jw->cabinPos) * cosf(rot1_d * D2R);
                ajw->cabin_z = ajw->z + (jw->extent + jw->cabinPos) * sinf(rot1_d * D2R);

                if (ajw->cabin_x > 1.0f || BETWEEN(ajw->psi, -130.0f, 20.0f)) { // on the right side or pointing away
                    log_msg("pointing away: %s, x: %0.2f, ajw->psi: %0.1f",
                            jw->name, ajw->cabin_x, ajw->psi);
                    continue;
                }

                ajw->dist = len2f(ajw->cabin_x, ajw->cabin_z);
                if (ajw->dist > dist_threshold)
                    continue;

                ajw->tgt_x = -jw->cabinLength;
                // tgt z = 0.0
                ajw->y = (jw->y + jw->height) - (plane_y + door_info[idoor].y);

                jw_xy_to_sam_dr(ajw, ajw->tgt_x, 0.0f, &ajw->tgt_rot1, &ajw->tgt_extent, &ajw->tgt_rot2, &ajw->tgt_rot3);

                if (idoor == 0)
                    ajw->tgt_rot2 += 3.0f;  // for door1 only

                if (!(BETWEEN(ajw->tgt_rot1, jw->minRot1, jw->maxRot1) && BETWEEN(ajw->tgt_rot2, jw->minRot2, jw->maxRot2)
                    && BETWEEN(ajw->tgt_extent, jw->minExtent, jw->maxExtent))) {
                    log_msg("jw: %s for door %d, rot1: %0.1f, rot2: %0.1f, rot3: %0.1f, extent: %0.1f",
                             jw->name, jw->door, ajw->tgt_rot1, ajw->tgt_rot2, ajw->tgt_rot3, ajw->tgt_extent);
                    log_msg("  does not fulfil min max criteria in sam.xml");
                    if (ajw->dist < 50.0f)
                        log_msg("  as the distance of %0.1f m to the door is < 50.0 m we take it anyway", ajw->dist);
                    else
                        continue;
                }

                // add to list
                log_msg("candidate %s, lib_id: %d, door %d, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, extent: %.1f",
                        jw->name, jw->library_id, jw->door,
                        ajw->x, ajw->z, ajw->y, ajw->psi, ajw->tgt_extent);
                nearest_jw[idoor][n_nearest[idoor]] = tentative_ajw;
                n_nearest[idoor]++;

                // if full, sort by dist and trim down to NEAR_JW_LIMIT
                if (n_nearest[idoor] == MAX_NEAREST) {
                    qsort(&nearest_jw[idoor][0], MAX_NEAREST, sizeof(active_jw_t), ajw_compar);
                    n_nearest[idoor] = NEAR_JW_LIMIT;
                    dist_threshold = nearest_jw[idoor][NEAR_JW_LIMIT - 1].dist;
                }
            }

        // final sort + trim down to limit
        qsort(&nearest_jw[idoor][0], n_nearest[idoor], sizeof(active_jw_t), ajw_compar);
        n_nearest[idoor] = MIN(n_nearest[idoor], NEAR_JW_LIMIT);

        for (int j = 0; j < n_nearest[idoor]; j++) {
            active_jw_t *ajw = &nearest_jw[idoor][j];
            sam_jw_t *jw = ajw->jw;

            // compute x,z of parked jw
            float rot1_d = RA((jw->initialRot1 + ajw->psi) - 90.0f);    // door frame
            float r = jw->initialExtent + jw->cabinPos;
            ajw->parked_x = ajw->x + r * cosf(rot1_d * D2R);
            ajw->parked_z = ajw->z + r * sinf(rot1_d * D2R);

            ajw->ap_x = ajw->tgt_x - JW_ALIGN_DIST;

            jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);

            log_msg("door %d, nearest %s, lib_id: %d, dist %0.1f, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, extent: %.1f",
                    idoor, jw->name, jw->library_id, ajw->dist,
                    ajw->x, ajw->z, ajw->y, ajw->psi, ajw->tgt_extent);
            jws_found = 1;
        }
    }

    return jws_found;
}

// auto select active jetways
static void
select_jws()
{
    if (n_door == 0)
        return;

    // from door 0 to n assign nearest jw
    for (int i = 0; i < n_door; i++)
        if (n_nearest[i] > 0)
            active_jw[i] = nearest_jw[i][0];

    // from door n down to 0 remove jetway if it's already assigned to a lower door #
    for (int i = n_door - 1; i > 0; i--)
        for (int j = 0; j < i; j++)
            if (active_jw[i].jw == active_jw[j].jw)
                active_jw[i].jw = NULL;

    for (int i = 0; i < n_door; i++) {
        sam_jw_t *jw = active_jw[i].jw;
        if (jw) {
            log_msg("active jetway for door %d: %s", i, jw->name);
            n_active_jw++;
        }
    }
}

static int
rotate_wheel_base(active_jw_t *ajw, float dt)
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
rotate_1_extend(active_jw_t *ajw, float cabin_x, float cabin_z)
{
    sam_jw_t *jw = ajw->jw;

    jw_xy_to_sam_dr(ajw, cabin_x, cabin_z, &jw->rotate1, &jw->extent, NULL, NULL);
    jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);
}

// rotation 3
// return 1 when done
static int
rotate_3(active_jw_t *ajw, float tgt_rot3, float dt)
{
    sam_jw_t *jw = ajw->jw;

    if (fabsf(jw->rotate3 - tgt_rot3) > 0.5) {
        float d_rot3 = (dt * JW_HEIGHT_SPEED / (jw->cabinPos + jw->extent)) / D2R;  // strictly it's atan
        if (jw->rotate3 >= tgt_rot3)
            jw->rotate3 = MAX(jw->rotate3 - d_rot3, tgt_rot3);
        else
             jw->rotate3 = MIN(jw->rotate3 + d_rot3, tgt_rot3);
    }

    jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);

    if (fabsf(jw->rotate3 - tgt_rot3) > 0.5f) {
        //log_msg("jw->rotate3: %0.3f,tgt_rot3: %0.3f", jw->rotate3, tgt_rot3);
        return 0;
    }

    return 1;
}

// rotation 2
// return 1 when done
static int
rotate_2(active_jw_t *ajw, float tgt_rot2, float dt) {
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
animate_wheels(active_jw_t *ajw, float ds)
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
dock_drive(active_jw_t *ajw)
{
    sam_jw_t *jw = ajw->jw;

    if (ajw->state == AJW_DOCKED)
        return 1;

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

        ajw->cabin_x += cos(drive_angle * D2R) * ds;
        ajw->cabin_z += sin(drive_angle * D2R) * ds;

        // wb_rot is drive_angle in the 'tunnel frame'
        ajw->wb_rot = RA(drive_angle - rot1_d);

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

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - ajw->cabin_x), fabs(ajw->cabin_z));
        if (fabs(tgt_x - ajw->cabin_x) < eps && fabs(ajw->cabin_z) < eps)  {
            ajw->state = AJW_DOCKED;
            log_msg("door reached");
            jw->warnlight = 0;
            alert_off(ajw);
            return 1;   // done
        }
    }

    return 0;
}


// drive jetway to parked position
// return 1 when done
static float
undock_drive(active_jw_t *ajw)
{
    sam_jw_t *jw = ajw->jw;

    if (ajw->state == AJW_PARKED)
        return 1;

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

        ajw->cabin_x += cos(drive_angle * D2R) * ds;
        ajw->cabin_z += sin(drive_angle * D2R) * ds;
        //log_msg("to parked: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle: %0.2f",
        //       rot1_d, ajw->cabin_x, ajw->cabin_z, wheel_x, wheel_z, drive_angle);

        ajw->wb_rot = RA(drive_angle - rot1_d);
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

    return 0;
}

// the state machine triggered by the flight loop
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
            if (on_ground && !beacon_on) {
                parked_x = XPLMGetDataf(plane_x_dr);
                parked_y = XPLMGetDataf(plane_y_dr);
                parked_ngen = ref_gen;
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
                log_msg("parked on airport: %s, lat,lon: %0.4f,%0.4f", airport_id, lat, lon);
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
            if (n_active_jw)
                new_state = CAN_DOCK;
            break;

        case CAN_DOCK:
            if (beacon_on) {
                log_msg("CAN_DOCK and beacon goes on");
                new_state = IDLE;
            }

            if (dock_requested || toggle_requested) {
                log_msg("docking requested");
                for (int i = 0; i < n_door; i++) {
                    active_jw_t *ajw = &active_jw[i];
                    if (NULL == ajw->jw)
                        continue;
                    ajw->state = AJW_TO_AP;
                    ajw->last_step_ts = now;
                    ajw->timeout = now + JW_ANIM_TIMEOUT;
                    alert_on(ajw);
                    ajw->jw->warnlight = 1;
                    new_state = DOCKING;
                }
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
                active_jw_t *ajw = &active_jw[i];
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

            if (undock_requested || toggle_requested) {
                log_msg("undocking requested");
                for (int i = 0; i < n_door; i++) {
                    active_jw_t *ajw = &active_jw[i];
                    if (NULL == ajw->jw)
                        continue;
                    ajw->state = AJW_TO_AP;
                    ajw->last_step_ts = now;
                    ajw->timeout = now + JW_ANIM_TIMEOUT;
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
                active_jw_t *ajw = &active_jw[i];
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

    dock_requested = undock_requested = toggle_requested = 0;

    prev_state = state;

    if (new_state != state) {
        log_msg("jw state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;

        // from anywhere to idle nullifies all selections
        if (state == IDLE) {
            n_active_jw = 0;
            memset(active_jw, 0, sizeof(active_jw));
            memset(n_nearest, 0, sizeof(n_nearest));
        }

        ui_unlocked = 0;
        update_ui(1);
        return -1;  // see you on next frame
    }

    return 0.5;
}

int
jw_init()
{
    // create the jetway animation datarefs
    for (dr_code_t drc = DR_ROTATE1; drc < N_JW_DR; drc++) {
        char name[100];
        name[99] = '\0';
        snprintf(name, sizeof(name) - 1, "sam/jetway/%s", dr_name_jw[drc]);
        XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL,
                                 NULL, read_jw_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(uint64_t)drc, NULL);

        for (int i = 1; i <= MAX_SAM3_LIB_JW; i++) {
            snprintf(name, sizeof(name) - 1, "sam/jetway/%02d/%s", i, dr_name_jw[drc]);
            uint64_t ctx = (uint64_t)i << 32|(uint64_t)drc;
            XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL,
                                     NULL, read_jw_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void *)ctx, NULL);
        }

    }

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, jw_status_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, NULL, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/number", xplmType_Int, 0, jw_status_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, &n_active_jw, NULL);

    reset_jetways();

    char fn[sizeof(base_dir) + 100];
    strcpy(fn, base_dir);
    strcat(fn, "sound/alert.wav");
    read_wav(fn, &alert);
    if (alert.data)
        log_msg("alert sound loaded, channels: %d, bit_rate: %d, size: %d",
                alert.num_channels, alert.sample_rate, alert.size);

    return 1;
}