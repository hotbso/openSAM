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

#include <stddef.h>
#include <string.h>
#include <math.h>

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
    DISABLED=0, IDLE, PARKED, CAN_DOCK,
    DOCKING, DOCKED, UNDOCKING, CANT_DOCK
} state_t;

static const char * const state_str[] = {
    "DISABLED", "IDLE", "PARKED", "CAN_DOCK",
    "DOCKING", "DOCKED", "UNDOCKING", "CANT_DOCK" };

static state_t state = IDLE;

// keep in sync!
typedef enum dr_code_e {
    DR_ROTATE1, DR_ROTATE2, DR_ROTATE3, DR_EXTENT,
    DR_WHEELS, DR_WHEELROTATEC, DR_WHEELROTATER, DR_WHEELROTATEL,
    N_JW_DR
} dr_code_t;

static const char *dr_name_jw[] = {
    "sam/jetway/rotate1",
    "sam/jetway/rotate2",
    "sam/jetway/rotate3",
    "sam/jetway/extent",
    "sam/jetway/wheels",
    "sam/jetway/wheelrotatec",
    "sam/jetway/wheelrotater",
    "sam/jetway/wheelrotatel"};

typedef enum ajw_status_e {
    AJW_PARKED,
    AJW_TO_AP, AJW_AT_AP, AJW_TO_DOOR, AJW_DOCKED,  // sequence for docking

    // AJW_TO_AP,                                   // sequence for undocking
    AJW_TO_PARK
} ajw_status_t;

typedef struct active_jw_ {
    sam_jw_t *jw;
    ajw_status_t state;

    // in door local coordinates
    float x, y, z, psi;
    float dist;         // distance to door

    // target cabin position with corresponding dref values
    float tgt_x, tgt_rot1, tgt_rot2, tgt_rot3, tgt_extent;
    float ap_x;      // alignment point abeam door
    float parked_x, parked_z;

    double cabin_x, cabin_z;

    int wait_wb_rot;    // waiting for wheel base rotation
    float wb_rot;       // to this angle

    float last_step_ts;
    float timeout;      // so we don't get stuck
} active_jw_t;

static int n_active_jw;
static active_jw_t active_jw[MAX_DOOR];

//
// Accessor for the "sam/jetway/..." datarefs
//
// This function is called from draw loops, efficient coding required.
//
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
            dr_code_t drc = (long long)ref;
            switch (drc) {
                case DR_ROTATE1:
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
    UNUSED(ref);
    if (0 == n_active_jw)
        return 0;

    if (CAN_DOCK == state)
        return 1;

    if (DOCKED == state)
        return 2;

    return -1;
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
            //log_msg("%s %5.6f %5.6f", jw->name, jw->latitude, jw->longitude);
       }

   n_active_jw = 0;
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


// try to find dockable jetways and save their info
static int
find_dockable_jws()
{
    if (n_door == 0) {
        log_msg("acf has no doors!");
        return 0;
    }

    n_active_jw = 0;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_y = XPLMGetDataf(plane_y_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);
    float plane_psi = XPLMGetDataf(plane_true_psi_dr);

    float sin_psi = sinf(D2R * plane_psi);
    float cos_psi = cosf(D2R * plane_psi);

    memset(active_jw, 0, sizeof(active_jw));
    for (int i = 0; i < MAX_DOOR; i++)
        active_jw[i].dist = 1.0E10;

    // Unfortunately maxExtent in sam.xml can be bogus (e.g. FlyTampa EKCH)
    // So we find the nearest jetways on the left and do some heuristics
    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            if (jw->obj_ref_gen < ref_gen)  // not visible -> not dockable
                continue;

            if (jw->door >= n_door)
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

            // move into door local frame
            ajw->x -= door_info[jw->door].x;
            ajw->z -= door_info[jw->door].z;

            if (ajw->x > 0.0 || BETWEEN(ajw->psi, -170.0f, 20.0f))   // on the right side or pointing away
                continue;

            ajw->dist = len2f(ajw->x, ajw->z);
            if (ajw->dist > active_jw[jw->door].dist)
                continue;

            ajw->tgt_x = -jw->cabinLength;
            // tgt z = 0.0
            ajw->y = (jw->y + jw->height) - (plane_y + door_info[jw->door].y);

            jw_xy_to_sam_dr(ajw, ajw->tgt_x, 0.0f, &ajw->tgt_rot1, &ajw->tgt_extent, &ajw->tgt_rot2, &ajw->tgt_rot3);
            if (jw->door == 0)
                ajw->tgt_rot2 += 3.0f;  // for door1 only

            log_msg("candidate %s, door %d, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, extent: %.1f",
                    jw->name, jw->door,
                    ajw->x, ajw->z, ajw->y, ajw->psi, ajw->tgt_extent);
            active_jw[jw->door] = tentative_ajw;
        }

    // perform sanity check of jetways found and compute final values
    for (int i = 0; i < n_door; i++) {
        active_jw_t *ajw = &active_jw[i];
        sam_jw_t *jw = ajw->jw;
        if (jw == NULL) // empty slot
            continue;

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

        // compute x,z of parked jw
        float rot1_d = RA((jw->initialRot1 + ajw->psi) - 90.0f);    // door frame
        float r = jw->initialExtent + jw->cabinPos;
        ajw->parked_x = ajw->x + r * cosf(rot1_d * D2R);
        ajw->parked_z = ajw->z + r * sinf(rot1_d * D2R);
        ajw->cabin_x = ajw->x + (jw->extent + jw->cabinPos) * cosf(rot1_d * D2R);
        ajw->cabin_z = ajw->z + (jw->extent + jw->cabinPos) * sinf(rot1_d * D2R);

        ajw->ap_x = ajw->tgt_x - JW_ALIGN_DIST;

        jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);

        log_msg("active jetway for door %d: %s", jw->door, jw->name);
        n_active_jw = jw->door + 1;
    }

    return n_active_jw;
}

static int
rotate_wheel_base(sam_jw_t *jw, float wb_rot, float dt)
{
    //log_msg("wb_rot: %0.2f", wb_rot);

    // driving backwards?
    if (fabs(wb_rot) > 90.0f) {
        wb_rot = RA(180.0f + wb_rot);
    }

    //wb_rot = clampf(wb_rot, -90.0f, 90.0f);
    // TODO: convert drive_angle back after clamping ?

    // wheel base rotation
    int result = 0;
    float d_rot;
    if (fabsf(jw->wheelrotatec - wb_rot) > 2.0f) {
        d_rot = dt * JW_TURN_SPEED;
        //log_msg("turning wheel base by %0.2f°", d_rot);
        if (wb_rot < jw->wheelrotatec)
            d_rot = -d_rot;

        jw->wheelrotatec += d_rot;

        result = 1;    // must wait
    } else {
        d_rot = wb_rot - jw->wheelrotatec;
        jw->wheelrotatec = wb_rot;
    }

    float da_rot = d_rot * (jw->wheelDistance / jw->wheelDiameter);

    jw->wheelrotatel += da_rot;
    jw->wheelrotater -= da_rot;
    return result;
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

    float da_ds = (ds / jw->wheelDiameter) / D2R;
    if (fabsf(jw->wheelrotatec) > 90.0f)
        da_ds = -da_ds;

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
            if (rotate_wheel_base(jw, ajw->wb_rot, dt))
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
        //log_msg("to ap: rot1_d: %.2f, ajw->cabin_x: %0.3f, ajw->cabin_z: %0.3f, drive_angle: %0.2f",
        //        rot1_d, ajw->cabin_x, ajw->cabin_z, drive_angle);

        ajw->wb_rot = RA(drive_angle - rot1_d);
        if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
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
        float drive_angle = 0.0;
        rotate_wheel_base(jw, RA(drive_angle - rot1_d), dt);

        // rotation 2 + 3 must be at target now
        if (rotate_2(ajw, ajw->tgt_rot2, dt) && rotate_3(ajw, ajw->tgt_rot3, dt))
            ajw->state = AJW_TO_DOOR;
    }

    if (ajw->state == AJW_TO_DOOR) {
        if (ajw->wait_wb_rot) {
            // log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(jw, ajw->wb_rot, dt))
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
        if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
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
            if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
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
        if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        animate_wheels(ajw, -ds);
    }

    if (ajw->state == AJW_AT_AP) {
        // nothing for now
        ajw->state = AJW_TO_PARK;
    }

    if (ajw->state == AJW_TO_PARK) {
        if (ajw->wait_wb_rot) {
            // log_msg("AJW_TO_AP: waiting for wb rotation");
            if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
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
        if (rotate_wheel_base(jw, ajw->wb_rot, dt)) {
            ajw->wait_wb_rot = 1;
            return 0;
        }
        ajw->wait_wb_rot = 0;

        rotate_2(ajw, jw->initialRot2, dt);
        rotate_3(ajw, jw->initialRot3, dt);
        rotate_1_extend(ajw, ajw->cabin_x, ajw->cabin_z);
        animate_wheels(ajw, -ds);

        float eps = MAX(2.0f * dt * JW_DRIVE_SPEED, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - ajw->cabin_x), fabs(tgt_z - ajw->cabin_z));
        if (fabs(tgt_x - ajw->cabin_x) < eps && fabs(tgt_z -ajw->cabin_z) < eps)  {
            ajw->state = AJW_PARKED;
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
    if (beacon_on)
        return 0.5;

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

            if (find_dockable_jws())
                new_state = CAN_DOCK;
            else
                new_state = CANT_DOCK;
            break;

        case CAN_DOCK:
            if (dock_requested || toggle_requested) {
                log_msg("docking requested");
                for (int i = 0; i < n_active_jw; i++) {
                    active_jw_t *ajw = &active_jw[i];
                    ajw->state = AJW_TO_AP;
                    ajw->last_step_ts = now;
                    ajw->timeout = now + JW_ANIM_TIMEOUT;
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
            for (int i = 0; i < n_active_jw; i++)
                n_done += dock_drive(&active_jw[i]);

            if (n_done == n_active_jw)
               new_state = DOCKED;
            else
                return JW_ANIM_INTERVAL;
            break;

        case DOCKED:
            if (!on_ground || beacon_on) {
                new_state = IDLE;
                break;
            }

            if (undock_requested || toggle_requested) {
                log_msg("undocking requested");
                for (int i = 0; i < n_active_jw; i++) {
                    active_jw_t *ajw = &active_jw[i];
                    ajw->state = AJW_TO_AP;
                    ajw->last_step_ts = now;
                    ajw->timeout = now + JW_ANIM_TIMEOUT;
                    new_state = UNDOCKING;
                }
            }
            break;

        case UNDOCKING:
            n_done = 0;
            for (int i = 0; i < n_active_jw; i++)
                n_done += undock_drive(&active_jw[i]);

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

    if (new_state != state) {
        log_msg("jw state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        return -1;  // see you on next frame
    }

    return 0.5;
}

int
jw_init()
{
    // create the jetway animation datarefs
    for (dr_code_t drc = DR_ROTATE1; drc < N_JW_DR; drc++)
        XPLMRegisterDataAccessor(dr_name_jw[drc], xplmType_Float, 0, NULL,
                                 NULL, read_jw_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(long long)drc, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, jw_status_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, NULL, NULL);
    reset_jetways();
    return 1;
}