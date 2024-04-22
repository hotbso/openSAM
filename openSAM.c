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
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define XPLM200
#define XPLM300
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"

#include "openSAM.h"

/*
 * On the various coordinate systems and angles:
 *
 * Objects are drawn in a +x east , -z true north, +y up system.
 * Headings (hdgt) are measured from -z (=true north) right turning
 *
 * Imagine looking from below to the sky you have the more traditional 'math' view
 * +x right, +z up, angles left turning from the +x-axis to the +z axis.
 *
 * So if alpha is a 'math' angle we have
 *
 * hdgt = 90° + alpha.
 *
 * If we have a coordinate system that is rotated by an angle of psi we have
 *
 * hdgt\rot = hdgt - psi
 *
 * When it comes to object rotations or longitudes we use relative angles in (-180, +180]
 * This is done by RA().
 *
 * So first of all we rotate and shift everything into the 'door' frame i.e.
 * acf nose pointing up to -z, door at (0,0), jetways to the left somewhere at -x and some z
 *
 * These values are kept in the "active_jw".
 *
 * Then we do our math in the door frame and transform everything back to the jetway frame
 * to get the ratation angles.
 *
 */


static const float JW_DRIVE_SPEED = 1.0;    /* m/s */
static const float JW_TURN_SPEED = 10.0;    /* °/s */
static const float JW_HEIGHT_SPEED = 0.1;   /* m/s */
static const float JW_ANIM_INTERVAL = 0.01;


static char pref_path[512];
static const char *psep;
static XPLMMenuID seasons_menu;
static int auto_item, season_item[4];
static int auto_season;
static int airport_loaded;
static int nh;     // on northern hemisphere
static int season; // 0-3
static const char *dr_name[] = {"sam/season/winter", "sam/season/spring",
            "sam/season/summer", "sam/season/autumn"};


static XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, plane_y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

typedef enum
{
    DISABLED=0, IDLE, PARKED, CAN_DOCK,
    DOCKING, DOCKED, UNDOCKING, CANT_DOCK
} state_t;

const char * const state_str[] = {
    "DISABLED", "IDLE", "PARKED", "CAN_DOCK",
    "DOCKING", "DOCKED", "UNDOCKING", "CANT_DOCK" };

static int on_ground = 1;
static float on_ground_ts;
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

typedef struct door_info_ {
    float x, y, z;
} door_info_t;

static int n_door;
static door_info_t door_info[2];

typedef enum ajw_status_e {
    AJW_PARKED, AJW_DOCKING,
    AJW_DOCKED, AJW_UNDOCKING
} ajw_status_t;

typedef struct active_jw_ {
    sam_jw_t *jw;
    ajw_status_t state;

    /* in door local coordinates */
    float x, y, z, psi;
    float tgt_x, tgt_rot1, tgt_rot2, tgt_rot3, tgt_extent;

    float last_step_ts;
    float timeout;  // so we don't get stuck
} active_jw_t;

static int n_active_jw;
static active_jw_t active_jw[2];

static float lat_ref = -1000, lon_ref = -1000;
/* generation # of reference frame
 * init with 1 so jetways never seen by the accessor won't be considered in find_dockable_jws() */
static unsigned int ref_gen = 1;

static float now;           /* current timestamp */
static int beacon_state, beacon_last_pos;   /* beacon state, last switch_pos, ts of last switch actions */
static float beacon_off_ts, beacon_on_ts;

static int use_engine_running;              /* instead of beacon, e.g. MD11 */
static int dont_connect_jetway;             /* e.g. for ZIBO with own ground service */
static float plane_cg_y, plane_cg_z;


static unsigned long long int stat_far_skip, stat_near_skip, stat_acc_called, stat_jw_match;

static int dock_requested, undock_requested;

#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define BETWEEN(x ,a ,b) ((a) <= (x) && (x) <= (b))

#define SQR(x) \
    ({float x_ = (x); x_ * x_;})

static inline
float RA(float angle)
{
    if (angle > 180.0f)
        return 360.0f - angle;

    if (angle <= -180.0f)
        return angle + 360.0f;

    return angle;
}

static inline float
clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}


static void
save_pref()
{
    FILE *f = fopen(pref_path, "w");
    if (NULL == f)
        return;

    /* encode southern hemisphere with negative season */
    int s = nh ? season : -season;
    fprintf(f, "%d,%d", auto_season, s);
    fclose(f);

    log_msg("Saving pref auto_season: %d, season: %d", auto_season, s);
}

static void
load_pref()
{
    FILE *f  = fopen(pref_path, "r");
    if (NULL == f)
        return;

    nh = 1;
    if (2 == fscanf(f, "%i,%i", &auto_season, &season))
        log_msg("From pref: auto_season: %d, seasons: %d",
                auto_season,  season);
    else {
        auto_season = 0;
        season = 0;
        log_msg("Error reading pref");
    }

    fclose(f);

    if (season < 0) {
        nh = 0;
        season = -season;
    }
}

static int
check_beacon(void)
{
    if (use_engine_running) {
        int er[8];
        int n = XPLMGetDatavi(eng_running_dr, er, 0, 8);
        for (int i = 0; i < n; i++)
            if (er[i])
                return 1;

        return 0;
    }

    /* when checking the beacon guard against power transitions when switching
       to the APU generator (e.g. for the ToLiss fleet).
       Report only state transitions when the new state persisted for 3 seconds */

    int beacon = XPLMGetDatai(beacon_dr);
    if (beacon) {
        if (! beacon_last_pos) {
            beacon_on_ts = now;
            beacon_last_pos = 1;
        } else if (now > beacon_on_ts + 3.0)
            beacon_state = 1;
    } else {
        if (beacon_last_pos) {
            beacon_off_ts = now;
            beacon_last_pos = 0;
        } else if (now > beacon_off_ts + 3.0)
            beacon_state = 0;
   }

   return beacon_state;
}

// Accessor for the "sam/season/*" datarefs
static int
read_season_acc(void *ref)
{
    int s = (long long)ref;
    int val = (s == season);

    //log_msg("accessor %s called, returns %d", dr_name[s], val);
    return val;
}

/*
 * Accessor for the "sam/jetway/..." datarefs
 *
 * This function is called from draw loops, efficient coding required.
 *
 */
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

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            float dlon_m = fabsf(lon - jw->longitude) * LON_D2M;
            float dlat_m = fabsf(lat - jw->latitude) * cosf(D2R * lat) * LON_D2M;

            /* quick check by lat/lon */
            if (dlon_m > FAR_SKIP || dlat_m > FAR_SKIP) {
                stat_far_skip++;
                continue;
            }

            if (jw->xml_ref_gen < ref_gen) {
                XPLMWorldToLocal(jw->latitude, jw->longitude, elevation, &jw->xml_x, &jw->xml_y, &jw->xml_z);
                jw->xml_ref_gen = ref_gen;
            }

            if (fabsf(obj_x - jw->xml_x) > NEAR_SKIP || fabsf(obj_z - jw->xml_z) > NEAR_SKIP) {
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

    return 0.0;
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

/* convert wheel at (x, z) to dataref values, rot2, rot3 can be NULL */
static inline void
jw_xy_to_sam_dr(const active_jw_t *ajw, float x, float z, float *rot1, float *extent, float *rot2, float *rot3)
{
    const sam_jw_t *jw = ajw->jw;

    float dist = sqrtf(SQR(x - ajw->x) + SQR(z - ajw->z));

    float rot1_d = 90.0f + asinf((z - ajw->z)/ dist) / D2R;   // door frame
    *rot1 =  RA(rot1_d - ajw->psi);
    *extent = dist - jw->cabinPos;

    /* angle 0° door frame  -> hdgt -> jw frame -> diff to rot1 */
    float r2 = RA(0.0f + 90.0f - ajw->psi - *rot1);
    if (rot2)
        *rot2 = r2;

    if (rot3) {
        float net_length = dist + jw->cabinLength * cosf(r2 * D2R);
        *rot3 = RA(-asinf(ajw->y / net_length) / D2R);
    }
}


/* try to find dockable jetways and save their info */
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

    XPLMProbeRef probe_ref = XPLMCreateProbe(xplm_ProbeY);
    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo)) {
        log_msg("XPLMProbeTerrainXYZ failed");
        goto out;
    }

    float door_agl = plane_y - probeinfo.locationY + door_info[0].y;
    log_msg("plane: x: %5.3f, z: %5.3f, y: %5.3f, door_agl: %.2f, psi: %4.1f",
            plane_x, plane_z, plane_y, door_agl, plane_psi);

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++)
        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            if (jw->obj_ref_gen < ref_gen)  /* not visible -> not dockable */
                continue;

            if (jw->door != 0)
                continue;

            log_msg("%s, global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f",
                    jw->name, jw->x, jw->z, jw->y, jw->psi);

            active_jw_t *ajw = &active_jw[0];
            memset(ajw, 0, sizeof(active_jw_t));
            ajw->jw = jw;

            /* rotate into plane local frame */
            float dx = jw->x - plane_x;
            float dz = jw->z - plane_z;
            ajw->x =  cos_psi * dx + sin_psi * dz;
            ajw->z = -sin_psi * dx + cos_psi * dz;
            ajw->psi = RA(jw->psi - plane_psi);

            /* move into door local frame */
            ajw->x -= door_info[0].x;
            ajw->z -= door_info[0].z;

            if (ajw->x > 0.0)   // on the right side
                continue;

            ajw->tgt_x = -jw->cabinLength;
            // tgt z = 0.0
            ajw->y = jw->height - door_agl;

            jw_xy_to_sam_dr(ajw, ajw->tgt_x, 0.0f, &ajw->tgt_rot1, &ajw->tgt_extent, &ajw->tgt_rot2, &ajw->tgt_rot3);
            ajw->tgt_rot2 += 3.0f;  /* for door1 only */
            jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);

            log_msg("%s, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, extent: %.1f", jw->name,
                    ajw->x, ajw->z, ajw->y, ajw->psi, ajw->tgt_extent);

            if (ajw->tgt_extent > jw->maxExtent || ajw->tgt_extent < 0.5) {
                log_msg("dist %0.2f too far or invalid", ajw->tgt_extent);
                continue;
            }

            log_msg("match: %s, rot1: %0.1f, rot2: %0.1f, rot3: %0.1f, extent: %0.1f",
                    jw->name, ajw->tgt_rot1, ajw->tgt_rot2, ajw->tgt_rot3, ajw->tgt_extent);

            if (BETWEEN(ajw->tgt_rot1, jw->minRot1, jw->maxRot1) && BETWEEN(ajw->tgt_rot2, jw->minRot2, jw->maxRot2)
                && BETWEEN(ajw->tgt_extent, jw->minExtent, jw->maxExtent)) {
                n_active_jw = 1;
                break;
            } else {
                log_msg("jw: %s does not match min, max criteria", jw->name);
            }
        }

out:
    XPLMDestroyProbe(probe_ref);
    return n_active_jw;
}

static void
animate_wheels(sam_jw_t *jw, float ds, float d_rot)
{
    float da_ds = (ds / jw->wheelDiameter) / D2R;
    float da_rot = d_rot * (jw->wheelDistance / jw->wheelDiameter);

    jw->wheelrotatel = jw->wheelrotatel + da_ds + da_rot;
    if (jw->wheelrotatel > 360.0f)
        jw->wheelrotatel -= 360.0f;
    else if (jw->wheelrotatel < 0.0f)
        jw->wheelrotatel += 360.0f;

    jw->wheelrotater = jw->wheelrotater + da_ds - da_rot;
    if (jw->wheelrotater > 360.0f)
        jw->wheelrotater -= 360.0f;
    else if (jw->wheelrotater < 0.0f)
        jw->wheelrotater += 360.0f;
}

static int
rotate_wheel_base(sam_jw_t *jw, float wb_rot, float dt, float ds)
{
    //log_msg("drive angle: %0.2f, wb_rot: %0.2f", drive_angle, wb_rot);
    wb_rot = clampf(wb_rot, -90.0f, 90.0f);
    // TODO: convert drive_angle back after clamping ?

    /* wheel rotation */
    if (fabs(jw->wheelrotatec - wb_rot) > 2.0f) {
        float d_rot = dt * JW_TURN_SPEED;
        //log_msg("turning wheel base by %0.2f°", d_rot);
        if (wb_rot > jw->wheelrotatec)
            jw->wheelrotatec += d_rot;
        else
            jw->wheelrotatec -= d_rot;

        animate_wheels(jw, 0.0f, d_rot);
        return 1;   /* must wait */
    }

    float d_rot = wb_rot - jw->wheelrotatec;
    jw->wheelrotatec = wb_rot;
    animate_wheels(jw, ds, d_rot);
    return 0;
}

static float
dock_drive()
{
    if (n_active_jw == 0)
        return 0.5;

    active_jw_t *ajw = &active_jw[0];
    sam_jw_t *jw = ajw->jw;
    //log_msg("dock_drive(): state: %d", ajw->state);
    if (ajw->state != AJW_DOCKING)
        return 0.5;

    if (now > ajw->timeout) {
        log_msg("dock_drive() timeout!");
        ajw->state = AJW_DOCKED;
        jw->rotate1 = ajw->tgt_rot1;
        jw->rotate2 = ajw->tgt_rot2;
        jw->rotate3 = ajw->tgt_rot3;
        jw->extent = ajw->tgt_extent;
        return 0;   // -> done
    }

    float dt = now - ajw->last_step_ts;
    float remaining = JW_ANIM_INTERVAL - dt;
    if (remaining > 0)
        return remaining;

    ajw->last_step_ts = now;

    float tgt_x = ajw->tgt_x;

    float rot1_d = RA((jw->rotate1 + ajw->psi) - 90.0f);    // door frame
    float r = jw->extent + jw->cabinPos;

    float wheel_x = ajw->x + r * cosf(rot1_d * D2R);
    float wheel_z = ajw->z + r * sinf(rot1_d * D2R);

    /* ramp down speed when approaching the plane */
    float drive_speed = JW_DRIVE_SPEED;
    if (wheel_x >= (tgt_x - 0.8f))
        drive_speed = JW_DRIVE_SPEED * (0.1f + 0.9f * MAX(0.0f, (tgt_x - wheel_x) / 0.8f));

    float eps = MAX(2.0f * dt * drive_speed, 0.05f);
    if (fabsf(tgt_x - wheel_x) < eps && fabsf(wheel_z) < eps)  {
        ajw->state = AJW_DOCKED;
        log_msg("target_pos reached");
        return 0;   // -> done
    }

    wheel_x = MIN(wheel_x, tgt_x); // dont drive beyond the target point

    /* pick intermediate target halfway between wheel_x and door
     * to get a better straighten out */
    if (wheel_x < (tgt_x - 1.0f))
        tgt_x = 0.5 * (wheel_x + (tgt_x - 1.0f));

    float dir_x = tgt_x - wheel_x;
    float dir_z = - wheel_z;
    float ds = sqrtf(SQR(dir_x) + SQR(dir_z));
    ds = MAX(ds, 0.0001f);

    dir_x /= ds;
    dir_z /= ds;

    //log_msg("anim_step: rot1_d: %.2f, wheel_x: %0.2f, wheel_z: %.2f, dir_x: %.2f, dir_z: %.2f",
    //        rot1_d, wheel_x, wheel_z, dir_x, dir_z);

    float dx = dir_x * dt * drive_speed;
    float dz = dir_z * dt * drive_speed;
    wheel_x += dx;
    wheel_z += dz;

    float drive_angle = fabsf(wheel_z) <= 0.1 ? 0.0f : atan2(dz, dx) / D2R;

    if (rotate_wheel_base(jw, RA(drive_angle - rot1_d), dt, ds))
        return JW_ANIM_INTERVAL;

    /* rotation2 */
    float tgt_rot2 = ajw->tgt_rot2;
    if (wheel_x < (tgt_x - 1.0f)) {
        float angle_to_door = atan2f(-wheel_z, ajw->tgt_x - wheel_x) / D2R;
        tgt_rot2 = angle_to_door + 90.0f - ajw->psi - jw->rotate1; /* point to door */
    }
    //log_msg("jw->rotate2: %0.1f, ajw->tgt_rot2: %0.1f, tgt_rot2: %0.1f", jw->rotate2, ajw->tgt_rot2, tgt_rot2);

    if (fabsf(jw->rotate2 - tgt_rot2) > 0.5) {
        float d_rot2 = dt * JW_TURN_SPEED;
        if (jw->rotate2 >= tgt_rot2)
            jw->rotate2 = MAX(jw->rotate2 - d_rot2, tgt_rot2);
        else
             jw->rotate2 = MIN(jw->rotate2 + d_rot2, tgt_rot2);

        if (wheel_x >= (tgt_x - 1.0f))
            return JW_ANIM_INTERVAL;
    }

    /* rotation1 + rotation3 */
    jw_xy_to_sam_dr(ajw, wheel_x, wheel_z, &jw->rotate1, &jw->extent, NULL, NULL);

    if (fabsf(jw->rotate3 - ajw->tgt_rot3) > 0.5) {
        float d_rot3 = (dt * JW_HEIGHT_SPEED / (jw->wheelPos + jw->extent)) / D2R;  /* strictly it's atan */
        if (jw->rotate3 >= ajw->tgt_rot3)
            jw->rotate3 = MAX(jw->rotate3 - d_rot3, ajw->tgt_rot3);
        else
             jw->rotate3 = MIN(jw->rotate3 + d_rot3, ajw->tgt_rot3);
    }
    jw->wheels = tanf(jw->rotate3 * D2R) * (jw->wheelPos + jw->extent);

    if (JW_ANIM_INTERVAL < 0.02)
        return -1;
    return JW_ANIM_INTERVAL;
}


static float
undock_drive()
{
    if (n_active_jw == 0)
        return 0.5;

    log_msg("undock_drive()");

    active_jw_t *ajw = &active_jw[0];
    sam_jw_t *jw = ajw->jw;

    ajw->state = PARKED;
    jw->rotate1 = jw->initialRot1;
    jw->rotate2 = jw->initialRot2;
    jw->rotate3 = jw->initialRot3;
    jw->extent = jw->initialExtent;
    return 0.0f;
}

static int
cmd_dock_jw_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    if ((void *)1 == ref) {
        dock_requested = 1;
    } else {
        undock_requested = 1;
    }

    return 0;
}

static float parked_x, parked_y;
static int parked_ngen;

static int
check_teleportation()
{
    float x = XPLMGetDataf(plane_x_dr);
    float y = XPLMGetDataf(plane_y_dr);
    int ngen = ref_gen;

    if (parked_ngen != ngen || fabsf(parked_x - x) > 1.0f || fabsf(parked_y - y) > 1.0f) {
        log_msg("parked_ngen: %d, ngen: %d, parked_x: %0.3f, x: %0.3f, parked_y: %0.3f, y: %0.3f",
                parked_ngen, ngen, parked_x, x, parked_y, y);
        return 1;
    }

    return 0;
}

/* the state machine triggered by the flight loop */
static float
run_state_machine()
{
    float remaining;

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
            if (find_dockable_jws())
                new_state = CAN_DOCK;
            else
                new_state = CANT_DOCK;
            break;

        case CAN_DOCK:
            if (dock_requested) {
                log_msg("docking requested");
                active_jw_t *ajw = &active_jw[0];
                ajw->state = AJW_DOCKING;
                ajw->last_step_ts = now;
                ajw->timeout = now + 30.0f;
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
            remaining = dock_drive();
            if (remaining == 0.0f)
                new_state = DOCKED;
            else
                return remaining;

        case DOCKED:
            if (!on_ground || beacon_on) {
                new_state = IDLE;
                break;
            }

            if (undock_requested) {
                log_msg("undocking requested");
                active_jw_t *ajw = &active_jw[0];
                ajw->state = AJW_UNDOCKING;
                ajw->last_step_ts = now;
                ajw->timeout = XPLMGetDataf(total_running_time_sec_dr) + 30.0f;
                new_state = UNDOCKING;
            }
            break;

        case UNDOCKING:
            remaining = undock_drive();
            if (remaining == 0.0f)
                new_state = IDLE;
            else
                return remaining;
            break;

        default:
            log_msg("Bad state %d", state);
            new_state = DISABLED;
            break;
    }

    dock_requested = undock_requested = 0;

    if (new_state != state) {
        log_msg("state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        return -1;  // see you on next frame
    }

    return 0.5;
}

static float
flight_loop_cb(float inElapsedSinceLastCall,
               float inElapsedTimeSinceLastFlightLoop, int inCounter,
               void *inRefcon)
{
    float loop_delay = 2.0;

    now = XPLMGetDataf(total_running_time_sec_dr);
    int og = (XPLMGetDataf(gear_fnrml_dr) != 0.0);

    if (og != on_ground && now > on_ground_ts + 10.0) {
        on_ground = og;
        on_ground_ts = now;
        log_msg("transition to on_ground: %d", on_ground);
    }

    loop_delay = run_state_machine();
    return loop_delay;
}

// set season according to date
static void
set_season_auto()
{
    if (! auto_season)
        return;

    int day = XPLMGetDatai(date_day_dr);
    if (nh) {
        if (day <= 80) {
            season = 0;
        } else if (day <= 172) {
            season = 1;
        } else if (day <= 264) {
            season = 2;
        } else if (day <= 355) {
            season = 3;
        } else if (day) {
            season = 0;
        }
    } else {
        if (day <= 80) {
            season = 2;
        } else if (day <= 172) {
            season = 3;
        } else if (day <= 264) {
            season = 0;
        } else if (day <= 355) {
            season = 1;
        } else if (day) {
            season = 2;
        }
    }

    log_msg("nh: %d, day: %d, season: %d", nh, day, season);
}

// emuluate a kind of radio buttons
static void
set_menu()
{
    XPLMCheckMenuItem(seasons_menu, auto_item,
                      auto_season ? xplm_Menu_Checked : xplm_Menu_Unchecked);

    XPLMCheckMenuItem(seasons_menu, season_item[season], xplm_Menu_Checked);
    for (int i = 0; i < 4; i++)
        if (i != season)
            XPLMCheckMenuItem(seasons_menu, season_item[i], xplm_Menu_Unchecked);
}

static void
menu_cb(void *menu_ref, void *item_ref)
{
    int entry = (long long)item_ref;

    if (entry == 4) {
        auto_season = !auto_season;
        set_season_auto();
    } else {
        int checked;
        XPLMCheckMenuItemState(seasons_menu, season_item[entry], &checked);
        log_msg("menu_cb: entry %d, checked: %d", entry, checked);

        if (checked == 1 && entry != season) { // checking a prior unchecked entry
            season = entry;
        } /* else nothing, unchecking is not possible */

        auto_season = 0;    // selecting a season always goes to manual mode
   }

    set_menu();
    save_pref();
}

static int
find_icao_in_file(const char *acf_icao, const char *dir, const char *fn)
{
    char fn_full[512];
    snprintf(fn_full, sizeof(fn_full) - 1, "%s%s", dir, fn);

    int res = 0;
    FILE *f = fopen(fn_full, "r");
    if (f) {
        log_msg("check whether acf '%s' is in exception file %s", acf_icao, fn_full);
        char line[100];
        while (fgets(line, sizeof(line), f)) {
            char *cptr = strchr(line, '\r');
            if (cptr)
                *cptr = '\0';
            cptr = strchr(line, '\n');
            if (cptr)
                *cptr = '\0';

            if (0 == strcmp(line, acf_icao)) {
                log_msg("found acf %s in %s", acf_icao, fn);
                res = 1;
                break;
            }
        }

        fclose(f);
    }

    return res;
}

/* =========================== plugin entry points ===============================================*/
PLUGIN_API int
XPluginStart(char *out_name, char *out_sig, char *out_desc)
{
    log_msg("Startup " VERSION);

    strcpy(out_name, "openSAM " VERSION);
    strcpy(out_sig, "openSAM.hotbso");
    strcpy(out_desc, "A plugin that emulates SAM");

    /* Always use Unix-native paths on the Mac! */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    char xp_dir[512];
	XPLMGetSystemPath(xp_dir);
    psep = XPLMGetDirectorySeparator();

    /* set pref path */
    XPLMGetPrefsPath(pref_path);
    XPLMExtractFileAndPath(pref_path);
    strcat(pref_path, psep);
    strcat(pref_path, "sam_se.prf");

    date_day_dr = XPLMFindDataRef("sim/time/local_date_days");

    lat_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lat_ref");
    lon_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lon_ref");

    plane_x_dr = XPLMFindDataRef("sim/flightmodel/position/local_x");
    plane_y_dr = XPLMFindDataRef("sim/flightmodel/position/local_y");
    plane_z_dr = XPLMFindDataRef("sim/flightmodel/position/local_z");
    plane_lat_dr = XPLMFindDataRef("sim/flightmodel/position/latitude");
    plane_lon_dr = XPLMFindDataRef("sim/flightmodel/position/longitude");
    plane_elevation_dr= XPLMFindDataRef("sim/flightmodel/position/elevation");
    plane_true_psi_dr = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    plane_y_agl_dr = XPLMFindDataRef("sim/flightmodel2/position/y_agl");

    draw_object_x_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_x");
    draw_object_y_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_y");
    draw_object_z_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_z");
    draw_object_psi_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_psi");

    parkbrake_dr = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
    beacon_dr = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    eng_running_dr = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
    gear_fnrml_dr = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");

    acf_icao_dr = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    acf_cg_y_dr = XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    acf_cg_z_dr = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    acf_door_x_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    acf_door_y_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    acf_door_z_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_z");

    total_running_time_sec_dr = XPLMFindDataRef("sim/time/total_running_time_sec");
    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");

    /* create the seasons datarefs */
    for (int i = 0; i < 4; i++)
        XPLMRegisterDataAccessor(dr_name[i], xplmType_Int, 0, read_season_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(long long)i, NULL);

    /* own commands */
    XPLMCommandRef dock_cmdr = XPLMCreateCommand("openSAM/dock_jwy", "Dock jetway");
    XPLMRegisterCommandHandler(dock_cmdr, cmd_dock_jw_cb, 0, (void *)1);

    XPLMCommandRef undock_cmdr = XPLMCreateCommand("openSAM/undock_jwy", "Undock jetway");
    XPLMRegisterCommandHandler(undock_cmdr, cmd_dock_jw_cb, 0, (void *)0);

    /* build menues */
    XPLMMenuID menu = XPLMFindPluginsMenu();
    XPLMMenuID os_menu = XPLMCreateMenu("openSAM", menu,
                                        XPLMAppendMenuItem(menu, "openSAM", NULL, 0),
                                        NULL, NULL);
    /* openSAM */
    XPLMAppendMenuItemWithCommand(os_menu, "Dock Jetway", dock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Undock Jetway", undock_cmdr);

    XPLMAppendMenuSeparator(os_menu);

    /* openSAM -> Seasons */
    int seasons_menu_item = XPLMAppendMenuItem(os_menu, "Seasons", NULL, 0);
    seasons_menu = XPLMCreateMenu("Seasons", os_menu, seasons_menu_item, menu_cb, NULL);

    auto_item = XPLMAppendMenuItem(seasons_menu, "Automatic", (void *)4, 0);

    XPLMAppendMenuSeparator(seasons_menu);

    season_item[0] = XPLMAppendMenuItem(seasons_menu, "Winter", (void *)0, 0);
    season_item[1] = XPLMAppendMenuItem(seasons_menu, "Spring", (void *)1, 0);
    season_item[2] = XPLMAppendMenuItem(seasons_menu, "Summer", (void *)2, 0);
    season_item[3] = XPLMAppendMenuItem(seasons_menu, "Autumn", (void *)3, 0);
    /* --------------------- */

    load_pref();
    set_menu();

    if (!collect_sam_xml(xp_dir))
        log_msg("Error collecting sam.xml files!");

    log_msg("%d sceneries with sam jetways found", n_sceneries);
    reset_jetways();

    /* create the jetway datarefs */
    for (dr_code_t drc = DR_ROTATE1; drc < N_JW_DR; drc++)
        XPLMRegisterDataAccessor(dr_name_jw[drc], xplmType_Float, 0, NULL,
                                 NULL, read_jw_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(long long)drc, NULL);

    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;
}


PLUGIN_API void
XPluginStop(void)
{
}


PLUGIN_API void
XPluginDisable(void)
{
    save_pref();
    log_msg("acc called:  %llu", stat_acc_called);
    log_msg("far skip:    %llu", stat_far_skip);
    log_msg("near skip:   %llu", stat_near_skip);
}


PLUGIN_API int
XPluginEnable(void)
{
    return 1;
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID in_from, long in_msg, void *in_param)
{
    /* Everything before XPLM_MSG_AIRPORT_LOADED has bogus datarefs.
       Anyway it's too late for the current scenery. */
    if ((in_msg == XPLM_MSG_AIRPORT_LOADED) ||
        (airport_loaded && (in_msg == XPLM_MSG_SCENERY_LOADED))) {
        airport_loaded = 1;
        nh = (XPLMGetDatad(plane_lat_dr) >= 0.0);
        set_season_auto();
    }

    /* my plane loaded */
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        char acf_icao[41];
        memset(acf_icao, 0, sizeof(acf_icao));
        XPLMGetDatab(acf_icao_dr, acf_icao, 0, 40);
        acf_icao[4] = '\0';

        plane_cg_y = F2M * XPLMGetDataf(acf_cg_y_dr);
        plane_cg_z = F2M * XPLMGetDataf(acf_cg_z_dr);

        door_info[0].x = XPLMGetDataf(acf_door_x_dr);
        door_info[0].y = XPLMGetDataf(acf_door_y_dr);
        door_info[0].z = XPLMGetDataf(acf_door_z_dr);
        n_door = 1;

        /* check whether acf is listed in exception files */
        use_engine_running = 0;
        dont_connect_jetway = 0;

        char dir[512];
        dir[0] = '\0';
        XPLMGetPluginInfo(XPLMGetMyID(), NULL, dir, NULL, NULL);
        char *cptr = strrchr(dir, '/');    /* basename */
        if (cptr) {
            *cptr = '\0';
            cptr = strrchr(dir, '/');       /* one level up */
        }

        if (cptr)
            *(cptr + 1) = '\0';             /* keep the / */

        if (find_icao_in_file(acf_icao, dir, "acf_use_engine_running.txt"))
            use_engine_running = 1;

        if (find_icao_in_file(acf_icao, dir, "acf_dont_connect_jetway.txt"))
            dont_connect_jetway = 1;


        log_msg("plane loaded: %s, plane_cg_y: %1.2f, plane_cg_z: %1.2f, "
               "plane_door_x: %1.2f, plane_door_y: %1.2f, plane_door_z: %1.2f",
               acf_icao, plane_cg_y, plane_cg_z,
               door_info[0].x, door_info[0].y, door_info[0].z);
    }
}
