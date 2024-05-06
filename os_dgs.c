/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch
              (C) Jonathan Harris 2006-2013

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
#include <ctype.h>
#include <sys/types.h>

#include "openSAM.h"
#include "os_dgs.h"

/* DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand) */
static const float CAP_A = 15;  /* Capture */
static const float CAP_Z = 100;	/* (50-80 in Safedock2 flier) */

static const float AZI_A = 15;	    /* provide azimuth guidance */
static const float AZI_DISP_A = 10; /* max value for display */
static const float AZI_Z = 90;

static const float GOOD_Z= 0.5;     /* stop position for nw */
static const float GOOD_X = 2.0;    /* for mw */

static const float REM_Z = 12;	    /* Distance remaining from here on*/

static const float dgs_dist = 20.0f;    // distance from dgs to stand for azimuth computation

/* types */
typedef enum
{
    DISABLED=0, INACTIVE, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, DONE
} state_t;

const char * const state_str[] = {
    "DISABLED", "INACTIVE", "ACTIVE", "ENGAGED",
    "TRACK", "GOOD", "BAD", "PARKED", "DONE" };
static state_t state = DISABLED;
static float timestamp;

// Datarefs
static XPLMDataRef percent_lights_dr, sin_wave_dr;

// Published DataRef values
static int status, track, lr;
static float azimuth, distance;

float plane_nw_z, plane_mw_z, plane_cg_z;   // z value of plane's 0 to fw, mw and cg

static ramp_t *nearest_ramp;
static float nearest_ramp_ts;    // timestamp of last find_nearest_ramp()
static int is_marshaller;

static int update_ramp_log_ts;   // throttling of logging
static float sin_wave_prev;

enum _DGS_DREF {
    DGS_DR_UNHIDE,
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,
    DGS_DR_BRIGHTNESS,
    DGS_DR_NUM             // # of drefs
};

// keep exactly the same order as list above
static const char *dgs_dlist_dr[] = {
    "opensam/dgs/unhide",
    "opensam/dgs/status",
    "opensam/dgs/lr",
    "opensam/dgs/track",
    "opensam/dgs/azimuth",
    "opensam/dgs/distance",
    "opensam/dgs/icao_0",
    "opensam/dgs/icao_1",
    "opensam/dgs/icao_2",
    "opensam/dgs/icao_3",
    "opensam/dgs/vdgs_brightness",
    NULL
};

static float drefs[DGS_DR_NUM];

static void
reset_state(state_t new_state)
{
    if (state != new_state)
        log_msg("setting state to %s", state_str[new_state]);

    state = new_state;
    nearest_ramp = NULL;
}

/* set mode to arrival */
void
dgs_set_active(void)
{
    if (! on_ground) {
        log_msg("can't set active when not on ground");
        return;
    }

    // can be teleportation
    reset_state(INACTIVE);

    beacon_state = beacon_last_pos = XPLMGetDatai(beacon_dr);
    beacon_on_ts = beacon_off_ts = -10.0;

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);
    char airport_id[50];

    // find and load airport I'm on now
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, airport_id,
                NULL, NULL);
        log_msg("now on airport: %s", airport_id);
    }

    state = ACTIVE;
}

static void
xform_to_ref_frame(ramp_t *ramp)
{
    if (ramp->ref_gen < ref_gen) {
        XPLMWorldToLocal(ramp->lat, ramp->lon, XPLMGetDataf(plane_elevation_dr),
                         &ramp->stand_x, &ramp->stand_y, &ramp->stand_z);
        ramp->ref_gen = ref_gen;
        ramp->dgs_assoc = 0;    // association is lost
    }
}

static inline void
global_2_ramp(const ramp_t * ramp, float x, float z, float *x_l, float *z_l)
{
    float dx = x - ramp->stand_x;
    float dz = z - ramp->stand_z;

    *x_l =  dx * ramp->cos_hdgt + dz * ramp->sin_hdgt;
    *z_l = -dx * ramp->sin_hdgt + dz * ramp->cos_hdgt;
}

//
// Accessor for the "opensam/dgs/..." datarefs
//
// This function is called from draw loops, efficient coding required.
//
//
static float
read_dgs_acc(void *ref)
{
    if (NULL == nearest_ramp)
        return 0.0f;

    stat_acc_called++;

    // a super cheap test first
    if (fabs(RA(nearest_ramp->hdgt - XPLMGetDataf(draw_object_psi_dr))) > 3.0f)
        return 0.0f;

    // check for shift of reference frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        log_msg("reference frame shift");
    }

    xform_to_ref_frame(nearest_ramp);

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);

    int dr_index = (uint64_t)ref;

    if (nearest_ramp->dgs_assoc) {
        if (nearest_ramp->dgs_x != obj_x || nearest_ramp->dgs_z != obj_z)
            return 0.0f;

        if (DGS_DR_UNHIDE == dr_index) {
            is_marshaller = 1;      // only Marshaller queries unhide
            return 1.0f;
        }

        return drefs[dr_index];
    }

    float dgs_x_l, dgs_z_l;
    global_2_ramp(nearest_ramp, obj_x, obj_z, &dgs_x_l, &dgs_z_l);

    if (fabs(dgs_x_l) > 0.5f || dgs_z_l < -50.0f)
        return 0.0;

    // match, associate dgs to ramp
    nearest_ramp->dgs_assoc = 1;
    nearest_ramp->dgs_x = obj_x;
    nearest_ramp->dgs_z = obj_z;

    if (DGS_DR_UNHIDE == dr_index) {
        is_marshaller = 1;      // only Marshaller queries unhide
        return 1.0f;
    }

    return drefs[dr_index];
}

static void
find_nearest_ramp()
{
    double dist = 1.0E10;
    ramp_t *min_ramp = NULL;

    float plane_lat = XPLMGetDataf(plane_lat_dr);
    float plane_lon = XPLMGetDataf(plane_lon_dr);

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);

    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        // cheap check against bounding box
        if (plane_lat < sc->bb_lat_min || plane_lat > sc->bb_lat_max
            || RA(plane_lon - sc->bb_lon_min) < 0 || RA(plane_lon - sc->bb_lon_max) > 0) {
            continue;
        }

        for (ramp_t *ramp = sc->ramps; ramp < sc->ramps + sc->n_ramps; ramp++) {

            // heading in local system
            float local_hdgt = RA(plane_hdgt - ramp->hdgt);

            if (fabs(local_hdgt) > 90.0)
                continue;   // not looking to ramp

            xform_to_ref_frame(ramp);

            float local_x, local_z;
            global_2_ramp(ramp, plane_x, plane_z, &local_x, &local_z);

            // nose wheel
            float nw_z = local_z - plane_nw_z;
            float nw_x = local_x + plane_nw_z * sin(D2R * local_hdgt);

            float d = len2f(nw_x, nw_z);
            if (d > CAP_Z + 50) // fast exit
                continue;

            //log_msg("stand: %s, z: %2.1f, x: %2.1f", ramp->id, nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                //log_msg("behind: %s",ramp->id);
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / D2R;
                //log_msg("angle to plane: %s, %3.1f",ramp->id, angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = RA(-angle - local_hdgt);

                //log_msg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //      ramp->id, nw_x, local_hdgt, rel_to_stand);

                if ((nw_x > 10.0 && rel_to_stand < -60.0)
                    || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    //log_msg("drive by %s",ramp->id);
                    continue;
                }
            }

            // for the final comparison give azimuth a higher weight
            static const float azi_weight = 4.0;
            d = len2f(azi_weight * nw_x, nw_z);

            if (d < dist) {
                //log_msg("new min: %s, z: %2.1f, x: %2.1f",ramp->id, nw_z, nw_x);
                dist = d;
                min_ramp = ramp;
            }
        }
    }

    if (min_ramp != NULL && min_ramp != nearest_ramp) {
        is_marshaller = 0;
        log_msg("ramp: %s, %f, %f, %f, dist: %f, dgs_dist: %0.2f", min_ramp->id,
                min_ramp->lat, min_ramp->lon,
                min_ramp->hdgt, dist, dgs_dist);

        nearest_ramp = min_ramp;
        state = ENGAGED;
    }
}


int
dgs_init()
{
    percent_lights_dr = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
    sin_wave_dr = XPLMFindDataRef("sim/graphics/animation/sin_wave_2");

    // create the dgs animation datarefs
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL,
                                 NULL, read_dgs_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(uint64_t)i, NULL);

    reset_state(INACTIVE);
    return 1;
}

float
dgs_state_machine()
{
    if (state <= INACTIVE)
        return 2.0;

    // throttle costly search
    if (INACTIVE < state && now > nearest_ramp_ts + 2.0) {
        find_nearest_ramp();
        nearest_ramp_ts = now;
    }

    if (nearest_ramp == NULL) {
        state = ACTIVE;
        return 2.0;
    }

    int lr_prev = lr;
    int track_prev = track;
    float distance_prev = distance;

    float loop_delay = 0.2;
    state_t new_state = state;

    // xform plane pos into stand local coordinate system

    float local_x, local_z;
    global_2_ramp(nearest_ramp, XPLMGetDataf(plane_x_dr), XPLMGetDataf(plane_z_dr),
                  &local_x, &local_z);

    // relative reading to stand +/- 180
    float local_hdgt = RA(XPLMGetDataf(plane_true_psi_dr) - nearest_ramp->hdgt);

    // nose wheel
    float nw_z = local_z - plane_nw_z;
    float nw_x = local_x + plane_nw_z * sinf(D2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane_mw_z;
    float mw_x = local_x + plane_mw_z * sinf(D2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = clampf((nw_z - 6.0) / 20.0, 0.0, 1.0);
    float plane_z_dr = (1.0 - a) * plane_nw_z + a * plane_mw_z;
    float z_dr = local_z - plane_z_dr;
    float x_dr = local_x + plane_z_dr * sin(D2R * local_hdgt);

    if (fabs(x_dr) > 0.5 && z_dr > 0)
        azimuth = atanf(x_dr / (z_dr + 0.5 * dgs_dist)) / D2R;
    else
        azimuth = 0.0;

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 0.5 * dgs_dist)) / D2R;
    else
        azimuth_nw = 0.0;

    int locgood = (fabsf(mw_x) <= GOOD_X && fabsf(nw_z) <= GOOD_Z);
    int beacon_on = check_beacon();

    status = lr = track = 0;
    distance = nw_z - GOOD_Z;

    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    int phase180 = (sin_wave_prev > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev = sin_wave;

    /* set drefs according to *current* state */
    switch (state) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance <= CAP_Z) && (fabsf(azimuth_nw) <= CAP_A))
                    new_state = TRACK;
            } else { // not beacon_on
                new_state = DONE;
            }
            break;

        case TRACK:
            if (locgood) {
                new_state = GOOD;
                break;
            }

            if (nw_z < -GOOD_Z) {
                new_state = BAD;
                break;
            }

            if ((distance > CAP_Z) || (fabsf(azimuth_nw) > CAP_A)) {
                new_state = ENGAGED;    // moving away from current gate
                break;
            }

            status = 1;	/* plane id */
            if (distance > AZI_Z || fabsf(azimuth_nw) > AZI_A) {
                track=1;	/* lead-in only */
                break;
            }

            /* compute distance and guidance commands */
            azimuth = clampf(azimuth, -AZI_A, AZI_A);
            float req_hdgt = -3.5 * azimuth;        // to track back to centerline
            float d_hdgt = req_hdgt - local_hdgt;   // degrees to turn

            if (now > update_ramp_log_ts + 2.0)
                log_msg("is_marshaller: %d, azimuth: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), ref: (%0.1f, %0.1f), "
                       "x: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f",
                       is_marshaller, azimuth, mw_x, mw_z, nw_x, nw_z,
                       x_dr, z_dr,
                       local_x, local_hdgt, d_hdgt);

            if (d_hdgt < -1.5)
                lr = 2;
            else if (d_hdgt > 1.5)
                lr = 1;

            // xform azimuth to values required by OBJ
            azimuth = clampf(azimuth, -AZI_DISP_A, AZI_DISP_A) * 4.0 / AZI_DISP_A;
            azimuth=((float)((int)(azimuth * 2))) / 2;  // round to 0.5 increments

            if (distance <= REM_Z/2) {
                track = 3;
                loop_delay = 0.03;
            } else /* azimuth only */
                track = 2;

            if (! phase180) { // no wild oscillation
                lr = lr_prev;

                // sync transition with Marshaller's arm movement
                if (is_marshaller && track == 3 && track_prev == 2) {
                    track = track_prev;
                    distance = distance_prev;
                }
            }
            break;

        case GOOD:
            /* @stop position*/
            status = 2; lr = 3;

            int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5);
            if (!locgood)
                new_state = TRACK;
            else if (parkbrake_set || !beacon_on)
                new_state = PARKED;
            break;

        case BAD:
            if (!beacon_on
                && (now > timestamp + 5.0)) {
                reset_state(INACTIVE);
                return loop_delay;
            }

            if (nw_z >= -GOOD_Z)
                new_state = TRACK;
            else {
                /* Too far */
                status = 4;
                lr = 3;
            }
            break;

        case PARKED:
            status = 3;
            lr = 0;
            // wait for beacon off
            if (! beacon_on)
                new_state = DONE;
            break;

        case DONE:
            if (now > timestamp + 5.0) {
                if (!dont_connect_jetway)   // wait some seconds for the jw handler to catch up
                    dock_requested = 1;

                reset_state(INACTIVE);
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state) {
        log_msg("dgs state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        timestamp = now;
        return -1;  // see you on next frame
    }

    if (state > ACTIVE) {
        /* xform drefs into required constraints for the OBJs */
        if (track == 0 || track == 1) {
            distance = 0;
            azimuth = 0.0;
        }

        distance = clampf(distance, -GOOD_Z, REM_Z);

        // is not necessary for Marshaller + SafedockT2
        // distance=((float)((int)((distance)*2))) / 2;    // multiple of 0.5m

        static const float min_brightness = 0.025;   // relativ to 1
        float brightness = min_brightness + (1 - min_brightness) * powf(1 - XPLMGetDataf(percent_lights_dr), 1.5);

        // don't flood the log
        if (now > update_ramp_log_ts + 2.0) {
            update_ramp_log_ts = now;
            log_msg("ramp: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, azimuth: %0.1f, brightness: %0.2f",
                   nearest_ramp->id, state_str[state], status, track, lr, distance, azimuth, brightness);
        }

        memset(drefs, 0, sizeof(drefs));
        drefs[DGS_DR_STATUS] = status;
        drefs[DGS_DR_TRACK] = track;
        drefs[DGS_DR_DISTANCE] = distance;
        drefs[DGS_DR_AZIMUTH] = azimuth;
        drefs[DGS_DR_LR] = lr;

        if (state == TRACK) {
            for (int i = 0; i < 4; i++)
                drefs[DGS_DR_ICAO_0 + i] = acf_icao[i];

            if (isalpha(acf_icao[3]))
                drefs[DGS_DR_ICAO_3] += 0.98;    // bug in VDGS
        }

        drefs[DGS_DR_BRIGHTNESS] = brightness;
    }

    return loop_delay;
}
