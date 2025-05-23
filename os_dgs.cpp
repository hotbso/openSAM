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

#include <cstddef>
#include <cstring>

#include "openSAM.h"
#include "os_dgs.h"
#include "plane.h"

#include "XPLMInstance.h"
#include "XPLMNavigation.h"

// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static constexpr float CAP_A = 15;              // Capture
static constexpr float CAP_Z = 140;	            // (50-80 in Safedock2 flier)

static constexpr float AZI_A = 15;              // provide azimuth guidance
static constexpr float AZI_DISP_A = 10;         // max value for display
static constexpr float AZI_Z = 90;

static constexpr float GOOD_Z= 0.5;             // stop position for nw
static constexpr float GOOD_X = 2.0;            // for mw

static constexpr float REM_Z = 12;      	    // Distance remaining from here on

static constexpr float MAX_DGS_2_STAND_X = 10.0f; // max offset/distance from DGS to stand
static constexpr float MAX_DGS_2_STAND_Z = 80.0f;

static constexpr float dgs_dist = 20.0f;        // distance from dgs to stand for azimuth computation

// types
typedef enum
{
    DISABLED=0, INACTIVE, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, CHOCKS, DONE
} state_t;

static const char * const state_str[] = {
    "DISABLED", "INACTIVE", "ACTIVE", "ENGAGED",
    "TRACK", "GOOD", "BAD", "PARKED", "CHOCKS", "DONE" };
static state_t state = DISABLED;
static float timestamp; // for various states in the state_machine

// Datarefs
static XPLMDataRef percent_lights_dr, sin_wave_dr,
    zulu_time_minutes_dr, zulu_time_hours_dr;

// Published DataRef values
static int status, track, lr;
static float azimuth, distance;

static Stand *nearest_stand;
static float nearest_stand_ts;    // timestamp of last find_nearest_stand()
// track the max local z (= closest to stand) of dgs objs for nearest_stand
static float assoc_dgs_z_l, assoc_dgs_x_l, assoc_dgs_ts;

// flag if stand is associated with a dgs
static int dgs_assoc;

static int is_marshaller;
static float marshaller_x, marshaller_y, marshaller_z, marshaller_y_0, marshaller_psi;
static XPLMObjectRef marshaller_obj, stairs_obj;
static XPLMInstanceRef marshaller_inst, stairs_inst;

static int update_stand_log_ts;   // throttling of logging
static float sin_wave_prev;

static float time_utc_m0, time_utc_m1, time_utc_h0, time_utc_h1, vdgs_brightness;

enum _DGS_DREF {
    DGS_DR_IDENT,
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_DISTANCE_0,      // if distance < 10: full meters digit
    DGS_DR_DISTANCE_01,     // first decimal digit
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,
    DGS_DR_NUM              // # of drefs
};

// keep exactly the same order as list above
static const char *dgs_dlist_dr[] = {
    "opensam/dgs/ident",
    "opensam/dgs/status",
    "opensam/dgs/lr",
    "opensam/dgs/track",
    "opensam/dgs/azimuth",
    "opensam/dgs/distance",
    "opensam/dgs/distance_0",
    "opensam/dgs/distance_01",
    "opensam/dgs/icao_0",
    "opensam/dgs/icao_1",
    "opensam/dgs/icao_2",
    "opensam/dgs/icao_3",
    NULL
};

static float drefs[DGS_DR_NUM];

// SAM1 drefs
enum _SAM1_DREF {
    SAM1_DR_STATUS,
    SAM1_DR_LATERAL,
    SAM1_DR_LONGITUDINAL,
    SAM1_DR_ICAO,
};

enum _SAM1_STATE {
    SAM1_TRACK = 1,
    SAM1_STOP_ZONE,
    SAM1_IDLE
};
static constexpr float SAM1_LATERAL_OFF = 10.0f;   // switches off VDGS

// dref values
static float sam1_status, sam1_lateral, sam1_longitudinal;


void
dgs_set_inactive(void)
{
    log_msg("dgs set to INACTIVE");
    nearest_stand = NULL;
    state = INACTIVE;

    if (marshaller_inst) {
        XPLMDestroyInstance(marshaller_inst);
        marshaller_inst = NULL;
        if (stairs_inst) {
            XPLMDestroyInstance(stairs_inst);
            stairs_inst = NULL;
        }
    }
}

// set mode to arrival
void
dgs_set_active(void)
{
    if (! my_plane.on_ground()) {
        log_msg("can't set active when not on ground");
        return;
    }

    // can be teleportation
    dgs_set_inactive();
    my_plane.reset_beacon();

    float lat = my_plane.lat();
    float lon = my_plane.lon();
    char airport_id[50];

    // find and load airport I'm on now
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, airport_id,
                NULL, NULL);
        log_msg("now on airport: %s", airport_id);
    }

    state = ACTIVE;
    log_msg("dgs set to ACTIVE");
}


// xform lat,lon into the active global frame
void
Stand::xform_to_ref_frame()
{
    if (ref_gen_ < ::ref_gen) {
        XPLMWorldToLocal(lat, lon, my_plane.elevation(),
                         &stand_x, &stand_y, &stand_z);
        ref_gen_ = ::ref_gen;
        dgs_assoc = 0;    // association is lost
        assoc_dgs_z_l = -1.0E10;
        assoc_dgs_x_l = 1.0E10;
        assoc_dgs_ts = 1.0E10;
    }
}

// xform global coordinates into the stand frame
void
Stand::global_2_stand(float x, float z, float& x_l, float& z_l)
{
    float dx = x - stand_x;
    float dz = z - stand_z;

    x_l =  dx * cos_hdgt + dz * sin_hdgt;
    z_l = -dx * sin_hdgt + dz * cos_hdgt;
}

//
// Check whether dgs obj is the (an) active one.
//
// We are looking for the DGS closest to the stand.
// First of all it must be in a box around the stand and reasonably aligned.
//
// Assume that we have an association with x/z values in assoc_dgs_[xz]...
//
// Primary criterion is max z value in the stand's local system
// but...
// if distance of new candidate (z) nearly the same but closer to stand's centerline(x)
//    consider it nearer even if z is slightly less
//
static inline bool
is_dgs_active(float obj_x, float obj_z, float obj_psi)
{
    if (NULL == nearest_stand)
        return false;

    stat_dgs_acc++;

    float dgs_x_l, dgs_z_l;
    nearest_stand->global_2_stand(obj_x, obj_z, dgs_x_l, dgs_z_l);
    //log_msg("dgs_x_l: %0.2f, dgs_z_l: %0.2f", dgs_x_l, dgs_z_l);

    if (dgs_assoc && (dgs_z_l < assoc_dgs_z_l - 2.0f || fabsf(dgs_x_l) > assoc_dgs_x_l))
        return false;   // already have a closer one

    // must be in a box +- MAX_DGS_2_STAND_X, MAX_DGS_2_STAND_Z
    // and reasonably aligned with stand (or for SAM1 anti aligned)
    if (fabsf(dgs_x_l) > MAX_DGS_2_STAND_X
        || dgs_z_l < -MAX_DGS_2_STAND_Z || dgs_z_l > -5.0f
        || BETWEEN(fabsf(RA(nearest_stand->hdgt - obj_psi)), 10.0f, 170.0f))
        return false;

    // we found one
    if ((dgs_z_l > assoc_dgs_z_l - 2.0f && fabsf(dgs_x_l) < assoc_dgs_x_l - 1.0f)
            || (dgs_z_l > assoc_dgs_z_l)) {
        is_marshaller = 0;   // associated to a new dgs, don't know yet whether it's a marshaller
        assoc_dgs_z_l = dgs_z_l;
        assoc_dgs_x_l = fabsf(dgs_x_l);
        assoc_dgs_ts = now;
        log_msg("associating DGS: dgs_x_l: %0.2f, dgs_z_l: %0.2f", dgs_x_l, dgs_z_l);
    }

    dgs_assoc = 1;
    return true;
}

// Dataref accessor for the global datarefs, *_utc_* + brightness
static float
DgsGlobalAcc(void *ref)
{
    if (ref == nullptr)
        return -1.0f;

    return *(float *)ref;
}

//
// Accessor for the "opensam/dgs/..." datarefs
//
// This function is called from draw loops, efficient coding required.
//
//
static float
DgsActiveAcc(void *ref)
{
    int dr_index = (uint64_t)ref;

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    if (!is_dgs_active(obj_x, obj_z, obj_psi))
        return 0.0f;

    if (DGS_DR_IDENT == dr_index) {
        if (fabsf(RA(nearest_stand->hdgt - obj_psi)) > 10.0f)   // no anti alignment for the Marshaller
            return 0.0;

        // if last nearest dgs was found 2 seconds ago
        // this should be the nearest one in this stand's bbox
        if (now > assoc_dgs_ts + 2.0f) {
            is_marshaller = 1;      // only marshaller queries ident
            marshaller_x = obj_x;
            marshaller_y = XPLMGetDataf(draw_object_y_dr);
            marshaller_z = obj_z;
            marshaller_psi = obj_psi;
            return 0.0f;
        }
    }

    return drefs[dr_index];
}

//
// Accessor for the "sam/..." docking related datarefs
//
// This function is called from draw loops, efficient coding required.
//
static float
read_sam1_acc(void *ref)
{
    int dr_index = (uint64_t)ref;
    if (!is_dgs_active(XPLMGetDataf(draw_object_x_dr), XPLMGetDataf(draw_object_z_dr),
                       XPLMGetDataf(draw_object_psi_dr)))
        switch (dr_index) {
            case SAM1_DR_STATUS:
                return SAM1_IDLE;
            case SAM1_DR_LATERAL:
                return SAM1_LATERAL_OFF;           // switch off VDGS
            case SAM1_DR_LONGITUDINAL:
                return 0.0f;
        }

    //log_msg("read_sam1_acc: %d", dr_index);
    switch (dr_index) {
        case SAM1_DR_STATUS:
            return sam1_status;
        case SAM1_DR_LATERAL:
            return sam1_lateral;
        case SAM1_DR_LONGITUDINAL:
            return sam1_longitudinal;
    }

    return 0.0f;
}

//
// Accessor for the "sam/...icao[0]" docking related datarefs
//
// This function is called from draw loops, efficient coding required.
//
static int
read_sam1_icao_acc([[maybe_unused]] XPLMDataRef ref, int *values, int ofs, int n)
{
    if (values == NULL)
        return 4;

    if (n <= 0 || ofs < 0 || ofs >= 4)
        return 0;

    n = std::min(n, 4 - ofs);
    const char *icao = my_plane.icao().c_str();
    for (int i = 0; i < n; i++) {
        char c = icao[ofs + i];
        if (isalpha(c))
            values[i] = (c - 'A') + 1;
        else
            values[i] = (c - '0') + 27;
    }

    return n;
}

static void
find_nearest_stand()
{
    double dist = 1.0E10;
    Stand *min_stand = NULL;

    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    float plane_x = my_plane.x();
    float plane_z = my_plane.z();

    float plane_hdgt = my_plane.psi();

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (plane_lat < sc->bb_lat_min || plane_lat > sc->bb_lat_max
            || RA(plane_lon - sc->bb_lon_min) < 0 || RA(plane_lon - sc->bb_lon_max) > 0) {
            continue;
        }

        for (auto stand : sc->stands) {

            // heading in local system
            float local_hdgt = RA(plane_hdgt - stand->hdgt);

            if (fabs(local_hdgt) > 90.0)
                continue;   // not looking to stand

            stand->xform_to_ref_frame();

            float local_x, local_z;
            stand->global_2_stand(plane_x, plane_z, local_x, local_z);

            // nose wheel
            float nw_z = local_z - my_plane.nose_gear_z_;
            float nw_x = local_x + my_plane.nose_gear_z_ * sin(D2R * local_hdgt);

            float d = len2f(nw_x, nw_z);
            if (d > CAP_Z + 50) // fast exit
                continue;

            //log_msg("stand: %s, z: %2.1f, x: %2.1f", stand->id, nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                //log_msg("behind: %s",stand->id);
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / D2R;
                //log_msg("angle to plane: %s, %3.1f",stand->id, angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = RA(-angle - local_hdgt);

                //log_msg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //      stand->id, nw_x, local_hdgt, rel_to_stand);

                if ((nw_x > 10.0 && rel_to_stand < -60.0)
                    || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    //log_msg("drive by %s",stand->id);
                    continue;
                }
            }

            // for the final comparison give azimuth a higher weight
            static constexpr float azi_weight = 4.0;
            d = len2f(azi_weight * nw_x, nw_z);

            if (d < dist) {
                //log_msg("new min: %s, z: %2.1f, x: %2.1f",stand->id, nw_z, nw_x);
                dist = d;
                min_stand = stand;
            }
        }
    }

    if (min_stand != NULL && min_stand != nearest_stand) {
        is_marshaller = 0;

        if (marshaller_inst) {
            XPLMDestroyInstance(marshaller_inst);
            marshaller_inst = NULL;
            if (stairs_inst) {
                XPLMDestroyInstance(stairs_inst);
                stairs_inst = NULL;
            }
        }

        log_msg("stand: %s, %f, %f, %f, dist: %f, dgs_dist: %0.2f", min_stand->id,
                min_stand->lat, min_stand->lon,
                min_stand->hdgt, dist, dgs_dist);

        nearest_stand = min_stand;
        dgs_assoc = 0;
        assoc_dgs_z_l = -1.0E10;
        assoc_dgs_ts = 1.0E10;
        assoc_dgs_x_l = 1.0E10;
        state = ENGAGED;
    }
}

int
dgs_init()
{
    percent_lights_dr = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
    sin_wave_dr = XPLMFindDataRef("sim/graphics/animation/sin_wave_2");
    zulu_time_minutes_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_minutes");
    zulu_time_hours_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_hours");

    // create the dgs animation datarefs
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL,
                                 NULL, DgsActiveAcc, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(uint64_t)i, NULL);

    // these are served globally
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_m0", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m0, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_m1", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m1, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_h0", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h0, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_h1", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h1, 0);
    XPLMRegisterDataAccessor("opensam/dgs/vdgs_brightness", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &vdgs_brightness, 0);

    XPLMRegisterDataAccessor("sam/vdgs/status", xplmType_Float, 0, NULL,
                             NULL, read_sam1_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_STATUS, NULL);

    XPLMRegisterDataAccessor("sam/docking/lateral", xplmType_Float, 0, NULL,
                             NULL, read_sam1_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_LATERAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/longitudinal", xplmType_Float, 0, NULL,
                             NULL, read_sam1_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_LONGITUDINAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/icao", xplmType_IntArray, 0, NULL, NULL,
                             NULL, NULL, NULL, NULL, read_sam1_icao_acc, NULL,
                             NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_ICAO, NULL);

    // some custom VDGS use "sam/docking/status", e.g. Gaya LOWW
    XPLMRegisterDataAccessor("sam/docking/status", xplmType_Float, 0, NULL,
                             NULL, read_sam1_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_STATUS, NULL);

    marshaller_obj = XPLMLoadObject("Resources/plugins/openSAM/objects/Marshaller.obj");
    if (NULL == marshaller_obj) {
        log_msg("Could not load Marshaller.obj");
        return 0;
    }

    stairs_obj = XPLMLoadObject("Resources/default scenery/airport scenery/Ramp_Equipment/Stair_Maint_1.obj");
    if (NULL == stairs_obj) {
        log_msg("Could not load Stair_Maint_1.obj");
        return 0;
    }

    dgs_set_inactive();
    return 1;
}

float
dgs_state_machine()
{
    // update global dataref values
    static constexpr float min_brightness = 0.025;   // relativ to 1
    vdgs_brightness = min_brightness + (1 - min_brightness) * powf(1 - XPLMGetDataf(percent_lights_dr), 1.5);
    int zm = XPLMGetDatai(zulu_time_minutes_dr);
    int zh = XPLMGetDatai(zulu_time_hours_dr);
    time_utc_m0 = zm % 10;
    time_utc_m1 = zm / 10;
    time_utc_h0 = zh % 10;
    time_utc_h1 = zh / 10;

    if (state <= INACTIVE)
        return 1.0;

    // throttle costly search
    if (INACTIVE < state && now > nearest_stand_ts + 2.0) {
        find_nearest_stand();
        nearest_stand_ts = now;
    }

    if (nearest_stand == NULL) {
        state = ACTIVE;
        return 1.0;
    }

    int lr_prev = lr;
    int track_prev = track;
    float distance_prev = distance;

    float loop_delay = 0.2;
    state_t new_state = state;

    // xform plane pos into stand local coordinate system

    float local_x, local_z;
    nearest_stand->global_2_stand(my_plane.x(), my_plane.z(), local_x, local_z);

    // relative reading to stand +/- 180
    float local_hdgt = RA(my_plane.psi() - nearest_stand->hdgt);

    // nose wheel
    float nw_z = local_z - my_plane.nose_gear_z_;
    float nw_x = local_x + my_plane.nose_gear_z_ * sinf(D2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - my_plane.main_gear_z_;
    float mw_x = local_x + my_plane.main_gear_z_ * sinf(D2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = clampf((nw_z - 6.0f) / 20.0f, 0.0f, 1.0f);
    float plane_z_dr = (1.0f - a) * my_plane.nose_gear_z_ + a * my_plane.main_gear_z_;
    float z_dr = local_z - plane_z_dr;
    float x_dr = local_x + plane_z_dr * sin(D2R * local_hdgt);

    if (fabs(x_dr) > 0.5f && z_dr > 0)
        azimuth = atanf(x_dr / (z_dr + 0.5f * dgs_dist)) / D2R;
    else
        azimuth = 0.0;

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 0.5f * dgs_dist)) / D2R;
    else
        azimuth_nw = 0.0;

    int locgood = (fabsf(mw_x) <= GOOD_X && fabsf(nw_z) <= GOOD_Z);
    int beacon_on = my_plane.beacon_on();

    status = lr = track = 0;
    distance = nw_z - GOOD_Z;

    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    int phase180 = (sin_wave_prev > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev = sin_wave;

    // set drefs according to *current* state
    switch (state) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance <= CAP_Z) && (fabsf(azimuth_nw) <= CAP_A))
                    new_state = TRACK;
            } else { // not beacon_on
                new_state = DONE;
            }
            break;

        case TRACK: {
                if (!beacon_on) {       // don't get stuck in TRACK
                    new_state = DONE;
                    break;
                }

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

                status = 1;	// plane id
                if (distance > AZI_Z || fabsf(azimuth_nw) > AZI_A) {
                    track=1;	// lead-in only
                    break;
                }

                // compute distance and guidance commands
                azimuth = clampf(azimuth, -AZI_A, AZI_A);
                float req_hdgt = -3.5f * azimuth;        // to track back to centerline
                float d_hdgt = req_hdgt - local_hdgt;   // degrees to turn

                if (now > update_stand_log_ts + 2.0f)
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
                } else // azimuth only
                    track = 2;

                if (! phase180) { // no wild oscillation
                    lr = lr_prev;

                    // sync transition with Marshaller's arm movement
                    if (is_marshaller && track == 3 && track_prev == 2) {
                        track = track_prev;
                        distance = distance_prev;
                    }
                }
            }
            break;

        case GOOD: {
                // @stop position*/
                status = 2; lr = 3;

                int parkbrake_set = my_plane.parkbrake_set();
                if (!locgood)
                    new_state = TRACK;
                else if (parkbrake_set || !beacon_on)
                    new_state = PARKED;
            }
            break;

        case BAD:
            if (!beacon_on
                && (now > timestamp + 5.0f)) {
                dgs_set_inactive();
                return loop_delay;
            }

            if (nw_z >= -GOOD_Z)
                new_state = TRACK;
            else {
                // Too far
                status = 4;
                lr = 3;
            }
            break;

        case PARKED:
            status = 3;
            lr = 0;
            // wait for beacon off
            if (! beacon_on) {
                new_state = DONE;
                if (!my_plane.dont_connect_jetway_) { // check whether it's a ToLiss, then set chocks
                    XPLMDataRef tls_chocks = XPLMFindDataRef("AirbusFBW/Chocks");
                    if (tls_chocks) {
                        XPLMSetDatai(tls_chocks, 1);
                        if (! is_marshaller)
                            new_state = CHOCKS;
                    }
                }
            }
            break;

        case CHOCKS:
            status = 6;
            if (now > timestamp + 5.0)
                new_state = DONE;
            break;

        case DONE:
            if (now > timestamp + 3.0f) {
                if (!my_plane.dont_connect_jetway_)   // wait some seconds for the jw handler to catch up
                    my_plane.request_dock();

                dgs_set_inactive();
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
        // xform drefs into required constraints for the OBJs
        if (track == 0 || track == 1) {
            distance = 0;
            azimuth = 0.0;
        }

        distance = clampf(distance, -GOOD_Z, REM_Z);
        float d_0 = 0.0f;
        float d_01 = 0.0f;
        if (0.0f <= distance && distance < 10.0f) {
            d_0 = (float)(int)distance;
            d_01 = (int)((distance - d_0) * 10.0f);
        }

        distance = ((float)((int)((distance)*2))) / 2;    // multiple of 0.5m

        memset(drefs, 0, sizeof(drefs));
        drefs[DGS_DR_STATUS] = status;
        drefs[DGS_DR_TRACK] = track;
        drefs[DGS_DR_DISTANCE] = distance;
        drefs[DGS_DR_DISTANCE_0] = d_0;
        drefs[DGS_DR_DISTANCE_01] = d_01;
        drefs[DGS_DR_AZIMUTH] = azimuth;
        drefs[DGS_DR_LR] = lr;

        const char* icao = my_plane.icao().c_str();
        if (state == TRACK) {
            for (int i = 0; i < 4; i++)
                drefs[DGS_DR_ICAO_0 + i] = icao[i];
        }

        // translate into compatible SAM1 values
        sam1_lateral = -x_dr;
        sam1_longitudinal = std::min(z_dr, 30.0f);

        switch (state) {
            case ENGAGED:
            case TRACK:
                sam1_status = SAM1_TRACK;
                break;

            case GOOD:
            case PARKED:
                if (sam1_longitudinal < 0.1f) {
                    sam1_status = SAM1_STOP_ZONE;
                    sam1_lateral = 0.0f;
                } else
                    sam1_status = SAM1_TRACK;
                break;

            case BAD:
                sam1_status = SAM1_TRACK;
                break;

            case CHOCKS:
            case DONE:
                sam1_status = SAM1_IDLE;
                sam1_longitudinal = 0.0;
                break;

            default:
                sam1_status = SAM1_IDLE;
                sam1_lateral = SAM1_LATERAL_OFF;
                sam1_longitudinal = 0.0f;
        }

        if (is_marshaller && BETWEEN(state, ENGAGED, PARKED)) {
            XPLMDrawInfo_t drawinfo = {};
            drawinfo.structSize = sizeof(XPLMDrawInfo_t);
            drawinfo.heading = marshaller_psi;
            drawinfo.pitch = drawinfo.roll = 0.0;

            if (NULL == marshaller_inst) {
                log_msg("place marshaller at %0.2f, %0.2f, %0.2f, hdg: %0.1f°",
                        marshaller_x, marshaller_y, marshaller_z, marshaller_psi);

                marshaller_inst = XPLMCreateInstance(marshaller_obj, dgs_dlist_dr);
                if (marshaller_inst == NULL) {
                    log_msg("error creating marshaller instance");
                    state = DISABLED;
                    return 0.0;
                }

                // now check whether it's Marshaller_high

                if (xplm_ProbeHitTerrain == XPLMProbeTerrainXYZ(probe_ref, marshaller_x, marshaller_y, marshaller_z,
                                                               &probeinfo)) {
                    marshaller_y_0 = probeinfo.locationY;   // ground 0

                    if (marshaller_y - marshaller_y_0 > 2.0f) {
                        log_msg("Marshaller_high detected, place stairs");
                        static const char * null[] = {NULL};
                        stairs_inst = XPLMCreateInstance(stairs_obj, null);
                        if (stairs_inst == NULL) {
                            log_msg("error creating stairs instance");
                            state = DISABLED;
                            return 0.0;
                        }

                        // move slightly to the plane
                        static constexpr float delta_z = 1.0f;
                        drawinfo.x = marshaller_x - delta_z * nearest_stand->sin_hdgt;
                        drawinfo.y = marshaller_y_0;
                        drawinfo.z = marshaller_z + delta_z * nearest_stand->cos_hdgt;
                        XPLMInstanceSetPosition(stairs_inst, &drawinfo, NULL);
                    }
                }
            }

            // update datarefs
            drawinfo.x = marshaller_x;
            drawinfo.y = marshaller_y;
            drawinfo.z = marshaller_z;
            XPLMInstanceSetPosition(marshaller_inst, &drawinfo, drefs);
        }

        // don't flood the log
        if (now > update_stand_log_ts + 2.0f) {
            update_stand_log_ts = now;
            log_msg("stand: %s, state: %s, assoc: %d, status: %d, track: %d, lr: %d, distance: %0.2f, azimuth: %0.1f",
                   nearest_stand->id, state_str[state], dgs_assoc,
                   status, track, lr, distance, azimuth);
            log_msg("sam1: status %0.0f, lateral: %0.1f, longitudinal: %0.1f",
                    sam1_status, sam1_lateral, sam1_longitudinal);
        }
    }

    return loop_delay;
}
