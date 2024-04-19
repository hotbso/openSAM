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
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"

#include "openSAM.h"

static const float D2R = M_PI/180.0;
static const float F2M = 0.3048;	/* 1 ft [m] */
static const float LON_D2M = 111120;    /* 1° lon in m */
static const float FAR_SKIP = 4000; /* don't consider jetways farther than that */
static const float NEAR_SKIP = 2; /* don't consider jetways farther than that */

static char pref_path[512];
static const char *psep;
static XPLMMenuID menu_id;
static int auto_item, season_item[4];
static int auto_season;
static int airport_loaded;

static XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

static XPLMProbeRef probe_ref;

typedef enum
{
    DISABLED=0, IDLE, PARKED, CAN_DOCK, DOCKING, DOCKED, CANT_DOCK
} state_t;

const char * const state_str[] = {
    "DISABLED", "IDLE", "PARKED", "CAN_DOCK",
    "DOCKING", "DOCKED", "CANT_DOCK" };

static int on_ground = 1;
static float on_ground_ts;
static state_t state = IDLE;

static int nh;     // on northern hemisphere
static int season; // 0-3
static const char *dr_name[] = {"sam/season/winter", "sam/season/spring",
            "sam/season/summer", "sam/season/autumn"};

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

static float lat_ref = -1000, lon_ref = -1000;
static unsigned int ref_gen;    /* generation # of reference frame */

static float now;           /* current timestamp */
static int beacon_state, beacon_last_pos;   /* beacon state, last switch_pos, ts of last switch actions */
static float beacon_off_ts, beacon_on_ts;

static int use_engine_running;              /* instead of beacon, e.g. MD11 */
static int dont_connect_jetway;             /* e.g. for ZIBO with own ground service */
static float plane_cg_y, plane_cg_z;


static unsigned long long int stat_far_skip, stat_near_skip, stat_acc_called, stat_jw_match;


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

// Accessor for the "sam/jetway/*" datarefs
static float
read_jw_acc(void *ref)
{
    stat_acc_called++;

    //plane_door_x = XPLMGetDataf(acf_door_x_dr);
    //log_msg("plane_door_x : %.2f", plane_door_x);
    now = XPLMGetDataf(total_running_time_sec_dr);

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_y = XPLMGetDataf(draw_object_y_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);

    // check for shift of regerence frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        log_msg("reference frame shift");
    }

    for (sam_jw_t *jw = sam_jws; jw < sam_jws + n_sam_jws; jw++) {
        //log_msg("lat: %f %f", lat, jw->latitude);
        //log_msg("lon: %f %f", lon, jw->longitude);

        float dlon_m = fabs(lon - jw->longitude) * LON_D2M;
        float dlat_m = fabs(lat - jw->latitude) * cosf(D2R * lat) * LON_D2M;

        if (dlon_m > FAR_SKIP || dlat_m > FAR_SKIP) {
            stat_far_skip++;
            continue;
        }

        if (jw->ref_gen < ref_gen) {
            XPLMWorldToLocal(jw->latitude, jw->longitude, 0.0, &jw->x, &jw->y, &jw->z);
            jw->psi = XPLMGetDataf(draw_object_psi_dr);
            jw->ref_gen = ref_gen;
        }

        if (fabs(obj_x - jw->x) > NEAR_SKIP || fabs(obj_z - jw->z) > NEAR_SKIP) {
            stat_near_skip++;
            continue;
        }

        // have a match
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

/* try to find dockable jetways and save their info */
static int
find_dockable_jws()
{
    return 0;
}

#if 0
static void
reset_state(state_t new_state)
{
    if (state != new_state)
        log_msg("setting state to %s", state_str[new_state]);

    state = new_state;
}

/* set mode to arrival */
static void
set_active(void)
{
    if (! on_ground) {
        log_msg("can't set active when not on ground");
        return;
    }

    if (state > INACTIVE)
        return;

    beacon_state = beacon_last_pos = XPLMGetDatai(beacon_dr);
    beacon_on_ts = beacon_off_ts = -10.0;

    log_msg("new state: ACTIVE");
    state = ACTIVE;
}
#endif

/* the state machine triggered by the flight loop */
static float
run_state_machine()
{
    if (state == DISABLED)
        return 2.0;

    state_t new_state = state;

    int beacon_on = check_beacon();
    if (beacon_on)
        return 0.5;

    switch (state) {
        case IDLE:
            if (on_ground && !beacon_on)
                new_state = PARKED;
            break;

        case PARKED:
            if (find_dockable_jws())
                new_state = CANT_DOCK;
            else
                new_state = CANT_DOCK;
            break;

        case CAN_DOCK:
            break;

        case CANT_DOCK:
            break;

        default:
            log_msg("Bad state %d", state);
            new_state = DISABLED;
            break;
    }

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

#if 0
        if (on_ground) {
            set_active();
        } else {
            reset_state(INACTIVE);
            if (probe_ref) {
                XPLMDestroyProbe(probe_ref);
                probe_ref = NULL;
            }
        }
#endif
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
    XPLMCheckMenuItem(menu_id, auto_item,
                      auto_season ? xplm_Menu_Checked : xplm_Menu_Unchecked);

    XPLMCheckMenuItem(menu_id, season_item[season], xplm_Menu_Checked);
    for (int i = 0; i < 4; i++)
        if (i != season)
            XPLMCheckMenuItem(menu_id, season_item[i], xplm_Menu_Unchecked);
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
        XPLMCheckMenuItemState(menu_id, season_item[entry], &checked);
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
    XPLMMenuID menu;
    int sub_menu;

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
    y_agl_dr = XPLMFindDataRef("sim/flightmodel2/position/y_agl");

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

    menu = XPLMFindPluginsMenu();
    sub_menu = XPLMAppendMenuItem(menu, "openSAM", NULL, 1);
    menu_id = XPLMCreateMenu("openSAM", menu, sub_menu, menu_cb, NULL);

    auto_item = XPLMAppendMenuItem(menu_id, "Automatic", (void *)4, 0);
    XPLMAppendMenuSeparator(menu_id);
    season_item[0] = XPLMAppendMenuItem(menu_id, "Winter", (void *)0, 0);
    season_item[1] = XPLMAppendMenuItem(menu_id, "Spring", (void *)1, 0);
    season_item[2] = XPLMAppendMenuItem(menu_id, "Summer", (void *)2, 0);
    season_item[3] = XPLMAppendMenuItem(menu_id, "Autumn", (void *)3, 0);

    load_pref();
    set_menu();

    if (!collect_sam_xml(xp_dir))
        log_msg("Error collecting sam.xml files!");

    log_msg("%d sam jetways found", n_sam_jws);

    if (n_sam_jws > 0) {
        for (sam_jw_t *jw = sam_jws; jw < sam_jws + n_sam_jws; jw++) {
            jw->rotate1 = jw->initialRot1;
            jw->rotate2 = jw->initialRot2;
            jw->rotate3 = jw->initialRot3;
            jw->extent = jw->initialExtent;
            //log_msg("%s %5.6f %5.6f", jw->name, jw->latitude, jw->longitude);
       }

        /* create the jetway datarefs */
        for (dr_code_t drc = DR_ROTATE1; drc < N_JW_DR; drc++)
            XPLMRegisterDataAccessor(dr_name_jw[drc], xplmType_Float, 0, NULL,
                                     NULL, read_jw_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void *)(long long)drc, NULL);
    }

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
