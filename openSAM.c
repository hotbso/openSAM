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
#include <ctype.h>
#include <math.h>

#include "openSAM.h"
#include "os_dgs.h"
#include "os_jw.h"

#include "XPLMPlanes.h"

/*
 * On the various coordinate systems and angles:
 *
 * Objects are drawn in a +x east , -z true north, +y up system.
 * Headings (hdgt) are measured from -z (=true north) right turning
 *
 * Imagine looking from below to the sky you have the more traditional 'math' view
 * +x right, +z up, angles left turning from the +x axis to the +z axis.
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
 * acf nose pointing up to -z, door at (0,0), jetways to the left somewhere at -x and some z.
 *
 * These values are kept in the "active_jw".
 *
 * Then we do our math in the door frame and transform everything back to the jetway frame
 * to get the rotation angles.
 *
 * The datarefs for jetway animation are:
 *  rotation1       tunnel relative to the placed object
 *  rotation2       cabin relative to the tunnel
 *  rotation3       tunnel relative to horizontal (= x-z plane)
 *  wheelrotatec    wheel base relative to tunnel around the y-axis
 *  wheelrotater    right wheel
 *  wheelrotatel    left wheel
 *  wheel           delta height in m of tunnel over wheelbase relative to horizontal
 *
 * Likewise for DGS we xform everything into the stand frame and go from there.
 *
 */


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

XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, plane_y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr, acf_gear_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

int on_ground = 1;
static float on_ground_ts;

float lat_ref = -1000, lon_ref = -1000;
/* generation # of reference frame
 * init with 1 so jetways never seen by the accessor won't be considered in find_dockable_jws() */
unsigned int ref_gen = 1;

float parked_x, parked_y;
int parked_ngen;

float now;          // current timestamp
char base_dir[512]; // base directory of openSAM

int beacon_state, beacon_last_pos;   // beacon state, last switch_pos, ts of last switch actions
float beacon_off_ts, beacon_on_ts;

int use_engine_running;              // instead of beacon, e.g. MD11
int dont_connect_jetway;             // e.g. for ZIBO with own ground service
static float plane_cg_y;

int n_door;
door_info_t door_info[MAX_DOOR];

char acf_icao[5];

unsigned long long int stat_sc_far_skip, stat_far_skip, stat_near_skip,
    stat_acc_called, stat_jw_match;

int dock_requested, undock_requested, toggle_requested;

static void
save_pref()
{
    FILE *f = fopen(pref_path, "w");
    if (NULL == f)
        return;

    // encode southern hemisphere with negative season
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

int
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

static int
cmd_dock_jw_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(cmdr);

    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd_dock_jw_cb called");

    *(int *)ref = 1;
     return 0;
}

static int
cmd_activate_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(cmdr);
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd manually_activate");
    dgs_set_active();
    return 0;
}

int
check_teleportation()
{
    float x = XPLMGetDataf(plane_x_dr);
    float y = XPLMGetDataf(plane_y_dr);
    int ngen = ref_gen;

    if (parked_ngen != ngen || fabsf(parked_x - x) > 1.0f || fabsf(parked_y - y) > 1.0f) {
        log_msg("parked_ngen: %d, ngen: %d, parked_x: %0.3f, x: %0.3f, parked_y: %0.3f, y: %0.3f",
                parked_ngen, ngen, parked_x, x, parked_y, y);

        on_ground_ts = now + 10.0f; // wait for the dust to settle
        on_ground = 1;
        return 1;
    }

    return 0;
}


static float
flight_loop_cb(float inElapsedSinceLastCall,
               float inElapsedTimeSinceLastFlightLoop, int inCounter,
               void *inRefcon)
{
    UNUSED(inElapsedSinceLastCall);
    UNUSED(inElapsedTimeSinceLastFlightLoop);
    UNUSED(inCounter);
    UNUSED(inRefcon);

    static float jw_next_ts, dgs_next_ts;

    now = XPLMGetDataf(total_running_time_sec_dr);
    int og = (XPLMGetDataf(gear_fnrml_dr) != 0.0);

    if (og != on_ground && now > on_ground_ts + 10.0f) {
        on_ground = og;
        on_ground_ts = now;
        log_msg("transition to on_ground: %d", on_ground);

        if (on_ground)
            dgs_set_active();
        else
            dgs_set_inactive();
    }

    float jw_loop_delay = 1.0f;
    float dgs_loop_delay = 1.0f;
    if (jw_next_ts <= now) {
        jw_loop_delay = jw_state_machine();
        jw_next_ts = now + jw_loop_delay;
    }

    if (dgs_next_ts <= now) {
        dgs_loop_delay = dgs_state_machine();
        dgs_next_ts = now + dgs_loop_delay;
    }

    return MIN(jw_loop_delay, dgs_loop_delay);
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

// emulate a kind of radio buttons
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
    UNUSED(menu_ref);

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
        } // else nothing, unchecking is not possible

        auto_season = 0;    // selecting a season always goes to manual mode
   }

    set_menu();
    save_pref();
}

static int
find_icao_in_file(const char *acf_icao, const char *dir, const char *fn,
                  char *line, int len)
{
    char fn_full[512];
    snprintf(fn_full, sizeof(fn_full) - 1, "%s%s", dir, fn);

    int res = 0;
    FILE *f = fopen(fn_full, "r");
    line[len-1] = '\0';
    if (f) {
        log_msg("check whether acf '%s' is in  file %s", acf_icao, fn_full);
        while (fgets(line, len-1, f)) {
            char *cptr = strchr(line, '\r');
            if (cptr)
                *cptr = '\0';
            cptr = strchr(line, '\n');
            if (cptr)
                *cptr = '\0';

            if (line == strstr(line, acf_icao)) {
                log_msg("found acf %s in %s", acf_icao, fn);
                res = 1;
                break;
            }
        }

        fclose(f);
    }

    return res;
}

// =========================== plugin entry points ===============================================
PLUGIN_API int
XPluginStart(char *out_name, char *out_sig, char *out_desc)
{
    log_msg("Startup " VERSION);

    strcpy(out_name, "openSAM " VERSION);
    strcpy(out_sig, "openSAM.hotbso");
    strcpy(out_desc, "A plugin that emulates SAM");

    // Always use Unix-native paths on the Mac!
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    char xp_dir[512];
	XPLMGetSystemPath(xp_dir);
    psep = XPLMGetDirectorySeparator();

    // set pref path
    XPLMGetPrefsPath(pref_path);
    XPLMExtractFileAndPath(pref_path);
    strcat(pref_path, psep);
    strcat(pref_path, "sam_se.prf");

    // get my base dir
    XPLMGetPluginInfo(XPLMGetMyID(), NULL, base_dir, NULL, NULL);
    char *cptr = strrchr(base_dir, '/');
    if (cptr)
        *cptr = '\0';

    cptr = strrchr(base_dir, '/');
    if (cptr)
        *(cptr + 1) = '\0';         // keep /

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
    acf_gear_z_dr = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    acf_door_x_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    acf_door_y_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    acf_door_z_dr = XPLMFindDataRef("sim/aircraft/view/acf_door_z");

    total_running_time_sec_dr = XPLMFindDataRef("sim/time/total_running_time_sec");
    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");

    // create the seasons datarefs
    for (int i = 0; i < 4; i++)
        XPLMRegisterDataAccessor(dr_name[i], xplmType_Int, 0, read_season_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(long long)i, NULL);

    // own commands
    XPLMCommandRef dock_cmdr = XPLMCreateCommand("openSAM/dock_jwy", "Dock jetway");
    XPLMRegisterCommandHandler(dock_cmdr, cmd_dock_jw_cb, 0, &dock_requested);

    XPLMCommandRef undock_cmdr = XPLMCreateCommand("openSAM/undock_jwy", "Undock jetway");
    XPLMRegisterCommandHandler(undock_cmdr, cmd_dock_jw_cb, 0, &undock_requested);

    XPLMCommandRef toggle_cmdr = XPLMCreateCommand("openSAM/toggle_jwy", "Toggle jetway");
    XPLMRegisterCommandHandler(toggle_cmdr, cmd_dock_jw_cb, 0, &toggle_requested);

    XPLMCommandRef activate_cmdr = XPLMCreateCommand("openSAM/activate", "Manually activate searching for DGS");
    XPLMRegisterCommandHandler(activate_cmdr, cmd_activate_cb, 0, NULL);

    // build menues
    XPLMMenuID menu = XPLMFindPluginsMenu();
    XPLMMenuID os_menu = XPLMCreateMenu("openSAM", menu,
                                        XPLMAppendMenuItem(menu, "openSAM", NULL, 0),
                                        NULL, NULL);
    // openSAM
    XPLMAppendMenuItemWithCommand(os_menu, "Dock Jetway", dock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Undock Jetway", undock_cmdr);

    XPLMAppendMenuSeparator(os_menu);

    XPLMAppendMenuItemWithCommand(os_menu, "Manually activate searching for DGS", activate_cmdr);

    XPLMAppendMenuSeparator(os_menu);

    // openSAM -> Seasons
    int seasons_menu_item = XPLMAppendMenuItem(os_menu, "Seasons", NULL, 0);
    seasons_menu = XPLMCreateMenu("Seasons", os_menu, seasons_menu_item, menu_cb, NULL);

    auto_item = XPLMAppendMenuItem(seasons_menu, "Automatic", (void *)4, 0);

    XPLMAppendMenuSeparator(seasons_menu);

    season_item[0] = XPLMAppendMenuItem(seasons_menu, "Winter", (void *)0, 0);
    season_item[1] = XPLMAppendMenuItem(seasons_menu, "Spring", (void *)1, 0);
    season_item[2] = XPLMAppendMenuItem(seasons_menu, "Summer", (void *)2, 0);
    season_item[3] = XPLMAppendMenuItem(seasons_menu, "Autumn", (void *)3, 0);
    // ---------------------

    load_pref();
    set_menu();

    if (!collect_sam_xml(xp_dir))
        log_msg("Error collecting sam.xml files!");

    log_msg("%d sceneries with sam jetways found", n_sceneries);

    jw_init();
    dgs_init();

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
    log_msg("acc called:       %llu", stat_acc_called);
    log_msg("scenery far skip: %llu", stat_sc_far_skip);
    log_msg("far skip:         %llu", stat_far_skip);
    log_msg("near skip:        %llu", stat_near_skip);
}


PLUGIN_API int
XPluginEnable(void)
{
    return 1;
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID in_from, long in_msg, void *in_param)
{
    UNUSED(in_from);

    // Everything before XPLM_MSG_AIRPORT_LOADED has bogus datarefs.
    //   Anyway it's too late for the current scenery.
    if ((in_msg == XPLM_MSG_AIRPORT_LOADED) ||
        (airport_loaded && (in_msg == XPLM_MSG_SCENERY_LOADED))) {
        airport_loaded = 1;
        nh = (XPLMGetDatad(plane_lat_dr) >= 0.0);
        set_season_auto();
        return;
    }

    // my plane loaded
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        memset(acf_icao, 0, 4);
        XPLMGetDatab(acf_icao_dr, acf_icao, 0, 4);
        acf_icao[4] = '\0';

        for (int i=0; i < 4; i++)
            acf_icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';

        plane_cg_y = F2M * XPLMGetDataf(acf_cg_y_dr);
        plane_cg_z = F2M * XPLMGetDataf(acf_cg_z_dr);

        float gear_z[2];
        if (2 == XPLMGetDatavf(acf_gear_z_dr, gear_z, 0, 2)) {      // nose + main wheel
            plane_nw_z = -gear_z[0];
            plane_mw_z = -gear_z[1];
        } else
            plane_nw_z = plane_mw_z = plane_cg_z;         // fall back to CG

        // check whether acf is listed in exception files
        use_engine_running = 0;
        dont_connect_jetway = 0;

        char line[200];
        if (find_icao_in_file(acf_icao, base_dir, "acf_use_engine_running.txt", line, sizeof(line))) {
            use_engine_running = 1;
            log_msg("found");
        }

        if (find_icao_in_file(acf_icao, base_dir, "acf_dont_connect_jetway.txt", line, sizeof(line))) {
            dont_connect_jetway = 1;
            log_msg("found");
        }

        door_info[0].x = XPLMGetDataf(acf_door_x_dr);
        door_info[0].y = XPLMGetDataf(acf_door_y_dr);
        door_info[0].z = XPLMGetDataf(acf_door_z_dr);

        n_door = 1;

        log_msg("plane loaded: %s, plane_cg_y: %1.2f, plane_cg_z: %1.2f, "
                "door 1: x: %1.2f, y: %1.2f, z: %1.2f",
                acf_icao, plane_cg_y, plane_cg_z,
                door_info[0].x, door_info[0].y, door_info[0].z);

        // check for a second door, seems to be not available by dataref
        // data in the acf file is often bogus, so check our own config file first
        line[sizeof(line) - 1] = '\0';
        if (find_icao_in_file(acf_icao, base_dir, "acf_door_position.txt", line, sizeof(line))) {
            int d;
            float x, y, z;
            if (4 == sscanf(line + 4, "%d %f %f %f", &d, &x, &y, &z)) {
                if (d == 2) {   // only door 2 for now
                    d--;
                    door_info[d].x = x;
                    door_info[d].y = y;
                    door_info[d].z = z;
                    n_door = 2;
                    log_msg("found door 2 in config file: x: %0.2f, y: %0.2f, z: %0.2f",
                            door_info[1].x, door_info[1].y, door_info[1].z);
                }
            }
        }

        return;
    }
}
