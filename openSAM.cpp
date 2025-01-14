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
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>

#include "openSAM.h"
#include "plane.h"
#include "os_dgs.h"
#include "samjw.h"
#include "jwctrl.h"
#include "os_anim.h"

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

//
// C++ style:
// This is largely moved over code from C to C++.
// Whenever something is refactored or added it should be formatted according to
// Google's style: https://google.github.io/styleguide/cppguide.html
//
//------------------------------------------------------------------------------------
//
// On the various coordinate systems and angles:
//
// Objects are drawn in a +x east , -z true north, +y up system.
// Headings (hdgt) are measured from -z (=true north) right turning
//
// Imagine looking from below to the sky you have the more traditional 'math' view
// +x right, +z up, angles left turning from the +x axis to the +z axis.
//
// So if alpha is a 'math' angle we have
//
// hdgt = 90° + alpha.
//
// If we have a coordinate system that is rotated by an angle of psi we have
//
// hdgt\rot = hdgt - psi
//
// When it comes to object rotations or longitudes we use relative angles in (-180, +180]
// This is done by RA().
//
// So first of all we rotate and shift everything into the 'door' frame i.e.
// acf nose pointing up to -z, door at (0,0), jetways to the left somewhere at -x and some z.
//
// These values are kept in the "active_jw".
//
// Then we do our math in the door frame and transform everything back to the jetway frame
// to get the rotation angles.
//
// The datarefs for jetway animation are:
//  rotation1       tunnel relative to the placed object
//  rotation2       cabin relative to the tunnel
//  rotation3       tunnel relative to horizontal (= x-z plane)
//  wheelrotatec    wheel base relative to tunnel around the y-axis
//  wheelrotater    right wheel
//  wheelrotatel    left wheel
//  wheel           delta height in m of tunnel over wheelbase relative to horizontal
//
// Likewise for DGS we xform everything into the stand frame and go from there.
//
///

static int init_fail;
std::string xp_dir;
static std::string pref_path;
static XPLMMenuID seasons_menu;
static int auto_item, season_item[4];
static int auto_season;
static int airport_loaded;
static int nh;     // on northern hemisphere
static int season; // 0-3
static const char *dr_name[] = {"sam/season/winter", "sam/season/spring",
            "sam/season/summer", "sam/season/autumn"};
static int sam_library_installed;

static XPLMDataRef date_day_dr;

XPLMDataRef lat_ref_dr, lon_ref_dr,
    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;


float lat_ref{-1000}, lon_ref{-1000};
unsigned int ref_gen{1};

XPLMCommandRef dock_cmdr, undock_cmdr, toggle_cmdr, toggle_ui_cmdr;
static int pref_auto_mode;

float now;            // current timestamp
std::string base_dir; // base directory of openSAM

std::map<std::string, DoorInfo> door_info_map;

unsigned long long stat_sc_far_skip, stat_far_skip, stat_near_skip,
    stat_acc_called, stat_jw_match, stat_dgs_acc, stat_dgs_acc_last,
    stat_anim_acc_called, stat_auto_drf_called;

XPLMProbeInfo_t probeinfo;
XPLMProbeRef probe_ref;
XPLMMenuID anim_menu;

static void
save_pref()
{
    FILE *f = fopen(pref_path.c_str(), "w");
    if (NULL == f)
        return;
    pref_auto_mode = my_plane->auto_mode();

    // encode southern hemisphere with negative season
    int s = nh ? season : -season;
    fprintf(f, "%d,%d,%d", auto_season, s, pref_auto_mode);
    fclose(f);

    log_msg("Saving pref auto_season: %d, season: %d, auto_select_jws: %d", auto_season, s, pref_auto_mode);
}

static void
load_pref()
{
    // set some reasonable default values in case there is no pref file
    nh = 1;
    auto_season = 1;
    season = 1;
    pref_auto_mode = 1;

    FILE *f  = fopen(pref_path.c_str(), "r");
    if (NULL == f)
        return;

    fscanf(f, "%i,%i,%i", &auto_season, &season, &pref_auto_mode);
    log_msg("From pref: auto_season: %d, seasons: %d, auto_select_jws: %d",
            auto_season,  season, pref_auto_mode);

    fclose(f);

    if (season < 0) {
        nh = 0;
        season = -season;
    }
}


// Accessor for the "opensam/SAM_Library_installed" dataref
static int
sam_lib_installed_acc(void *ref)
{
    UNUSED(ref);
    return sam_library_installed;
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
cmd_activate_cb([[maybe_unused]] XPLMCommandRef cmdr,
                XPLMCommandPhase phase, [[maybe_unused]] void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd manually_activate");
    dgs_set_active();
    return 0;
}

static int
cmd_toggle_ui_cb([[maybe_unused]]XPLMCommandRef cmdr,
                 XPLMCommandPhase phase, [[maybe_unused]] void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd toggle_ui");
    toggle_ui();
    return 0;
}

static float
flight_loop_cb([[maybe_unused]] float inElapsedSinceLastCall,
               [[maybe_unused]] float inElapsedTimeSinceLastFlightLoop, [[maybe_unused]] int inCounter,
               [[maybe_unused]] void *inRefcon)
{
    static float jw_next_ts, dgs_next_ts, anim_next_ts;

    now = XPLMGetDataf(total_running_time_sec_dr);

    bool on_ground_prev = my_plane->on_ground();
    my_plane->update();
    bool on_ground = my_plane->on_ground();

    // check for transition
    if (on_ground != on_ground_prev) {
        if (on_ground)
            dgs_set_active();
        else
            dgs_set_inactive();
    }

    float jw_loop_delay = jw_next_ts - now;
    float dgs_loop_delay = dgs_next_ts - now;
    float anim_loop_delay = anim_next_ts - now;

    if (! my_plane->is_helicopter_) {
        if (jw_loop_delay <= 0.0f) {
            jw_loop_delay = my_plane->jw_state_machine();
            jw_next_ts = now + jw_loop_delay;
        }

        if (dgs_loop_delay <= 0.0f) {
            dgs_loop_delay = dgs_state_machine();
            dgs_next_ts = now + dgs_loop_delay;
        }
    }

    if (anim_loop_delay <= 0.0f) {
        anim_loop_delay = anim_state_machine();
        anim_next_ts = now + anim_loop_delay;
    }

    return std::min(anim_loop_delay, std::min(jw_loop_delay, dgs_loop_delay));
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
menu_cb([[maybe_unused]] void *menu_ref, void *item_ref)
{
    int entry = (long long)item_ref;

    if (entry == 4) {
        auto_season = !auto_season;
        set_season_auto();
    } else {
        season = entry;
        auto_season = 0;    // selecting a season always goes to manual mode
    }

    set_menu();
    save_pref();
}

// dock/undock command
static int
cmd_dock_jw_cb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("cmd_dock_jw_cb called");

    if (ref == NULL)
        my_plane->request_dock();
    else if (ref == (void *)1)
        my_plane->request_undock();
    else if (ref == (void *)2)
        my_plane->request_toggle();

    return 0;
}

// intercept XP12's standard cmd
static int
cmd_xp12_dock_jw_cb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase,
                    [[maybe_unused]] void *ref)
{
    if (xplm_CommandBegin != phase)
        return 1;

    log_msg("cmd_xp12_dock_jw_cb called");

    my_plane->request_toggle();
    return 1;       // pass on to XP12, likely there is no XP12 jw here 8-)
}

static void
load_door_info(const std::string& fn)
{
    std::ifstream f(fn);
    if (!f.is_open())
        throw OsEx("Error loading " + fn);

    log_msg("Building door_info_map from %s",  fn.c_str());

    std::string line;
    while (std::getline(f, line)) {
        size_t i = line.find('\r');
        if (i != std::string::npos)
            line.resize(i);

        char icao[5];
        int d;
        float x, y, z;
        if (5 == sscanf(line.c_str(), "%4s %d %f %f %f", icao, &d, &x, &y, &z)) {
            if (icao[0] == '#')
                continue;

            if (d < 1 || d > kMaxDoor) {
                log_msg("invalid entry: '%s'", line.c_str());
                continue;
            }

            char c = d + '0';
            DoorInfo d{x, y, z};
            door_info_map[std::string(icao) + c] = d;
        }
    }
}

// =========================== plugin entry points ===============================================
PLUGIN_API int
XPluginStart(char *out_name, char *out_sig, char *out_desc)
{
    log_msg("Startup " VERSION);

    probeinfo.structSize = sizeof(XPLMProbeInfo_t);
    strcpy(out_name, "openSAM " VERSION);
    strcpy(out_sig, "openSAM.hotbso");
    strcpy(out_desc, "A plugin that emulates SAM");

    // Always use Unix-native paths on the Mac!
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

    char buffer[2048];
	XPLMGetSystemPath(buffer);
    xp_dir = std::string(buffer);

    // set pref path
    XPLMGetPrefsPath(buffer);
    XPLMExtractFileAndPath(buffer);
    pref_path = std::string(buffer) + "/openSAM.prf";

    // set plugin's base dir
    base_dir = xp_dir + "Resources/plugins/openSAM/";

    // collect all config and *.xml files
    try {
        load_door_info(base_dir + "acf_door_position.txt");
        SceneryPacks scp(xp_dir);
        sam_library_installed = scp.SAM_Library_path.size() > 0;
        collect_sam_xml(scp);
        log_msg("%d sceneries with sam jetways found", (int)sceneries.size());
        JwCtrl::sound_init();
    } catch (const OsEx& ex) {
        log_msg("fatal error: '%s', bye!", ex.what());
        return 0;   // bye
    }

    date_day_dr = XPLMFindDataRef("sim/time/local_date_days");

    lat_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lat_ref");
    lon_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lon_ref");

    draw_object_x_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_x");
    draw_object_y_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_y");
    draw_object_z_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_z");
    draw_object_psi_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_psi");

    total_running_time_sec_dr = XPLMFindDataRef("sim/time/total_running_time_sec");
    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");

    load_pref();

    probe_ref = XPLMCreateProbe(xplm_ProbeY);
    if (NULL == probe_ref) {
        log_msg("Can't create terrain probe");
        return 0;
    }

    // If commands or dataref accessors are already registered it's to late to
    // fail XPluginStart as the dll is unloaded and X-Plane crashes.
    // So from here on we are doomed to succeed.

    XPLMRegisterDataAccessor("opensam/SAM_Library_installed", xplmType_Int, 0, sam_lib_installed_acc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL);

    // create the seasons datarefs
    for (int i = 0; i < 4; i++)
        XPLMRegisterDataAccessor(dr_name[i], xplmType_Int, 0, read_season_acc,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, (void *)(long long)i, NULL);

    MyPlane::init();
    my_plane->auto_mode_set(pref_auto_mode);
    jw_init();
    JwCtrl::init();
    dgs_init();
    anim_init();

    // own commands
    XPLMCommandRef activate_cmdr = XPLMCreateCommand("openSAM/activate", "Manually activate searching for DGS");
    XPLMRegisterCommandHandler(activate_cmdr, cmd_activate_cb, 0, NULL);

    XPLMCommandRef toggle_ui_cmdr = XPLMCreateCommand("openSAM/toggle_ui", "Toggle UI");
    XPLMRegisterCommandHandler(toggle_ui_cmdr, cmd_toggle_ui_cb, 0, NULL);

    dock_cmdr = XPLMCreateCommand("openSAM/dock_jwy", "Dock jetway");
    XPLMRegisterCommandHandler(dock_cmdr, cmd_dock_jw_cb, 0, (void *)0);

    undock_cmdr = XPLMCreateCommand("openSAM/undock_jwy", "Undock jetway");
    XPLMRegisterCommandHandler(undock_cmdr, cmd_dock_jw_cb, 0, (void *)1);

    toggle_cmdr = XPLMCreateCommand("openSAM/toggle_jwy", "Toggle jetway");
    XPLMRegisterCommandHandler(toggle_cmdr, cmd_dock_jw_cb, 0, (void *)2);

    // augment XP12's standard cmd
    XPLMCommandRef xp12_toggle_cmdr = XPLMFindCommand("sim/ground_ops/jetway");
    if (xp12_toggle_cmdr)
        XPLMRegisterCommandHandler(xp12_toggle_cmdr, cmd_xp12_dock_jw_cb, 1, NULL);

    // build menues
    XPLMMenuID menu = XPLMFindPluginsMenu();
    XPLMMenuID os_menu = XPLMCreateMenu("openSAM", menu,
                                        XPLMAppendMenuItem(menu, "openSAM", NULL, 0),
                                        NULL, NULL);
    // openSAM
    XPLMAppendMenuItemWithCommand(os_menu, "Dock Jetway", dock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Undock Jetway", undock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Toggle UI", toggle_ui_cmdr);
    XPLMAppendMenuSeparator(os_menu);

    // openSAM -> Remote control
    int rc_menu_item = XPLMAppendMenuItem(os_menu, "Remote Control", NULL, 0);
    anim_menu = XPLMCreateMenu("Remote Control", os_menu, rc_menu_item, anim_menu_cb, NULL);

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

    set_menu();

    // ... and off we go
    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;

#if 0
    // keep in case we need it later
  fail:
    log_msg("init failure, can't enable openSAM");
    init_fail = 1;
    return 1;
#endif
}


PLUGIN_API void
XPluginStop(void)
{
}


PLUGIN_API void
XPluginDisable(void)
{
    if (probe_ref)
        XPLMDestroyProbe(probe_ref);

    save_pref();
    log_msg("acc called:               %llu", stat_acc_called);
    log_msg("scenery far skip:         %llu", stat_sc_far_skip);
    log_msg("far skip:                 %llu", stat_far_skip);
    log_msg("near skip:                %llu", stat_near_skip);
    log_msg("dgs acc called:           %llu", stat_dgs_acc);
    log_msg("last_dgs acc:             %llu", stat_dgs_acc_last);
    log_msg("stat_anim_acc_called:     %llu", stat_anim_acc_called);
    log_msg("stat_auto_drf_called:     %llu", stat_auto_drf_called);
}


PLUGIN_API int
XPluginEnable(void)
{
    if (init_fail)  // once and for all
        return 0;

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
        nh = (my_plane->lat() >= 0.0);
        set_season_auto();
        return;
    }

    // my plane loaded
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        my_plane->plane_loaded();
        return;
    }
    // livery loaded
    if (in_msg == XPLM_MSG_LIVERY_LOADED && in_param == 0) {
        my_plane->livery_loaded();
        return;
    }
}
