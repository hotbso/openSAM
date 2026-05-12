//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <filesystem>

#include "opensam.h"
#include "plane.h"
#include "samjw.h"
#include "jwctrl.h"
#include "os_anim.h"
#include "my_plane.h"
#include "mpadapter.h"
#include "opensam_airport.h"
#include "autodgs_airport.h"
#include <dgs/plane.h>
#include <sam1_dgs.h>

#include "flat_earth_math.h"
#include "seasons.h"

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

#include "version.h"

namespace fem = flat_earth_math;

//
// C++ style:
// This code loosely follows Google's style: https://google.github.io/styleguide/cppguide.html
//
//------------------------------------------------------------------------------------
//
// On the various coordinate systems and angles:
//
// Objects are drawn in a +x east , -z true north, +y up system (OpenGL standard).
// Headings (hdgt) are measured from -z (=true north) right turning.
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
// So first of all we rotate and shift everything into the 'plane' frame i.e.
// acf nose pointing up to -z, door at (0, some z, some y), jetways to the left somewhere at -x and some z.
//
// These values are kept in the "active_jw" object.
//
// Then we do our math in the plane frame and transform everything back to the jetway frame
// to get the rotation angles.
//
// The datarefs for jetway animation are:
//  rotation1       tunnel relative to the placed object
//  rotation2       cabin relative to the tunnel
//  rotation3       tunnel relative to horizontal (= x-z) plane
//  wheelrotatec    wheel base relative to tunnel around the y-axis
//  wheelrotater    right wheel
//  wheelrotatel    left wheel
//  wheel           delta height in m of tunnel over wheelbase relative to horizontal plane.
//
// Likewise for DGS we xform everything into the stand frame and go from there.
//
///

const char* log_msg_prefix = "opensam: ";

// no multiplayer processing if y_agl > limit
constexpr float kMultiPlayerHeightLimit = 1000.0f;

std::string xp_dir;
std::string sys_cfg_dir;    // <base_dir>/cfg
std::string user_cfg_dir;   // <xp_dir>/Output/openSAM
static std::string pref_path;

opmode_t operation_mode = MODE_AUTO;
const char* const opmode_str[] = { "Automatic", "Manual" };

static XPLMMenuID os_menu;
static int toggle_mp_item;
static const char* toggle_mp_support_txt = "Toggle Multiplayer Support";

static int airport_loaded;
static int sam_library_installed;   // int, is served to a dataref accessor, 0 or 1

XPLMDataRef lat_ref_dr, lon_ref_dr, draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr,
    total_running_time_sec_dr, vr_enabled_dr, plane_x_dr, plane_y_dr, plane_z_dr, plane_elevation_dr,
    plane_true_psi_dr, parkbrake_dr, sin_wave_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr, acf_gear_z_dr,
    eng_running_dr, beacon_dr, is_helicopter_dr;

XPLMCommandRef toggle_jetway_cmdr, activate_cmdr, toggle_ui_cmdr;

float lat_ref{-1000}, lon_ref{-1000};
unsigned int ref_gen{1};

static int pref_auto_mode;

float now;             // current timestamp
std::string base_dir;  // base directory of openSAM

static std::unique_ptr<MpAdapter> mp_adapter;

std::unordered_map<std::string, DoorInfo> csl_door_info_map;
std::unordered_map<std::string, std::string> acf_generic_type_map;

unsigned long long stat_sc_far_skip, stat_near_skip, stat_acc_called, stat_jw_match, stat_dgs_acc, stat_anim_acc_called,
    stat_auto_drf_called, stat_jw_cache_hit;

static XPLMMenuID vdgs_menu;
static constexpr int kNumDgsTypes = 2;        // must match number of items in default_vdgs_item and VdgsMenuCb()
static int default_vdgs_item[kNumDgsTypes];   // menu items
int default_vdgs_type = kVdgsSafedock_T2_24;  // index into default_vdgs_item

// 'emulate' a connection status dataref for XP12's jetway, toggled by intercepting XP12's standard cmd
bool xp12_jw_connected;
float xp12_jw_connected_ts;

XPLMProbeInfo_t probeinfo;
XPLMProbeRef probe_ref;
XPLMMenuID anim_menu;
static fem::LLPos plane_pos, plane_pos_prev;

bool error_disabled;

static float FlightLoopCb(float inElapsedSinceLastCall, [[maybe_unused]] float inElapsedTimeSinceLastFlightLoop, [[maybe_unused]] int inCounter,
                          [[maybe_unused]] void* inRefcon);

static XPLMCreateFlightLoop_t flight_loop_ctx = {sizeof(XPLMCreateFlightLoop_t),
                                                 xplm_FlightLoop_Phase_BeforeFlightModel, FlightLoopCb, nullptr};

static XPLMFlightLoopID flight_loop_id;

static bool pending_plane_loaded_cb = false;  // delayed init

void SavePrefs() {
    FILE* f = fopen(pref_path.c_str(), "w");
    if (NULL == f)
        return;
    pref_auto_mode = my_plane->auto_mode();

    // encode southern hemisphere with negative season
    int s = Seasons::nh ? Seasons::season : -Seasons::season;
    fprintf(f, "%d,%d,%d,%d", Seasons::auto_season, s, pref_auto_mode, default_vdgs_type);
    fclose(f);

    LogMsg("Saving pref auto_season: %d, season: %d, auto_select_jws: %d, default_vdgs_type: %d", Seasons::auto_season,
           s, pref_auto_mode, default_vdgs_type);
}

static void LoadPrefs() {
    // set some reasonable default values in case there is no pref file
    Seasons::nh = 1;
    Seasons::auto_season = 1;
    Seasons::season = 1;
    pref_auto_mode = 1;

    FILE* f = fopen(pref_path.c_str(), "r");
    if (NULL == f)
        return;

    [[maybe_unused]] int n =
        fscanf(f, "%i,%i,%i,%i", &Seasons::auto_season, &Seasons::season, &pref_auto_mode, &default_vdgs_type);
    LogMsg("From pref: auto_season: %d, seasons: %d, auto_select_jws: %d, default_vdgs_type: %d", Seasons::auto_season,
           Seasons::season, pref_auto_mode, default_vdgs_type);

    fclose(f);

    if (Seasons::season < 0) {
        Seasons::nh = 0;
        Seasons::season = -Seasons::season;
    }
}

// Accessor for the "opensam/SAM_Library_installed" dataref
static int SamLibInstalledAcc([[maybe_unused]] void* ref) {
    return sam_library_installed;
}

//------------------------------------------------------------------------------------
void LoadAndActivateAirport() {
    LogMsg("plane is now on the ground, trying to identify airport and stand");

    const dgs::AptAirport* arpt = dgs::AptAirport::LocateAirport(plane_pos);
    if (arpt == nullptr) {
        LogMsg("airport could not be identified at %0.8f,%0.8f", plane_pos.lat, plane_pos.lon);
        os_arpt = nullptr;
        adgs_arpt = nullptr;
    } else {
        LogMsg("now on airport: %s", arpt->icao_.c_str());
        if (arpt->is_opensam_) {
            if (os_arpt == nullptr || os_arpt->name() != arpt->icao_) {  // don't reload same
                os_arpt = std::make_unique<OsAirport>(*arpt);
                if (os_arpt) {
                    LogMsg("airport %s loaded successfully with openSAM personality", os_arpt->name().c_str());
                    os_arpt->SetArrival();
                }
            }
        } else {                                                             // AutoDGS
            if (adgs_arpt == nullptr || adgs_arpt->name() != arpt->icao_) {  // don't reload same
                adgs_arpt = std::make_unique<AdgsAirport>(*arpt);
            }
            if (adgs_arpt) {
                LogMsg("airport %s loaded successfully with AutoDGS personality", adgs_arpt->name().c_str());
                adgs_arpt->SetArrival();
            }
        }
    }
}

static int CmdActivateCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    LogMsg("cmd manually_activate");

    if (!my_plane->on_ground()) {
        LogMsg("plane is not on the ground, can't activate");
        return 0;
    }

    // There is a multitude of possible and timing related interactions between teleportation and on_ground signals
    // First try to set Arrival on an already active airport
    if (os_arpt) {
        os_arpt->SetArrival();
        return 0;
    }

    if (adgs_arpt) {
        adgs_arpt->SetArrival();
        return 0;
    }

    // .. then try to activate one
    LoadAndActivateAirport();
    return 0;
}

static int CmdToggleUICb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    LogMsg("cmd ToggleUI");

    if (os_arpt)
        OsToggleUI();
    else if (adgs_arpt)
        AdgsToggleUI();

    return 0;
}

// multiplayer activation
static int CmdToggleMpCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    LogMsg("cmd toggle_mp");
    if (mp_adapter) {
        mp_adapter = nullptr;
    } else {
        mp_adapter = MpAdapter_factory();
    }

    if (mp_adapter) {
        std::string menu_txt{toggle_mp_support_txt};
        menu_txt += " (";
        menu_txt += mp_adapter->personality();
        menu_txt += ")";
        XPLMSetMenuItemName(os_menu, toggle_mp_item, menu_txt.c_str(), 0);
    } else
        XPLMSetMenuItemName(os_menu, toggle_mp_item, toggle_mp_support_txt, 0);

    XPLMCheckMenuItem(os_menu, toggle_mp_item, mp_adapter ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    return 0;
}

static float FlightLoopCb(float inElapsedSinceLastCall,
                          [[maybe_unused]] float inElapsedTimeSinceLastFlightLoop, [[maybe_unused]] int inCounter,
                          [[maybe_unused]] void* inRefcon) {
    static float jw_next_ts, dgs_next_ts, anim_next_ts, mp_update_next_ts;
    static bool on_ground_prev = false;

    if (error_disabled)
        return 0;

    try {
        if (pending_plane_loaded_cb) {
            pending_plane_loaded_cb = false;
            my_plane->PlaneLoadedCb();
        }

        now = XPLMGetDataf(total_running_time_sec_dr);

        plane_pos_prev = plane_pos;
        plane_pos = fem::LLPos(my_plane->lat(), my_plane->lon());;

        // if we go 3 * supersonic it's a teleportation, e.g. a ToLiss situation reload
        if (fem::len(plane_pos - plane_pos_prev) > inElapsedSinceLastCall * 3.0f * 340.0f) {
            LogMsg("teleportation detected, resetting airport");
            os_arpt = nullptr;
            adgs_arpt = nullptr;
            on_ground_prev = false;  // to trigger airport identification on next loop
            return 5.0f; // let the dust settle
        }

        //LogMsg("FlightLoopCb called, now: %0.2f, on_ground_prev: %d", now, on_ground_prev);
        my_plane->Update();
        bool on_ground = my_plane->on_ground();
        //LogMsg("on_ground: %d", on_ground);

        // check for transition
        if (on_ground != on_ground_prev) {
            if (on_ground)
                LoadAndActivateAirport();
            else {  // airborne
                os_arpt = nullptr;
                adgs_arpt = nullptr;
            }
        }

        on_ground_prev = on_ground;

       if (os_arpt  == nullptr && adgs_arpt == nullptr)
            return 1.0f;  // no airport, nothing to do

        assert(os_arpt == nullptr || adgs_arpt == nullptr);  // can't be both

        // TODO: what if one of these values is not updated below due to if?
        float jw_loop_delay = jw_next_ts - now;
        float dgs_loop_delay = dgs_next_ts - now;
        float anim_loop_delay = anim_next_ts - now;
        float mp_update_delay = mp_update_next_ts - now;

        float my_y_agl = my_plane->y_agl();
        if (my_y_agl < kMultiPlayerHeightLimit && mp_adapter && mp_update_delay <= 0.0f)
            mp_update_next_ts = now + mp_adapter->update();

        if (!my_plane->is_helicopter_) {
            if (jw_loop_delay <= 0.0f) {
                jw_loop_delay = my_plane->JwStateMachine();
                if (my_y_agl < kMultiPlayerHeightLimit && mp_adapter)
                    jw_loop_delay = std::min(jw_loop_delay, mp_adapter->JwStateMachine());

                jw_next_ts = now + jw_loop_delay;
            }

            if (os_arpt) {
                if (dgs_loop_delay <= 0.0f) {
                    dgs_loop_delay = os_arpt->StateMachine();
                    dgs_next_ts = now + dgs_loop_delay;
                }
            }

            if (adgs_arpt) {
                if (dgs_loop_delay <= 0.0f) {
                    dgs_loop_delay = adgs_arpt->StateMachine();
                    dgs_next_ts = now + dgs_loop_delay;
                }
            }
        }

        if (anim_loop_delay <= 0.0f) {
            anim_loop_delay = AnimStateMachine();
            anim_next_ts = now + anim_loop_delay;
        }
        // LogMsg("jw_loop_delay: %0.2f", jw_loop_delay);
        float delay = std::min(anim_loop_delay, std::min(jw_loop_delay, dgs_loop_delay));
        if (delay <= 0.0f)  // negative is # of frames, 0 is stop
            delay = -1.0f;

        return delay;
    } catch (const std::exception& e) {
        LogMsg("FlightLoopCb caught exception: %s, openSAM disabled", e.what());
        error_disabled = true;
        return 0;
    }
}

// dock/undock command
static int CmdDockJwCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    LogMsg("cmd_dock_jw_cb called");

    if (ref == NULL)
        my_plane->RequestDock();
    else if (ref == (void*)1)
        my_plane->RequestUndock();
    else if (ref == (void*)2)
        my_plane->RequestToggle();

    return 0;
}

// intercept XP12's standard cmd
static int CmdXp12DockJwCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase,
                               [[maybe_unused]] void* ref) {
    if (error_disabled)
        return 0;

    if (xplm_CommandBegin != phase)
        return 1;

    LogMsg("cmd_xp12_dock_jw_cb called");
    if (os_arpt)
        my_plane->RequestToggle();

    if (adgs_arpt && adgs_arpt->active_stand_has_xp12_jw()) {
         xp12_jw_connected = !xp12_jw_connected;
         xp12_jw_connected_ts = now;
         LogMsg("XP12 jetway connected: %d", xp12_jw_connected);
    }

    return 1;  // pass on to XP12
}

static int AdgsCmdCycleDgsCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    if (adgs_arpt)
        adgs_arpt->CycleDgsType();
    AdgsUpdateUI();
    return 0;
}

static int AdgsCmdMoveDgsCloserCb([[maybe_unused]] XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void* ref) {
    if (error_disabled || xplm_CommandBegin != phase)
        return 0;

    if (adgs_arpt)
        adgs_arpt->DgsMoveCloser();
    return 0;
}

// emulate a kind of radio buttons
static void VdgsMenuRadioB() {
    XPLMCheckMenuItem(vdgs_menu, default_vdgs_item[default_vdgs_type], xplm_Menu_Checked);
    for (int i = 0; i < kNumDgsTypes; i++)
        if (i != default_vdgs_type)
            XPLMCheckMenuItem(vdgs_menu, default_vdgs_item[i], xplm_Menu_Unchecked);
}

static void VdgsMenuCb([[maybe_unused]] XPLMMenuID menu_ref, void* item_ref) {
    if (error_disabled)
        return;

    int sel = (int)(intptr_t)item_ref;
    if (sel < 0 || sel >= kNumDgsTypes)
        return;

    default_vdgs_type = sel;
    LogMsg("default VDGS type set to %d", default_vdgs_type);
    VdgsMenuRadioB();
}

static void LoadDoorInfo(const std::string& fn, std::unordered_map<std::string, DoorInfo>& di_map) {
    std::ifstream f(fn);
    if (!f.is_open())
        throw std::runtime_error("Error loading " + fn);

    LogMsg("Building door_info_map from %s", fn.c_str());

    std::string line;
    line.reserve(100);

    while (std::getline(f, line)) {
        if (line.empty())
            continue;

        if (line.back() == '\r')
            line.pop_back();

        if (line.empty() || line[0] == '#')
            continue;

        char icao[5];
        int d;
        float x, y, z;
        if (5 == sscanf(line.c_str(), "%4s %d %f %f %f", icao, &d, &x, &y, &z)) {
            if (d < 1 || d > kMaxDoor) {
                LogMsg("invalid entry: '%s'", line.c_str());
                continue;
            }

            char c = d + '0';
            DoorInfo d{x, y, z};
            di_map[std::string(icao) + c] = d;
        }
    }

    LogMsg("%d mappings loaded", (int)di_map.size());
}

static void LoadAcfGenericType(const std::string& fn) {
    std::ifstream f(fn);
    if (!f.is_open())
        throw std::runtime_error("Error loading " + fn);

    LogMsg("Building acf_generic_type_map from %s", fn.c_str());

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty())
            continue;

        if (line.back() == '\r')
            line.pop_back();

        if (line.empty() || line[0] == '#')
            continue;

        char code[5];
        char icao[5];
        if (2 == sscanf(line.c_str(), "%4s %4s", code, icao))
            acf_generic_type_map[std::string(code)] = std::string(icao);

    }

    LogMsg("%d mappings loaded", (int)acf_generic_type_map.size());
}

dgs::EqStatusVal PbbEqStatus() {
    if (adgs_arpt) {
        if (adgs_arpt->active_stand_has_xp12_jw()) {
            if (xp12_jw_connected && now > xp12_jw_connected_ts + 15.0f)  // needs some time to drive the jw
                return dgs::kEqOn;                                        // present and 'on' (connected)
            return dgs::kEqOff;                                           // present, assume off
        }

        return dgs::kEqUnknown;
    }

    if (os_arpt) {
        // 'query' "opensam/jetway/status" by calling the accessor with ref 1

        int s = MyPlane::JwStatusAcc((void*)1);

        if (s == 0)  // no jetway
            return dgs::EqStatusVal::kEqUnknown;

        if (s == 2)  // docked
            return dgs::EqStatusVal::kEqOn;

        // not docked or in transit
        return dgs::EqStatusVal::kEqOff;
    }

    // NOTREACHED, hopefully
    return dgs::kEqUnknown;
}

// =========================== plugin entry points ===============================================
PLUGIN_API int XPluginStart(char* out_name, char* out_sig, char* out_desc) {
    LogMsg("Startup " VERSION);

    strcpy(out_name, "openSAM " VERSION);
    strcpy(out_sig, "openSAM.hotbso");
    strcpy(out_desc, "A plugin that emulates SAM");

    // Always use Unix-native paths on the Mac!
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    char buffer[2048];
    XPLMGetSystemPath(buffer);
    xp_dir = std::string(buffer);

    // set pref path
    XPLMGetPrefsPath(buffer);
    XPLMExtractFileAndPath(buffer);
    pref_path = std::string(buffer) + "/openSAM.prf";

    // set plugin's base dir
    base_dir = xp_dir + "Resources/plugins/openSAM/";
    sys_cfg_dir = base_dir + "cfg/";
    user_cfg_dir = xp_dir + "Output/openSAM/";
    pref_path = user_cfg_dir + "prefs.prf";
    std::string res_dir = base_dir + "resources/";

    std::filesystem::create_directories(user_cfg_dir);

    // collect all config and *.xml files
    try {
        LoadDoorInfo(base_dir + "csl_door_position.txt", csl_door_info_map);
        LoadAcfGenericType(base_dir + "acf_generic_type.txt");

        SceneryPacks scp(xp_dir);
        if (scp.openSAM_Library_path.empty())
             LogMsg("WARNING: openSAM_Library not found!");

        sam_library_installed = scp.SAM_Library_path.size() > 0;

        CollectSamXml(scp);
        LogMsg("%d sceneries with sam jetways found", (int)sceneries.size());
        int n_stands = 0;
        if (!dgs::AptAirport::ParseAptDat(xp_dir + "/Global Scenery/Global Airports/Earth nav data/apt.dat", false, false, true, n_stands)) {
             LogMsg("WARNING: global apt.dat could not be parsed, no DGS support!");
            return 0;
        } else {
            LogMsg("%d stands with DGS found in global apt.dat", n_stands);
        }

        JwCtrl::SoundInit();
    } catch (const std::exception& ex) {
        LogMsg("fatal error: '%s', bye!", ex.what());
        return 0;  // bye
    }

    plane_x_dr = XPLMFindDataRef("sim/flightmodel/position/local_x");
    plane_y_dr = XPLMFindDataRef("sim/flightmodel/position/local_y");
    plane_z_dr = XPLMFindDataRef("sim/flightmodel/position/local_z");
    plane_elevation_dr = XPLMFindDataRef("sim/flightmodel/position/elevation");
    plane_true_psi_dr = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    parkbrake_dr = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");

    lat_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lat_ref");
    lon_ref_dr = XPLMFindDataRef("sim/flightmodel/position/lon_ref");

    draw_object_x_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_x");
    draw_object_y_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_y");
    draw_object_z_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_z");
    draw_object_psi_dr = XPLMFindDataRef("sim/graphics/animation/draw_object_psi");

    total_running_time_sec_dr = XPLMFindDataRef("sim/time/total_running_time_sec");
    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");
    sin_wave_dr = XPLMFindDataRef("sim/graphics/animation/sin_wave_2");

    acf_icao_dr = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    eng_running_dr = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
    beacon_dr = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    acf_cg_y_dr = XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    acf_cg_z_dr = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    acf_gear_z_dr = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    is_helicopter_dr = XPLMFindDataRef("sim/aircraft2/metadata/is_helicopter");

    LoadPrefs();
    if (!dgs::InitDGS(res_dir)) {
        LogMsg("Failed to initialize dgs library");
        return 0;
    }

    // If commands or dataref accessors are already registered it's too late to
    // fail XPluginStart as the dll gets unloaded and X-Plane crashes.
    // So from here on we are doomed to succeed.

    XPLMRegisterDataAccessor("opensam/SAM_Library_installed", xplmType_Int, 0, SamLibInstalledAcc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    Seasons::InitDataRefs();

    // Create my_plane early. Accessors don't check whether my_plane is initialized.
    my_plane = std::make_shared<MyPlane>();
    dgs::plane = std::dynamic_pointer_cast<dgs::Plane>(my_plane);

    dgs::InitSam1Legacy();
    create_api_drefs();

    try {
        my_plane->AutoModeSet(pref_auto_mode);
        JwInit();
        JwCtrl::Init();
        OsAirport::Init();
        AnimInit();
    } catch (const std::exception& e) {
        LogMsg("fatal error during initialization: '%s', bye!", e.what());
        error_disabled = true;
        return 1;  // bye
    }

    // own commands
    XPLMCommandRef activate_cmdr = XPLMCreateCommand("openSAM/activate", "Manually activate searching for DGS");
    XPLMRegisterCommandHandler(activate_cmdr, CmdActivateCb, 0, NULL);

    XPLMCommandRef toggle_ui_cmdr = XPLMCreateCommand("openSAM/ToggleUI", "Toggle UI");
    XPLMRegisterCommandHandler(toggle_ui_cmdr, CmdToggleUICb, 0, NULL);

    XPLMCommandRef dock_cmdr = XPLMCreateCommand("openSAM/dock_jwy", "Dock jetway");
    XPLMRegisterCommandHandler(dock_cmdr, CmdDockJwCb, 0, (void*)0);

    XPLMCommandRef undock_cmdr = XPLMCreateCommand("openSAM/undock_jwy", "Undock jetway");
    XPLMRegisterCommandHandler(undock_cmdr, CmdDockJwCb, 0, (void*)1);

    XPLMCommandRef toggle_cmdr = XPLMCreateCommand("openSAM/toggle_jwy", "Toggle jetway");
    XPLMRegisterCommandHandler(toggle_cmdr, CmdDockJwCb, 0, (void*)2);

    XPLMCommandRef toggle_mp_cmdr = XPLMCreateCommand("openSAM/toggle_multiplayer", "Toggle Multiplayer Support");
    XPLMRegisterCommandHandler(toggle_mp_cmdr, CmdToggleMpCb, 0, NULL);

    // AutoDGS mode
    XPLMCommandRef cycle_dgs_cmdr = XPLMCreateCommand("openSAM/cycle_dgs", "Cycle DGS between Marshaller, VDGS");
    XPLMRegisterCommandHandler(cycle_dgs_cmdr, AdgsCmdCycleDgsCb, 0, NULL);

    XPLMCommandRef move_dgs_closer_cmdr = XPLMCreateCommand("openSAM/move_dgs_closer", "Move DGS closer by 2m");
    XPLMRegisterCommandHandler(move_dgs_closer_cmdr, AdgsCmdMoveDgsCloserCb, 0, NULL);

    // augment XP12's standard cmd
    toggle_jetway_cmdr = XPLMFindCommand("sim/ground_ops/jetway");
    if (toggle_jetway_cmdr)
        XPLMRegisterCommandHandler(toggle_jetway_cmdr, CmdXp12DockJwCb, 1, NULL);

    // build menues
    XPLMMenuID menu = XPLMFindPluginsMenu();
    os_menu = XPLMCreateMenu("openSAM", menu, XPLMAppendMenuItem(menu, "openSAM", NULL, 0), NULL, NULL);
    // openSAM
    XPLMAppendMenuItemWithCommand(os_menu, "Dock Jetway", dock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Undock Jetway", undock_cmdr);
    XPLMAppendMenuItemWithCommand(os_menu, "Toggle UI", toggle_ui_cmdr);
    XPLMAppendMenuSeparator(os_menu);

    // openSAM -> Remote control
    int rc_menu_item = XPLMAppendMenuItem(os_menu, "Remote Control", NULL, 0);
    anim_menu = XPLMCreateMenu("Remote Control", os_menu, rc_menu_item, AnimMenuCb, NULL);

    XPLMAppendMenuSeparator(os_menu);

    XPLMAppendMenuItemWithCommand(os_menu, "Manually activate searching for DGS", activate_cmdr);

    XPLMAppendMenuSeparator(os_menu);

    toggle_mp_item = XPLMAppendMenuItemWithCommand(os_menu, toggle_mp_support_txt, toggle_mp_cmdr);

    XPLMAppendMenuSeparator(os_menu);

    int sub_menu = XPLMAppendMenuItem(os_menu, "AutoDGS", NULL, 1);
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", os_menu, sub_menu, NULL, NULL);

    XPLMAppendMenuItemWithCommand(adgs_menu, "Cycle DGS", cycle_dgs_cmdr);
    XPLMAppendMenuItemWithCommand(adgs_menu, "Move DGS closer by 2m", move_dgs_closer_cmdr);

    int vdgs_menu_item = XPLMAppendMenuItem(adgs_menu, "Default VDGS", NULL, 0);
    vdgs_menu = XPLMCreateMenu("Default VDGS", adgs_menu, vdgs_menu_item, VdgsMenuCb, NULL);
    default_vdgs_item[0] = XPLMAppendMenuItem(vdgs_menu, "Safedock T2-24", (void *)0, 0);
    default_vdgs_item[1] = XPLMAppendMenuItem(vdgs_menu, "Safedock-X", (void *)1, 0);
    VdgsMenuRadioB();

    XPLMAppendMenuSeparator(os_menu);

    Seasons::InitMenu(os_menu);

    flight_loop_id = XPLMCreateFlightLoop(&flight_loop_ctx);
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    // be a good SDK citizen
    // destroy everything that might call SDK functions. Even LogMsg() is a wrapper around a SDK call.
    mp_adapter = nullptr;
    os_arpt = nullptr;
    adgs_arpt = nullptr;
    dgs::plane = nullptr;
    my_plane = nullptr;
    LogMsg("plugin stopped");
}

PLUGIN_API void XPluginDisable(void) {
    if (probe_ref) {
        XPLMDestroyProbe(probe_ref);
        probe_ref = NULL;
    }

    SavePrefs();
    LogMsg("acc called:           %9llu", stat_acc_called);
    LogMsg("scenery far skip:     %9llu", stat_sc_far_skip);
    LogMsg("near skip:            %9llu", stat_near_skip);
    LogMsg("stat_jw_cache_hit     %9llu", stat_jw_cache_hit);
    LogMsg("cache hit rate:       %9.2f %%", 100.0f * stat_jw_cache_hit / (stat_acc_called + 1));
    LogMsg("dgs acc called:       %9llu", stat_dgs_acc);
    LogMsg("stat_anim_acc_called: %9llu", stat_anim_acc_called);
    LogMsg("stat_auto_drf_called: %9llu", stat_auto_drf_called);
    LogMsg("plugin disabled");
}

PLUGIN_API int XPluginEnable(void) {
    if (error_disabled)  // once and for all
        return 0;

    probe_ref = XPLMCreateProbe(xplm_ProbeY);
    if (NULL == probe_ref) {
        LogMsg("Can't create terrain probe");
        return 0;
    }
    stat_sc_far_skip = stat_near_skip = stat_acc_called = stat_jw_match = stat_dgs_acc = stat_anim_acc_called =
        stat_auto_drf_called = stat_jw_cache_hit = 0;

    LogMsg("plugin enabled");
    return 1;
}

PLUGIN_API void XPluginReceiveMessage([[maybe_unused]] XPLMPluginID in_from, long in_msg, void* in_param) {
    if (error_disabled)
        return;

    // my plane loaded
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        LogMsg("plane loaded, resetting airport");
        os_arpt = nullptr;
        adgs_arpt = nullptr;
        XPLMScheduleFlightLoop(flight_loop_id, 0, 0);
        pending_plane_loaded_cb = true;
        XPLMScheduleFlightLoop(flight_loop_id, 15.0, 1);  // let the dust settle
        return;
    }

    // Everything before XPLM_MSG_AIRPORT_LOADED has bogus datarefs.
    //   Anyway it's too late for the current scenery.
    if ((in_msg == XPLM_MSG_AIRPORT_LOADED) || (airport_loaded && (in_msg == XPLM_MSG_SCENERY_LOADED))) {
        airport_loaded = 1;
        Seasons::nh = (my_plane->lat() >= 0.0);
        Seasons::SetAuto();
        return;
    }
}
