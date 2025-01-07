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

#include <cmath>
#include <string>
#include <vector>
#include <map>

#define XPLM200
#define XPLM210
#define XPLM300
#define XPLM301
#define XPLM400

#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#include "XPLMNavigation.h"

#define UNUSED(x) (void)x

static const float D2R = M_PI/180.0;
static const float F2M = 0.3048;	    // 1 ft [m]
static const float LAT_2_M = 111120;    // 1° lat in m

// forwards
class Stand;
class SamObj;
class SamAnim;
class SamJw;
class SceneryPacks;

static float RA(float angle);

class OsEx : std::exception {
    std::string _reason;

  public:
    OsEx(std::string reason) : _reason(reason) {}
    const char* what() const noexcept {
        return  _reason.c_str();
    }
};

class Scenery {
  public:
    // Not copyable or movable
    Scenery(const Scenery&) = delete;
    Scenery& operator=(const Scenery&) = delete;

    char name[52];

    std::vector<SamJw*> sam_jws;
    std::vector<Stand*> stands;
    std::vector<SamObj*> sam_objs;
    std::vector<SamAnim*> sam_anims;

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for FAR_SKIP */

    Scenery() {
        sam_jws.reserve(100); stands.reserve(100);
        sam_objs.reserve(50);  sam_anims.reserve(50);
    }

    auto in_bbox(float lat, float lon) -> bool {
        return (lat >= bb_lat_min && lat <= bb_lat_max
            && RA(lon - bb_lon_min) >= 0 && RA(lon - bb_lon_max) <= 0);
    }
};

extern std::vector<Scenery *> sceneries;

// a poor man's factory for creating sceneries
extern void collect_sam_xml(const SceneryPacks &scp);

class SceneryPacks {
  public:
    std::string openSAM_Library_path;
    std::string SAM_Library_path;

    std::vector<std::string> sc_paths;

    SceneryPacks(const std::string& xp_dir);
};

struct DoorInfo {
    float x, y, z;
};

// key is icao + <door num in ascii>
extern std::map<std::string, DoorInfo> door_info_map;

#define MAX_DOOR 3
extern int n_door;
extern DoorInfo door_info[MAX_DOOR];
extern float plane_nw_z, plane_mw_z, plane_cg_z;   // z value of plane's 0 to fw, mw and cg
extern std::string acf_icao;

extern std::string base_dir;        // base directory of openSAM
extern int use_engine_running;      // instead of beacon, e.g. MD11
extern int dont_connect_jetway;     // e.g. for ZIBO with own ground service
extern int is_helicopter;

extern int beacon_state, beacon_last_pos;   // beacon state, last switch_pos, ts of last switch actions
extern float beacon_off_ts, beacon_on_ts;
extern float parked_x, parked_z;
extern  int parked_ngen;

extern XPLMDataRef date_day_dr,
    plane_x_dr, plane_y_dr, plane_z_dr, plane_lat_dr, plane_lon_dr, plane_elevation_dr,
    plane_true_psi_dr, plane_y_agl_dr, lat_ref_dr, lon_ref_dr,

    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, parkbrake_dr,
    beacon_dr, eng_running_dr, acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr, acf_gear_z_dr,
    acf_door_x_dr, acf_door_y_dr, acf_door_z_dr, acf_livery_path,
    gear_fnrml_dr,
    total_running_time_sec_dr,
    vr_enabled_dr;

extern unsigned long long stat_sc_far_skip, stat_far_skip, stat_near_skip,
    stat_acc_called, stat_jw_match, stat_dgs_acc, stat_dgs_acc_last,
    stat_anim_acc_called, stat_auto_drf_called;

extern float now;           // current timestamp
extern int on_ground;
extern float lat_ref, lon_ref;
// generation # of reference frame
// init with 1 so jetways never seen by the accessor won't be considered in find_dockable_jws()
extern unsigned int ref_gen;

extern XPLMMenuID anim_menu;
extern XPLMCommandRef dock_cmdr, undock_cmdr, toggle_cmdr;
extern int auto_select_jws;

// terrain probe
extern XPLMProbeInfo_t probeinfo;
extern XPLMProbeRef probe_ref;

// functions
extern void log_msg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
extern int check_beacon(void);
extern int check_teleportation(void);

extern void toggle_ui(void);

#define BETWEEN(x ,a ,b) ((a) <= (x) && (x) <= (b))

static inline
float RA(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        return angle - 360.0f;

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

/* norm-2 length */
static inline float
len2f(float x, float y)
{
    return sqrtf(x * x + y * y);
}

