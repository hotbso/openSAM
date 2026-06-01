//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
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

#include <cmath>
#include <numbers>
#include <string>
#include <vector>
#include <unordered_map>

#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"

#include "dgs/apt_airport.h"
#include "log_msg.h"

static constexpr float kD2R = std::numbers::pi/180.0;
static constexpr float kF2M = 0.3048;                   // 1 ft [m]
static constexpr float kLat2M = 111120;                 // 1° lat in m

// DGS types per stand, AutoDGS mode
static constexpr int kMarshaller = 0;
static constexpr int kVDGS = 1;
static constexpr int kAutomatic = 2;

enum VDgsType {
    kVdgsSafedock_T2_24,
    kVdgsSafedock_X
};

extern int default_vdgs_type;

typedef enum { MODE_AUTO, MODE_MANUAL } opmode_t;
extern const char * const opmode_str[];
extern opmode_t operation_mode;

// forwards
struct Stand;
struct SamObj;
struct SamJw;
struct SceneryPacks;
class SamAnim;

static float RA(float angle);

struct Scenery {
    // Not copyable or movable
    Scenery(const Scenery&) = delete;
    Scenery& operator=(const Scenery&) = delete;

    std::string name;
    std::string arpt_icao;

    std::vector<SamJw*> sam_jws;
    std::vector<SamObj*> sam_objs;
    std::vector<SamAnim*> sam_anims;

    float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for kFarSkip */

    Scenery() {
        sam_jws.reserve(100);
        sam_objs.reserve(50);  sam_anims.reserve(50);
    }

    bool InBbox(float lat, float lon) const{
        return (lat >= bb_lat_min && lat <= bb_lat_max
            && RA(lon - bb_lon_min) >= 0 && RA(lon - bb_lon_max) <= 0);
    }
};

extern std::vector<Scenery *> sceneries;

// a poor man's factory for creating sceneries
extern void CollectSamXml(const SceneryPacks &scp);

struct SceneryPacks {
    std::string openSAM_Library_path;
    std::string SAM_Library_path;

    std::vector<std::string> sc_paths;

    SceneryPacks(const std::string& xp_dir);
};

static constexpr int kMaxDoor = 3;
struct DoorInfo {
    float x, y, z;
};

// key is icao + <door num in ascii>
extern std::unordered_map<std::string, DoorInfo> csl_door_info_map;
// key is icao or iata -> icao
extern std::unordered_map<std::string, std::string> acf_generic_type_map;

extern std::string xp_dir;
extern std::string base_dir;        // base directory of openSAM
extern std::string sys_cfg_dir;
extern std::string user_cfg_dir;

extern XPLMDataRef lat_ref_dr, lon_ref_dr,
    draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr,
    total_running_time_sec_dr, sin_wave_dr,  acf_cg_y_dr, acf_cg_z_dr,
    vr_enabled_dr, plane_x_dr, plane_y_dr, plane_z_dr, plane_elevation_dr, plane_true_psi_dr, parkbrake_dr;

extern XPLMCommandRef toggle_jetway_cmdr;

extern unsigned long long stat_sc_far_skip, stat_near_skip,
    stat_acc_called, stat_jw_match, stat_dgs_acc, stat_dgs_acc_last,
    stat_anim_acc_called, stat_auto_drf_called,
    stat_jw_cache_hit;

extern float now;           // current timestamp
extern bool error_disabled; // set this on severe errors to disable openSAM and hopefully allow XP to continue

// detect shifts of the reference frame
extern float lat_ref, lon_ref;
// generation # of reference frame
// init with 1 so jetways never seen by the accessor won't be considered in JwCtrl::FindNearestJetways()
extern unsigned int ref_gen;

// terrain probe
extern XPLMProbeInfo_t probeinfo;
extern XPLMProbeRef probe_ref;

// functions
extern void create_api_drefs();
extern void CheckRefFrameShift();

#define BETWEEN(x, a, b) ((a) <= (x) && (x) <= (b))

// normalize angle to a relative angle in (-180,180]
static inline float RA(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        return angle - 360.0f;

    if (angle <= -180.0f)
        return angle + 360.0f;

    return angle;
}
