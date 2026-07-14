//
//    openSAM: manage DGS and jetways for X Plane
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

#pragma once

#include <string>

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMScenery.h"

static constexpr float kF2M = 0.3048;                   // 1 ft [m]
static constexpr float kLat2M = 111120;                 // 1° lat in m

extern std::string xp_dir;
extern std::string base_dir;        // base directory of openSAM
extern std::string sys_cfg_dir;
extern std::string user_cfg_dir;

extern XPLMDataRef draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr, total_running_time_sec_dr,
    sin_wave_dr, acf_cg_y_dr, acf_cg_z_dr, vr_enabled_dr, plane_x_dr, plane_y_dr, plane_z_dr, plane_elevation_dr,
    plane_true_psi_dr, parkbrake_dr;

extern XPLMCommandRef toggle_jetway_cmdr;

// emulate the status for XP12 jetways, called by the "opensam/jetway/status" dataref accessor
std::tuple<int, int> GetXP12JwStatus();    // -> (count, status)

extern unsigned long long stat_jw_acc_called, stat_anim_acc_called, stat_auto_drf_called,
    stat_jw_cache_hit, stat_sc_last;

extern float now;           // current timestamp
extern bool error_disabled; // set this on severe errors to disable openSAM and hopefully allow XP to continue

// generation # of reference frame
// init with 1 so jetways never seen by the accessor won't be considered in JwCtrl::FindNearestJetways()
extern unsigned int ref_gen;

// terrain probe
extern XPLMProbeInfo_t probeinfo;
extern XPLMProbeRef probe_ref;

// functions
extern void create_api_drefs();
extern void CheckRefFrameShift();

template<typename T>
bool is_between(T x, T a, T b) {
    return (a <= x && x <= b);
}
