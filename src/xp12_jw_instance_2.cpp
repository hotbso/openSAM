//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
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
#include <cstring>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#include "XPLMScenery.h"
#include "XPLMGraphics.h"

#include "opensam.h"
#include "my_plane.h"
#include "xp12_jw_instance.h"
#include "quadtree.inl"
#include "log_msg.h"

static constexpr float kRefreshInterval = 15.0f;  // refresh interval in seconds
static constexpr int kNStyle = 4;
static constexpr int kNLength = 4;

static XPLMObjectRef jetway_model_refs[kNStyle][kNLength];

static const char* jetway_model_paths[kNStyle][kNLength] = {
    {"jetway_1s_3t_11-23.obj", "jetway_1s_3t_14-29.obj", "jetway_1s_3t_17-38.obj", "jetway_1s_3t_20-47.obj"},
    {"jetway_1g_3t_11-23.obj", "jetway_1g_3t_14-29.obj", "jetway_1g_3t_17-38.obj", "jetway_1g_3t_20-47.obj"},
    {"jetway_2s_3t_11-23.obj", "jetway_2s_3t_14-29.obj", "jetway_2s_3t_17-38.obj", "jetway_2s_3t_20-47.obj"},
    {"jetway_2g_3t_11-23.obj", "jetway_2g_3t_14-29.obj", "jetway_2g_3t_17-38.obj", "jetway_2g_3t_20-47.obj"},
};

enum JetwayAnimDref {
    kBaseRotation,
    kTunnelPitch,
    kTunnelExtension,
    kCabinRotation,
    kBogieElevation,
    kBogieRotation,
    kBogieTilt,
    kWheelLeft,
    kWheelRight,
    kStairsAngle,
    kStairsBogieAngle,
    kIsMoving,
    kDrefCount
};

static const char* jetway_anim_drefs[] = {
    "sim/graphics/animation/jetways/jw_base_rotation",
    "sim/graphics/animation/jetways/jw_tunnel_pitch",
    "sim/graphics/animation/jetways/jw_tunnel_extension",
    "sim/graphics/animation/jetways/jw_cabin_rotation",
    "sim/graphics/animation/jetways/jw_bogie_elevation",    // + = up
    "sim/graphics/animation/jetways/jw_bogie_rotation",
    "sim/graphics/animation/jetways/jw_bogie_bogie_tilt",
    "sim/graphics/animation/jetways/jw_wheel_left",
    "sim/graphics/animation/jetways/jw_wheel_right",
    "sim/graphics/animation/jetways/jw_stairs_angle",       // relative to tunnel , + = up
    "sim/graphics/animation/jetways/jw_stairs_bogie_angle",
    "sim/graphics/animation/jetways/jw_is_moving",
    nullptr
};

static std::unordered_map<XP12JwInstance*, bool> active_jws;

void XP12JwInstance::Initialize() {
    LogMsg("Initializing XP12 jetway instance");
    for (int i = 0; i < kNStyle; ++i) {
        for (int j = 0; j < kNLength; ++j) {
            std::string model_path = "Resources/default scenery/airport scenery/Ramp_Equipment/jetways/" +
                                     std::string(jetway_model_paths[i][j]);
            jetway_model_refs[i][j] = XPLMLoadObject(model_path.c_str());
            if (jetway_model_refs[i][j] == nullptr)
                throw std::runtime_error("Failed to load jetway model: " + model_path);
            LogMsg("Successfully loaded jetway model: {}", model_path);
        }
    }

    active_jws.reserve(300);
}

void XP12JwInstance::Finalize() {
    LogMsg("Finalizing XP12 jetway instance");
}

static double last_altitude = 200.0;        // the next jw will use this as the initial altitude estimate

bool XP12JwInstance::UpdateDrawinfo() noexcept {
    if (obj_ref_gen_ == ref_gen)
        return false;

    if (altitude_is_estimate_) {
        altitude_is_estimate_ = false;
        XPLMWorldToLocal(latitude_, longitude_, last_altitude, &x_, &y_, &z_);

        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x_, y_, z_, &probeinfo)) {
            LogMsg("terrain probe 1 failed???");
        }

        // now a terrain probe with better altitude estimate
        double dummy1, dummy2;
        XPLMLocalToWorld(x_, probeinfo.locationY, z_, &dummy1, &dummy2, &altitude_);
        XPLMWorldToLocal(latitude_, longitude_, altitude_, &x_, &y_, &z_);
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x_, y_, z_, &probeinfo)) {
            LogMsg("terrain probe 2 failed???");
        }

        y_ = probeinfo.locationY;
        XPLMLocalToWorld(x_, y_, z_, &dummy1, &dummy2, &altitude_);
    }

    XPLMWorldToLocal(latitude_, longitude_, altitude_, &x_, &y_, &z_);

    obj_ref_gen_ = ref_gen;

    drawinfo_.x = x_;
    drawinfo_.y = y_;
    drawinfo_.z = z_;

    SetWheelsStairs(/* force_high_precision */ true);
    last_altitude = altitude_;
    return true;
}

void XP12JwInstance::CreateInstance() {
    bool changed = UpdateDrawinfo();

    if (instance_ref_ == nullptr) {
        assert(style_code_ < kNStyle && length_code_ < kNLength);
        XPLMObjectRef obj = jetway_model_refs[style_code_][length_code_];
        assert(obj != nullptr);

        instance_ref_ = XPLMCreateInstance(obj, jetway_anim_drefs);
        LogMsg("Creating XP12 jetway instance, ll: ({:.5f},{:.5f})", latitude_, longitude_);
        active_jws[this] = true;
        Reset();
        changed = true;
#if 0
    LogMsg(
        "XP12JwInstance {} instanced at ll: ({},{}), psi: {}, style: {}, length_code: {}, initial_extent: {}, "
        "intial_rot2: {}",
        name_, latitude_, longitude_, heading_, style_code_, length_code_, initial_extent_, initial_rot2_);
#endif
    }

    if (changed)
        UpdateInstance();
}

void XP12JwInstance::RemoveInstance() {
    LogMsg("Removing XP12 jetway instance, ll: ({:.5f},{:.5f})", latitude_, longitude_);
    if (instance_ref_ != nullptr) {
        XPLMDestroyInstance(instance_ref_);
        instance_ref_ = nullptr;
    }
}

void XP12JwInstance::UpdateInstance() {
    // LogMsg("Updating XP12 jetway instance");
    if (instance_ref_ == nullptr) {
        LogMsg("Updating XP12 jetway instance but instance_ref_ is null");
        return;
    }

    float drefs[kDrefCount];
    memset(drefs, 0, sizeof(drefs));

    drefs[kBaseRotation] = rotate1_;
    drefs[kCabinRotation] = rotate2_;
    drefs[kTunnelPitch] = -rotate3_;
    drefs[kTunnelExtension] = extent_;
    drefs[kBogieRotation] = wheelrotatec_;

    // there is a fundamental difference between positive and negative rotate3 that I don't understand
    // and the -0.04f is an empirical ad-hoc adjustment to account for that difference
    drefs[kBogieElevation] = -wheels_;
    if (rotate3_ <= -1.5f)
        drefs[kBogieElevation] -= 0.04f;

    drefs[kBogieTilt] = -rotate3_ * std::sin(wheelrotatec_ * kD2R);
    drefs[kWheelLeft] = wheelrotatel_;
    drefs[kWheelRight] = wheelrotater_;
    drefs[kStairsAngle] = -stairrotate_;
    drefs[kStairsBogieAngle] = stairrotate_ + rotate3_;

    #if 0
    LogMsg(
        "Updating XP12 jetway instance with drefs: base_rot: {:.1f}, cabin_rot: {:.1f}, tunnel_pitch: {:.1f}, tunnel_ext: "
        "{:.1f}, bogie_rot: {:.1f}, bogie_elev: {:.1f}, stairs_angle: {:.1f}",
        drefs[kBaseRotation], drefs[kCabinRotation], drefs[kTunnelPitch], drefs[kTunnelExtension],
        drefs[kBogieRotation], drefs[kBogieElevation], drefs[kStairsAngle]);
    #endif

    XPLMInstanceSetPositionDouble(instance_ref_, &drawinfo_, drefs);
}

static float last_refresh_ts = -1000.0f;
static unsigned int refresh_ref_gen;

// called every frame to update instances around the given location
void XP12JwInstance::InstanceAround(float lat, float lon, float distance) {
    // if very high remove everything
    float y_agl = my_plane->y_agl();
    if (y_agl > distance + 500.0f) {
        RemoveAll();
        return;
    }

    // in a band below that do nothing in order to avoid oscillations
    if (y_agl > distance - 100.0f)
        return;

    CheckRefFrameShift();

    if ((now > last_refresh_ts + kRefreshInterval) || (refresh_ref_gen < ref_gen)) {
        last_refresh_ts = now;
        refresh_ref_gen = ref_gen;

        quadtree::Box<double> search_box(lon, lat, distance);
        std::unordered_map<SamJw*, bool> found_map =
            jw_quadtree.FindInBox(search_box, [](const SamJw* jw) { return jw->class_code() == SamJw::kXP12Jw; });

        for (auto& [jw, _] : found_map) {
            assert(jw->class_code() == SamJw::kXP12Jw);
            XP12JwInstance* xp12_jw = static_cast<XP12JwInstance*>(jw);
            xp12_jw->CreateInstance();
        }

        // remove instances that are no longer in the active zone
        for (auto it = active_jws.begin(); it != active_jws.end();) {
            if (!found_map.contains(it->first)) {
                it->first->RemoveInstance();
                it = active_jws.erase(it);
            } else
                ++it;
        }

        LogMsg("Showing XP12 jetway instances around ll: ({},{}), distance: {}, instances: {}", lat, lon, distance,
               (int)active_jws.size());
    }
}

void XP12JwInstance::RemoveAll() {
    for (auto& [xp12_jw, _] : active_jws)
        xp12_jw->RemoveInstance();

    active_jws.clear();
}
