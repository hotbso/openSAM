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

#include <numbers>

#include "XPLMGraphics.h"

#include "opensam.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "jetway.h"
#include "log_msg.h"
#include "flat_earth_math.h"
namespace fem = flat_earth_math;

// from ReadWav.cpp
extern void ReadWav(const std::string& fname, Sound& sound);

Sound Jetway::alert_;
quadtree::LLQuadTree<double, Jetway, kMaxJwPerNode> jw_quadtree;
std::vector<Jetway*> jw_list;

bool Jetway::Lock(int pid) noexcept {  // -> whether lock could be aquired
    if (locked_ > 0 && lock_pid_ != pid) {
        LogMsg("pid={:03}, failed to lock jw '{}', already locked_ by pid={:03}", pid, name_, lock_pid_);
        return false;
    }

    if (locked_ == 0)
        LogMsg("pid={:03}, locking jw '{}'", pid, name_);

    locked_++;
    lock_pid_ = pid;
    return true;
}

void Jetway::Unlock() noexcept {
    locked_--;
    if (locked_ == 0) {
        LogMsg("pid={:03}, unlocking jw '{}'", lock_pid_, name_);
        lock_pid_ = 0;
    } else if (locked_ < 0) {
        LogMsg("jw '{}' unlocked too many times, locked_: {}", name_, locked_);
        locked_ = 0;
        lock_pid_ = -1;
    }
}

void Jetway::SetWheelsStairs(bool force_high_precision) {
    // low precision
    wheels_ = std::tan(rotate3_ * kD2R) * (wheel_pos_ + extent_);

    // Try terrain adjustment for the wheels
    // As long as XPLMProbeTerrainXYZ supports only float this has limited success
    // XPLMProbeTerrainXYZ is costly so throttle that by only probing when the wheel position has moved

    if (obj_ref_gen_ != ref_gen)
        return;

    static uint_fast8_t throttle;  // global for all moving jetways

    if (force_high_precision || (throttle++ & 0x8)) {
        double w_x = x_ + (extent_ + wheel_pos_) * std::cos((rotate1_ + psi_ - 90.0f) * kD2R);
        double w_z = z_ + (extent_ + wheel_pos_) * std::sin((rotate1_ + psi_ - 90.0f) * kD2R);

        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, w_x, y_, w_z, &probeinfo)) {
            LogMsg("SetWheelsStairs terrain probe failed???");
            wheels_adjust_ = 0.0f;
        } else {
            wheels_adjust_ = probeinfo.locationY - y_;  // wheels relative to tunnel
            // LogMsg("SetWheelsStairs: {} wheels_adjust_ = {:f}", name_, wheels_adjust_);
        }
    }

    wheels_ -= wheels_adjust_;

    if (stair_length_ == 0.0f)
        return;

    // Get the height from the ground of the stair pivot point
    float stair_pivot_height = std::sin(rotate3_ * kD2R) * (stair_pos_ + extent_);  //Distance ABOVE the jetway floor
    stair_pivot_height += height_;

    // Terrain a jetway is on may be sloped, but most likely, it will be linear between the wheel pos and origin.
    // This code effectively gets what % of the way from the origin to the wheels the stair ground contact is, and multiplies the wheel offset by that amount, aka interpolating the terrain offset
    float stair_contact_dist_from_pivot = std::cos(stairrotate_ * kD2R) * stair_length_;
    float stair_contact_from_origin = extent_ + stair_pos_ - stair_contact_dist_from_pivot;
    float wheel_from_origin = extent_ + wheel_pos_;
    float stair_extent_to_wheel_extent_rat = stair_contact_from_origin / wheel_from_origin;
    stair_pivot_height -= (wheels_adjust_ * stair_extent_to_wheel_extent_rat);
    // LogMsg("stair_pivot_height: {:.3f}, stair_extent_to_wheel_extent_rat: {:.3f}", stair_pivot_height,
    //     stair_extent_to_wheel_extent_rat);
    stairrotate_ = (std::asin(std::clamp(stair_pivot_height / std::abs(stair_length_), -1.0f, 1.0f)) / kD2R) - rotate3_;
}

void Jetway::Reset() {
    AlertOff();
    locked_ = 0;
    lock_pid_ = -1;
    rotate1_ = initial_rot1_;
    rotate2_ = initial_rot2_;
    rotate3_ = initial_rot3_;
    extent_ = initial_extent_;
    SetWheelsStairs();
    warnlight_ = 0;
    canopy_ = 0;
}

void Jetway::AlertComplete(void* ref, [[maybe_unused]] FMOD_RESULT status) {
    Jetway* jw = reinterpret_cast<Jetway*>(ref);
    jw->alert_chn_ = nullptr;
}

void Jetway::AlertOn() {
    if (alert_chn_)
        return;
    alert_chn_ = XPLMPlayPCMOnBus(alert_.data, alert_.size, FMOD_SOUND_FORMAT_PCM16, alert_.sample_rate,
                                  alert_.num_channels, 1, xplm_AudioExteriorEnvironment, AlertComplete, this);

    AlertSetpos();
    XPLMSetAudioVolume(alert_chn_, 2.0f);
    XPLMSetAudioFadeDistance(alert_chn_, 10.0f, 10000.0f);
}

void Jetway::AlertOff() {
    if (alert_chn_) {
        // beware: this is asynchronous, so the AlertComplete callback that clears alert_chn_ will be called later
        XPLMStopAudio(alert_chn_);
    }
}

void Jetway::AlertSetpos() {
    if (alert_chn_ == nullptr)
        return;

    static FMOD_VECTOR vel = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR pos;

    // the beeper position is roughly at the cabin door_

    // compute position in the local coordinate system of the scenery
    float rot1 = fem::RA((rotate1_ + psi_) - 90.0f);
    pos.x = x_ + (extent_ + cabin_pos_) * std::cos(rot1 * kD2R);
    pos.y = y_ + height_;
    pos.z = z_ + (extent_ + cabin_pos_) * std::sin(rot1 * kD2R);
    XPLMSetAudioPosition(alert_chn_, &pos, &vel);
}

// static
void Jetway::SoundInit() {
    // load alert sound
    ReadWav(base_dir + "sound/alert.wav", alert_);
    if (alert_.data)
        LogMsg("alert sound loaded, channels: {}, bit_rate: {}, size: {}", alert_.num_channels, alert_.sample_rate,
               alert_.size);
    else
        throw std::runtime_error("Could not load sound");
}

// static
void Jetway::Initialize() {
    for (auto jw : jw_list)
        jw->Reset();
}

// static
void Jetway::Finalize() {
#if 1
    // dump the quadtree and the list of jetways for debugging
    jw_quadtree.Dump();
    for (int i = 0; i < (int)jw_list.size(); i++) {
        Jetway* jw = jw_list[i];
        LogMsg("jw[{}]: '{}', ll: ({:.6}, {:.6}), is_undefined: {}, local: x: {:5.3f}, z: {:5.3f}, y: {:5.3f}, psi: {:4.1f}", i,
               jw->name_, jw->latitude_, jw->longitude_, jw->is_undefined_, jw->x_, jw->z_, jw->y_, jw->psi_);
    }
#endif
}
