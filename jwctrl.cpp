//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2025  Holger Teutsch
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
#include <cmath>
#include <ctime>
#include <cstring>
#include <algorithm>

#include "openSAM.h"
#include "plane.h"
#include "samjw.h"
#include "jwctrl.h"
#include "os_dgs.h"

// from os_read_wav.c
extern void ReadWav(const std::string& fname, Sound& sound);

static constexpr float kDriveSpeed = 1.0;    // m/s
static constexpr float kTurnSpeed = 10.0;    // °/s
static constexpr float kHeightSpeed = 0.1;   // m/s
static constexpr float kAnimTimeout = 50;    // s
static constexpr float kAlignDist = 1.0;     // m abeam door
static constexpr float kArrivalEps = 0.05f;  // 5cm fixed threshold for arrival detection

Sound JwCtrl::alert_;

// convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 can be nullptr
// Simplified math for small rot3 <= 5°!
void JwCtrl::XzToSamDref(float cabin_x, float cabin_z, float& rot1, float& extent, float* rot2, float* rot3) {
    float dist = len2f(cabin_x - x_, cabin_z - z_);

    float rot1_d = atan2(cabin_z - z_, cabin_x - x_) / kD2R;  // door frame
    rot1 = RA(rot1_d + 90.0f - psi_);
    extent = dist - jw_->cabinPos;

    // angle 0° door frame  -> hdgt -> jw_ frame -> diff to rot1
    float r2 = RA(0.0f + 90.0f - psi_ - rot1);
    if (rot2)
        *rot2 = r2;

    if (rot3) {
        float net_length = dist + jw_->cabinLength * cosf(r2 * kD2R);
        float sin_arg = y_ / net_length;
        sin_arg = std::clamp(sin_arg, -1.0f, 1.0f);  // clamp to valid range for asinf
        *rot3 = -asinf(sin_arg) / kD2R;
    }
}

//
// fill in geometry data related to specific door
//
void JwCtrl::SetupForDoor(Plane& plane, const DoorInfo& door_info) {
    // rotate into plane local frame using double precision for initial calculation
    double dx = static_cast<double>(jw_->x) - static_cast<double>(plane.x());
    double dz = static_cast<double>(jw_->z) - static_cast<double>(plane.z());
    double plane_psi = static_cast<double>(plane.psi());

    double sin_psi = sin(kD2R * plane_psi);
    double cos_psi = cos(kD2R * plane_psi);

    // Calculate in double precision, then convert to float for storage
    x_ = static_cast<float>(cos_psi * dx + sin_psi * dz);
    z_ = static_cast<float>(-sin_psi * dx + cos_psi * dz);
    psi_ = RA(jw_->psi - plane.psi());

    // xlate into door local frame
    x_ -= door_info.x;
    z_ -= door_info.z;

    float rot1_d = RA((jw_->initialRot1 + psi_) - 90.0f);  // door frame
    cabin_x_ = x_ + (jw_->extent + jw_->cabinPos) * cosf(rot1_d * kD2R);
    cabin_z_ = z_ + (jw_->extent + jw_->cabinPos) * sinf(rot1_d * kD2R);

    door_x_ = -jw_->cabinLength;
    // tgt z = 0.0
    y_ = (jw_->y + jw_->height) - (plane.y() + door_info.y);

    XzToSamDref(door_x_, 0.0f, door_rot1_, door_extent_, &door_rot2_, &door_rot3_);

    // parked position
    float r = jw_->initialExtent + jw_->cabinPos;
    parked_x_ = x_ + r * cosf(rot1_d * kD2R);
    parked_z_ = z_ + r * sinf(rot1_d * kD2R);

    ap_x_ = door_x_ - kAlignDist;

    jw_->SetWheels();
}

// a fuzzy comparator for jetway by door number
bool operator<(const JwCtrl& a, const JwCtrl& b) {
    // height goes first
    if (a.jw_->height < b.jw_->height - 1.0f)
        return true;

    if (a.jw_->height > b.jw_->height + 1.0f)
        return false;

    // then z
    if (a.z_ < b.z_ - 0.5f)
        return true;

    if (a.z_ > b.z_ + 0.5f)
        return false;

    // then x, further left (= towards -x) is higher priority
    if (a.x_ > b.x_)
        return true;

    return false;  // equal elements must return false for strict weak ordering
}

// filter list of jetways jws[]for candidates and add them to nearest_jws[]
static void FilterCandidates(Plane& plane, std::vector<JwCtrl>& nearest_jws, std::vector<SamJw*>& jws,
                             const DoorInfo& door_info) {
    // Unfortunately maxExtent in sam.xml can be bogus (e.g. FlyTampa EKCH)
    // So we find the nearest jetways on the left and do some heuristics

    int invisible_jws = 0;
    int locked_jws = 0;
    int too_far_jws = 0;
    int wrong_angle_jws = 0;
    int out_of_range_jws = 0;

    for (auto jw : jws) {
        if (jw->obj_ref_gen < ref_gen) { // not visible -> not dockable
            invisible_jws++;
            continue;
        }

        if (jw->locked) {
            locked_jws++;
            LogMsg("REJECTED %s: jetway is locked (in use by another plane)", jw->name.c_str());
            continue;
        }

        // LogMsg("pid=%02d, %s door %d, global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f",
        //         plane.id_, jw->name.c_str(), jw->door, jw->x, jw->z, jw->y, jw->psi);

        // set up a tentative JwCtrl ...
        JwCtrl njw{};
        njw.jw_ = jw;
        njw.SetupForDoor(plane, door_info);

        // ... and send it through the filters ...
        if (njw.x_ > 1.0f) {
            too_far_jws++;
            if (fabsf(njw.x_) < 120.0f && fabsf(njw.z_) < 120.0f)
                LogMsg("REJECTED %s: jetway on wrong side (x=%.1f > 1.0, should be on left)",
                       jw->name.c_str(), njw.x_);
            continue;
        }

        float rot_check = RA(njw.psi_ + jw->initialRot1);
        if (BETWEEN(rot_check, -130.0f, 20.0f)) {
            wrong_angle_jws++;
            if (fabsf(njw.x_) < 120.0f && fabsf(njw.z_) < 120.0f)
                LogMsg("REJECTED %s: jetway pointing wrong direction (rot=%.1f, should be outside [-130, 20])",
                       jw->name.c_str(), rot_check);
            continue;
        }

        if (njw.x_ < -80.0f || fabsf(njw.z_) > 80.0f) {
            too_far_jws++;
            if (fabsf(njw.x_) < 120.0f && fabsf(njw.z_) < 120.0f)
                LogMsg("REJECTED %s: jetway too far (x=%.1f, z=%.1f, max distance 80m)",
                       jw->name.c_str(), njw.x_, njw.z_);
            continue;
        }

        static constexpr float kSoftMatchMaxExtra = 8.0f;  // max extra extent for soft match

        if (!(BETWEEN(njw.door_rot1_, jw->minRot1, jw->maxRot1) && BETWEEN(njw.door_rot2_, jw->minRot2, jw->maxRot2) &&
              BETWEEN(njw.door_extent_, jw->minExtent, jw->maxExtent))) {
            LogMsg("REJECTED %s: out of mechanical range", jw->name.c_str());
            LogMsg("  rot1=%.1f (range: %.1f to %.1f)", njw.door_rot1_, jw->minRot1, jw->maxRot1);
            LogMsg("  rot2=%.1f (range: %.1f to %.1f)", njw.door_rot2_, jw->minRot2, jw->maxRot2);
            LogMsg("  extent=%.1f (range: %.1f to %.1f)", njw.door_extent_, jw->minExtent, jw->maxExtent);

            float extra_extent = njw.door_extent_ - jw->maxExtent;
            if (extra_extent > 0.0f && extra_extent < kSoftMatchMaxExtra) {
                LogMsg("  -> SOFT MATCH: extra extent %.1fm within tolerance %.1fm", extra_extent, kSoftMatchMaxExtra);
                njw.soft_match_ = 1;
            } else if (extra_extent >= kSoftMatchMaxExtra) {
                out_of_range_jws++;
                continue;
            } else {
                out_of_range_jws++;
                continue;
            }
        }

        // ... survived, add to list
        LogMsg("ACCEPTED %s: door=%d, x=%.1f, z=%.1f, rot1=%.1f, extent=%.1f",
               jw->name.c_str(), jw->door, njw.x_, njw.z_, njw.door_rot1_, njw.door_extent_);

        nearest_jws.push_back(njw);
    }

    // Summary statistics
    if (jws.size() > 0) {
        LogMsg("Filter summary: total=%d, invisible=%d, locked=%d, too_far=%d, wrong_angle=%d, out_of_range=%d, accepted=%d",
               (int)jws.size(), invisible_jws, locked_jws, too_far_jws, wrong_angle_jws, out_of_range_jws,
               (int)nearest_jws.size());
    }
}

// find nearest jetways, order by z (= door number, hopefully)
// static member, called by Plane
int JwCtrl::FindNearestJetway(Plane& plane, std::vector<JwCtrl>& nearest_jws) {
    int n_door = plane.n_door_;
    if (n_door == 0) {
        LogMsg("acf has no doors!");
        return 0;
    }

    // in case we move from a SAM airport to one with XP12 default
    // or autogate jetways this test never executes in the data accessors
    // so we may end up with a stale zc_jws table here
    CheckRefFrameShift();

    // compute the 'average' door location
    DoorInfo avg_di;
    avg_di.x = 0.0f;
    avg_di.y = 0.0f;
    avg_di.z = 0.0f;
    for (int i = 0; i < n_door; i++) {
        avg_di.x += plane.door_info_[i].x;
        avg_di.y += plane.door_info_[i].y;
        avg_di.z += plane.door_info_[i].z;
    }

    avg_di.x /= n_door;
    avg_di.y /= n_door;
    avg_di.z /= n_door;

    nearest_jws.resize(0);

    // custom jws
    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (!sc->in_bbox(plane_lat, plane_lon))
            continue;
        FilterCandidates(plane, nearest_jws, sc->sam_jws, avg_di);
    }

    // and zero config jetways
    FilterCandidates(plane, nearest_jws, zc_jws, avg_di);

    // sort for door assignment
    std::sort(nearest_jws.begin(), nearest_jws.end());

    // fake names for zc jetways
    int i{0};
    for (auto& njw : nearest_jws) {
        SamJw* jw = njw.jw_;
        if (jw->is_zc_jw) {
            Stand* stand = jw->stand;
            if (stand) {
                // stand->id can be eveything from "A11" to "A11 - Terminal 1 (cat C)"
                jw->name = stand->id;
                // truncate at ' ' or after 10 chars max
                size_t pos = jw->name.find(' ');
                if (pos != std::string::npos)
                    jw->name = jw->name.substr(0, pos);
                if (jw->name.length() > 10)
                    jw->name = jw->name.substr(0, 10);
                jw->name = jw->name + "_" + std::to_string(i);
            } else
                jw->name = "zc_" + std::to_string(i);

            i++;
        }
    }

    // lock all nearest_jws
    for (auto& njw : nearest_jws)
        njw.jw_->locked = true;

    // Summary log
    if (nearest_jws.size() == 0) {
        LogMsg("=== JETWAY SEARCH FAILED ===");
        LogMsg("  Plane ICAO: %s, position: x=%.1f, z=%.1f, psi=%.1f",
               plane.icao().c_str(), plane.x(), plane.z(), plane.psi());
        LogMsg("  Doors: %d", n_door);
        LogMsg("  Sceneries checked: %d", (int)sceneries.size());
        LogMsg("  Zero-config jetways: %d", (int)zc_jws.size());
        LogMsg("  Reason: No jetway within range or all filtered out");
    } else {
        LogMsg("=== JETWAY SEARCH SUCCESS ===");
        LogMsg("  Found %d candidate jetway(s)", (int)nearest_jws.size());
    }

    return nearest_jws.size();
}

// det of 2 column vectors x,y
static inline float det(float x1, float x2, float y1, float y2) {
    return x1 * y2 - x2 * y1;
}

// check whether extended nearest njw would crash into parked njw2
bool JwCtrl::CollisionCheck(const JwCtrl& njw2) {
    // S = start, E = extended, P = parked; all (x, z) vectors
    // we solve
    //  S1 + s * (E1 - S1) = S2 + t * (P2 - S2)
    //  s * (E1 - S1) + t * -(P2 - S2) = S2 - S1
    //          A                B          C
    // if the solutions for s, t are in [0,1] there is collision

    // x, z in the door frame
    float A1 = door_x_ - x_;
    float A2 =         -z_;  // door_z is 0 in the door frame

    float B1 = -(njw2.parked_x_ - njw2.x_);
    float B2 = -(njw2.parked_z_ - njw2.z_);

    float C1 = njw2.x_ - x_;
    float C2 = njw2.z_ - z_;

    float d = det(A1, A2, B1, B2);
    if (fabsf(d) < 0.2f)
        return false;

    float s = det(C1, C2, B1, B2) / d;
    float t = det(A1, A2, C1, C2) / d;
    LogMsg("collision check between jw %s and %s, s = %0.2f, t = %0.2f", jw_->name.c_str(), njw2.jw_->name.c_str(), s, t);

    if (BETWEEN(t, 0.0f, 1.0f) && BETWEEN(s, 0.0f, 1.0f)) {
        LogMsg("collision detected");
        return true;
    }

    return false;
}

// check whether two jetways collide when both extend to their doors
bool JwCtrl::CollisionCheckExtended(const JwCtrl& njw2) const {
    // S = start, E = extended; all (x, z) vectors
    // we solve
    //  S1 + s * (E1 - S1) = S2 + t * (E2 - S2)
    //  s * (E1 - S1) + t * -(E2 - S2) = S2 - S1
    //          A                B          C
    // if the solutions for s, t are in [0,1] there is collision

    // x, z in the door frame
    float A1 = door_x_ - x_;
    float A2 = -z_;  // door_z is 0 in the door frame

    float B1 = -(njw2.door_x_ - njw2.x_);
    float B2 = -(-njw2.z_);  // njw2.door_z is 0

    float C1 = njw2.x_ - x_;
    float C2 = njw2.z_ - z_;

    float d = det(A1, A2, B1, B2);
    if (fabsf(d) < 0.2f)
        return false;  // parallel or near parallel

    float s = det(C1, C2, B1, B2) / d;
    float t = det(A1, A2, C1, C2) / d;
    LogMsg("extended collision check between jw %s and %s, s = %0.2f, t = %0.2f", jw_->name.c_str(),
           njw2.jw_->name.c_str(), s, t);

    if (BETWEEN(s, 0.0f, 1.0f) && BETWEEN(t, 0.0f, 1.0f)) {
        LogMsg("collision detected (both extended)");
        return true;
    }

    return false;
}

// all the animation methods
bool JwCtrl::RotateWheelBase(float dt) {
    float delta_rot = RA(wb_rot_ - jw_->wheelrotatec);

    // optimize rotation
    if (delta_rot > 90.0f)
        delta_rot -= 180.0f;
    else if (delta_rot < -90.0f)
        delta_rot += 180.0f;

    // LogMsg("wb_rot_: %0.2f, delta_rot: %0.2f, wheelrotatec: %0.2f",
    //         wb_rot_, delta_rot, jw_->wheelrotatec);

    // wheel base rotation
    bool done = true;
    float d_rot;
    if (fabsf(delta_rot) > 2.0f) {
        d_rot = dt * kTurnSpeed;
        // LogMsg("turning wheel base by %0.2f°", d_rot);
        if (delta_rot < 0.0f)
            d_rot = -d_rot;

        jw_->wheelrotatec += d_rot;

        done = false;  // must wait
    } else {
        d_rot = delta_rot;
        jw_->wheelrotatec += delta_rot;
    }

    float da_rot = d_rot * (jw_->wheelDistance / jw_->wheelDiameter);

    jw_->wheelrotatel += da_rot;
    jw_->wheelrotater -= da_rot;
    return done;
}

// rotation1 + extend
void JwCtrl::Rotate1Extend() {
    XzToSamDref(cabin_x_, cabin_z_, jw_->rotate1, jw_->extent, nullptr, nullptr);
    jw_->SetWheels();
}

// rotation 3
bool JwCtrl::Rotate3(float rot3, float dt) {
    if (fabsf(jw_->rotate3 - rot3) > 0.1) {
        float d_rot3 = (dt * kHeightSpeed / (jw_->cabinPos + jw_->extent)) / kD2R;  // strictly it's atan
        if (jw_->rotate3 >= rot3)
            jw_->rotate3 = std::max(jw_->rotate3 - d_rot3, rot3);
        else
            jw_->rotate3 = std::min(jw_->rotate3 + d_rot3, rot3);
    }

    jw_->SetWheels();

    if (fabsf(jw_->rotate3 - rot3) > 0.1f)
        return 0;

    jw_->rotate3 = rot3;
    return 1;
}

// rotation 2
bool JwCtrl::Rotate2(float rot2, float dt) {
    if (fabsf(jw_->rotate2 - rot2) > 0.5) {
        float d_rot2 = dt * kTurnSpeed;
        if (jw_->rotate2 >= rot2)
            jw_->rotate2 = std::max(jw_->rotate2 - d_rot2, rot2);
        else
            jw_->rotate2 = std::min(jw_->rotate2 + d_rot2, rot2);
        return fabsf(jw_->rotate2 - rot2) <= 0.5;
    }

    jw_->rotate2 = rot2;
    return true;
}

// animate wheels for straight driving
void JwCtrl::AnimateWheels(float ds) {
    if (fabsf(RA(wb_rot_ - jw_->wheelrotatec)) > 90.0f)
        ds = -ds;
    // LogMsg("wb_rot_: %0.2f, wheelrotatec: %0.2f, ds: 0.3f", wb_rot_, jw_->wheelrotatec, ds);

    float da_ds = (ds / jw_->wheelDiameter) / kD2R;

    jw_->wheelrotatel += da_ds;
    jw_->wheelrotater += da_ds;
}

// drive jetway to the door
// return 1 when done
bool JwCtrl::DockDrive() {
    if (state_ == DOCKED)
        return true;

    if (now < start_ts_)
        return false;

    // guard against a hung animation
    if (now > timeout_) {
        LogMsg("dock_drive() timeout!");
        state_ = DOCKED;
        jw_->rotate1 = door_rot1_;
        jw_->rotate2 = door_rot2_;
        jw_->rotate3 = door_rot3_;
        jw_->extent = door_extent_;
        jw_->warnlight = 0;
        AlertOff();
        return true;  // -> done
    }

    float dt = now - last_step_ts_;
    last_step_ts_ = now;

    float rot1_d = RA((jw_->rotate1 + psi_) - 90.0f);  // door frame

    // float wheel_x = x_ + (jw_->extent + jw_->wheelPos) * cosf(rot1_d * kD2R);
    // float wheel_z = z_ + (jw_->extent + jw_->wheelPos) * sinf(rot1_d * kD2R);

    if (state_ == TO_AP) {
        if (wait_wb_rot_) {
            // LogMsg("TO_AP: waiting for wb rotation");
            if (!RotateWheelBase(dt))
                return false;
            wait_wb_rot_ = false;
        }

        float tgt_x = ap_x_;

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        // LogMsg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(cabin_z_) < eps) {
            state_ = AT_AP;
            LogMsg("align point reached");
            return false;
        }

        double ds = dt * kDriveSpeed;

        // Well, the wheels are somewhat behind the cabin so this is only approximate
        // but doesn't make much of a difference.
        double drive_angle = atan2(-cabin_z_, tgt_x - cabin_x_) / kD2R;

        // wb_rot_ is drive_angle in the 'tunnel frame'
        wb_rot_ = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw_->extent <= jw_->minExtent && wb_rot_ < -90.0f) {
            wb_rot_ = -90.0f;
            drive_angle = RA(rot1_d + -90.0f);
        }

        cabin_x_ += cos(drive_angle * kD2R) * ds;
        cabin_z_ += sin(drive_angle * kD2R) * ds;

        // LogMsg("to ap: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, drive_angle: %0.2f, wb_rot_: %0.2f",
        //         rot1_d, cabin_x_, cabin_z_, drive_angle, wb_rot_);

        if (!RotateWheelBase(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        // rotation2
        float tgt_rot2 = door_rot2_;
        if (cabin_x_ < (tgt_x - 1.0f) || cabin_z_ < -2.0f) {
            float angle_to_door = atan2f(-cabin_z_, door_x_ - cabin_x_) / kD2R;
            tgt_rot2 = RA(angle_to_door + 90.0f - psi_ - jw_->rotate1);  // point to door
        }
        // LogMsg("jw_->rotate2: %0.1f, tgt_rot2: %0.1f, tgt_rot2: %0.1f", jw_->rotate2, tgt_rot2, tgt_rot2);

        Rotate2(tgt_rot2, dt);
        Rotate1Extend();
        Rotate3(door_rot3_, dt);
        AnimateWheels(ds);
    }

    if (state_ == AT_AP) {
        // use the time to rotate the wheel base towards the door
        wb_rot_ = RA(-rot1_d);
        RotateWheelBase(dt);

        // rotation 2 + 3 must be at target now
        if (Rotate2(door_rot2_, dt) && Rotate3(door_rot3_, dt))
            state_ = TO_DOOR;
    }

    if (state_ == TO_DOOR) {
        if (wait_wb_rot_) {
            // LogMsg("TO_AP: waiting for wb rotation");
            if (!RotateWheelBase(dt))
                return false;
            wait_wb_rot_ = false;
        }

        double tgt_x = door_x_;

        cabin_x_ = std::min(cabin_x_, tgt_x);  // don't drive beyond the target point

        // LogMsg("to door: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f", rot1_d, cabin_x_, cabin_z_);

        // ramp down speed when approaching the plane
        float drive_speed = kDriveSpeed;
        if (cabin_x_ >= (tgt_x - 0.8f))
            drive_speed = kDriveSpeed * (0.1f + 0.9f * std::max(0.0f, float((tgt_x - cabin_x_)) / 0.8f));

        float ds = dt * drive_speed;

        cabin_x_ += ds;
        // LogMsg("cabin_x_: %0.3f, cabin_z_: %0.3f", cabin_x_, cabin_z_);

        wb_rot_ = RA(-rot1_d);
        if (!RotateWheelBase(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        Rotate1Extend();
        AnimateWheels(ds);

        // Use fixed threshold for final arrival detection
        if (fabs(tgt_x - cabin_x_) < kArrivalEps) {
            state_ = DOCKED;
            LogMsg("door reached");
            jw_->warnlight = 0;
            AlertOff();
            return true;  // done
        }
    }

    AlertSetpos();
    return false;
}

// drive jetway to parked position
bool JwCtrl::UndockDrive() {
    if (state_ == PARKED)
        return true;

    if (now < start_ts_)
        return false;

    // guard against a hung animation
    if (now > timeout_) {
        LogMsg("UndockDrive() timeout!");
        state_ = PARKED;
        jw_->Reset();
        AlertOff();
        return true;  // -> done
    }

    float dt = now - last_step_ts_;
    last_step_ts_ = now;

    float rot1_d = RA((jw_->rotate1 + psi_) - 90.0f);  // door frame

    // float wheel_x = x + (jw_->extent + jw_->wheelPos) * cosf(rot1_d * kD2R);
    // float wheel_z = z + (jw_->extent + jw_->wheelPos) * sinf(rot1_d * kD2R);

    if (state_ == TO_AP) {
        if (wait_wb_rot_) {
            // LogMsg("TO_AP: waiting for wb rotation");
            if (!RotateWheelBase(dt)) {
                return false;
            }
            wait_wb_rot_ = false;
        }

        float tgt_x = ap_x_;

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        // LogMsg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(cabin_z_) < eps) {
            state_ = AT_AP;
            LogMsg("align point reached reached");
            return false;
        }

        double ds = dt * 0.5 * kDriveSpeed;
        double drive_angle = atan2(-cabin_z_, tgt_x - cabin_x_) / kD2R;

        cabin_x_ += cos(drive_angle * kD2R) * ds;
        cabin_z_ += sin(drive_angle * kD2R) * ds;
        // LogMsg("to ap: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle:
        // %0.2f",
        //         rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z, drive_angle);

        wb_rot_ = RA(drive_angle - rot1_d);
        if (!RotateWheelBase(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        Rotate1Extend();
        AnimateWheels(ds);
    }

    if (state_ == AT_AP) {
        // nothing for now
        state_ = TO_PARK;
    }

    if (state_ == TO_PARK) {
        if (wait_wb_rot_) {
            // LogMsg("TO_AP: waiting for wb rotation");
            if (!RotateWheelBase(dt)) {
                return false;
            }
            wait_wb_rot_ = false;
        }

        float tgt_x = parked_x_;
        float tgt_z = parked_z_;

        // LogMsg("to park: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f",
        //         rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z);

        double ds = dt * kDriveSpeed;
        double drive_angle = atan2(tgt_z - cabin_z_, tgt_x - cabin_x_) / kD2R;

        // wb_rot_ is drive_angle in the 'tunnel frame'
        wb_rot_ = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw_->extent <= jw_->minExtent && wb_rot_ > 90.0f) {
            wb_rot_ = 90.0f;
            drive_angle = RA(rot1_d + 90.0f);
        }

        cabin_x_ += cos(drive_angle * kD2R) * ds;
        cabin_z_ += sin(drive_angle * kD2R) * ds;
        // LogMsg("to parked: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f,
        // drive_angle: %0.2f",
        //        rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z, drive_angle);

        if (!RotateWheelBase(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        Rotate2(jw_->initialRot2, dt);
        Rotate3(jw_->initialRot3, dt);
        Rotate1Extend();
        AnimateWheels(ds);

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        // LogMsg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(tgt_z - cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(tgt_z - cabin_z_) < eps) {
            state_ = PARKED;
            jw_->warnlight = 0;
            AlertOff();
            LogMsg("park position reached");
            jw_->locked = false;
            return true;  // done
        }
    }

    AlertSetpos();
    return false;
}

void JwCtrl::SetupDockUndock(float start_time, bool with_sound) {
    state_ = TO_AP;
    start_ts_ = start_time;
    last_step_ts_ = start_ts_;
    timeout_ = start_ts_ + kAnimTimeout;
    if (with_sound)
        AlertOn();
    jw_->warnlight = 1;
}

void JwCtrl::Reset() {
    AlertOff();
    jw_->Reset();
}

// static
void JwCtrl::SoundInit() {
    // load alert sound
    ReadWav(base_dir + "sound/alert.wav", alert_);
    if (alert_.data)
        LogMsg("alert sound loaded, channels: %d, bit_rate: %d, size: %d", alert_.num_channels, alert_.sample_rate,
               alert_.size);
    else
        throw OsEx("Could not load sound");

    if (!SoundDevInit())
        throw OsEx("Could not init sound");
}

// static
void JwCtrl::Init() {
}
