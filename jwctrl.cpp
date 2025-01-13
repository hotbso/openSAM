/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2025  Holger Teutsch

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
extern void read_wav(const std::string& fname, Sound& sound);

static constexpr float kDriveSpeed = 1.0;    // m/s
static constexpr float kTurnSpeed = 10.0;    // °/s
static constexpr float kHeightSpeed = 0.1;   // m/s
static constexpr float kAnimTimeout = 50;    // s
static constexpr float kAlignDist = 1.0;     // m abeam door

Sound JwCtrl::alert_;

// opensam/jetway/status dataref
//  0 = no jetway
//  1 = can dock
//  2 = docked
// -1 = can't dock or in transit
//
// ref == 0: opensam/jetway/number
// ref == 1: opensam/jetway/status
static int
jw_status_acc(void *ref)
{
    // opensam/jetway/number
    if (ref == 0)
        return my_plane->active_jws_.size();

    // opensam/jetway/status
    if (0 == my_plane->active_jws_.size())
        return 0;

    if (Plane::CAN_DOCK == my_plane->state_)
        return 1;

    if (Plane::DOCKED == my_plane->state_)
        return 2;

    return -1;
}

// opensam/jetway/door/status array by door
//  0 = not docked
//  1 = docked
//
static int
jw_door_status_acc(XPLMDataRef ref, int *values, int ofs, int n)
{
    UNUSED(ref);

    if (values == nullptr)
        return kMaxDoor;

    if (n <= 0 || ofs < 0 || ofs >= kMaxDoor)
        return 0;

    n = std::min(n, kMaxDoor - ofs);

    for (int i = 0; i < n; i++) {
        values[i] = 0;
    }

    for (auto & ajw : my_plane->active_jws_)
        if (ajw.state_ == JwCtrl::DOCKED)
            values[ajw.door_] = 1;

    return n;
}


// convert tunnel end at (cabin_x, cabin_z) to dataref values; rot2, rot3 can be nullptr
void
JwCtrl::xz_to_sam_dr(float cabin_x, float cabin_z,
                     float& rot1, float& extent, float *rot2, float *rot3)
{
    float dist = len2f(cabin_x - x_, cabin_z - z_);

    float rot1_d = atan2(cabin_z - z_, cabin_x - x_) / D2R;   // door frame
    rot1 =  RA(rot1_d + 90.0f - psi_);
    extent = dist - jw_->cabinPos;

    // angle 0° door frame  -> hdgt -> jw_ frame -> diff to rot1
    float r2 = RA(0.0f + 90.0f - psi_ - rot1);
    if (rot2)
        *rot2 = r2;

    if (rot3) {
        float net_length = dist + jw_->cabinLength * cosf(r2 * D2R);
        *rot3 = -atan2f(y_, net_length) / D2R;
    }
}

//
// fill in geometry data related to specific door
//
void
JwCtrl::setup_for_door(Plane *plane, const DoorInfo& door_info)
{
    // rotate into plane local frame
    float dx = jw_->x - plane->x();
    float dz = jw_->z - plane->z();
    float plane_psi = plane->psi();

    float sin_psi = sinf(D2R * plane_psi);
    float cos_psi = cosf(D2R * plane_psi);

    x_ =  cos_psi * dx + sin_psi * dz;
    z_ = -sin_psi * dx + cos_psi * dz;
    psi_ = RA(jw_->psi - plane_psi);

    // xlate into door local frame
    x_ -= door_info.x;
    z_ -= door_info.z;

    float rot1_d = RA((jw_->initialRot1 + psi_) - 90.0f);    // door frame
    cabin_x_ = x_ + (jw_->extent + jw_->cabinPos) * cosf(rot1_d * D2R);
    cabin_z_ = z_ + (jw_->extent + jw_->cabinPos) * sinf(rot1_d * D2R);

    door_x_ = -jw_->cabinLength;
    // tgt z = 0.0
    y_ = (jw_->y + jw_->height) - (plane->y() + door_info.y);

    xz_to_sam_dr(door_x_, 0.0f, door_rot1_, door_extent_, &door_rot2_, &door_rot3_);

    float r = jw_->initialExtent + jw_->cabinPos;
    parked_x_ = x_ + r * cosf(rot1_d * D2R);
    parked_z_ = z_ + r * sinf(rot1_d * D2R);

    ap_x_ = door_x_ - kAlignDist;

    jw_->set_wheels();
}

// a fuzzy comparator for jetway by door number
bool
operator<(const JwCtrl& a, const JwCtrl& b)
{
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

    // then x, further left (= towards -x) is higher
    if (a.x_ < b.x_)
        return false;

    if (a.x_ > b.x_)
        return true;

    return true;
}

// filter list of jetways jws[]for candidates and add them to nearest_jws[]
static void
filter_candidates(Plane *plane, std::vector<SamJw*> &jws, const DoorInfo& door_info, float& dist_threshold)
{
    // Unfortunately maxExtent in sam.xml can be bogus (e.g. FlyTampa EKCH)
    // So we find the nearest jetways on the left and do some heuristics

    for (auto jw_ : jws) {
        if (jw_->obj_ref_gen < ref_gen)  // not visible -> not dockable
            continue;

        //log_msg("%s door %d, global: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f",
        //        jw_->name, jw_->door, jw_->x, jw_->z, jw_->y, jw_->psi);

        // set up a tentative JwCtrl ...
        JwCtrl njw{};
        njw.jw_ = jw_;
        njw.setup_for_door(plane, door_info);

        // ... and send it through the filters ...
        if (njw.x_ > 1.0f || BETWEEN(RA(njw.psi_ + jw_->initialRot1), -130.0f, 20.0f) ||   // on the right side or pointing away
            njw.x_ < -80.0f || fabsf(njw.z_) > 80.0f) {             // or far away
            if (fabsf(njw.x_) < 120.0f && fabsf(njw.z_) < 120.0f)   // don't pollute the log with jws VERY far away
                log_msg("too far or pointing away: %s, x: %0.2f, z: %0.2f, (njw.psi + jw_->initialRot1): %0.1f",
                        jw_->name, njw.x_, njw.z_, njw.psi_ + jw_->initialRot1);
            continue;
        }

        if (njw.z_ > dist_threshold)
            continue;

        if (!(BETWEEN(njw.door_rot1_, jw_->minRot1, jw_->maxRot1) && BETWEEN(njw.door_rot2_, jw_->minRot2, jw_->maxRot2)
            && BETWEEN(njw.door_extent_, jw_->minExtent, jw_->maxExtent))) {
            log_msg("jw_: %s for door %d, rot1: %0.1f, rot2: %0.1f, rot3: %0.1f, extent: %0.1f",
                     jw_->name, jw_->door, njw.door_rot1_, njw.door_rot2_, njw.door_rot3_, njw.door_extent_);
            log_msg("  does not fulfil min max criteria in sam.xml");
            float extra_extent = njw.door_extent_ - jw_->maxExtent;
            if (extra_extent < 10.0f) {
                log_msg("  as extra extent of %0.1f m < 10.0 m we take it as a soft match", extra_extent);
                njw.soft_match_ = 1;
            } else
                continue;
        }

        // ... survived, add to list
        log_msg("--> candidate %s, lib_id: %d, door %d, door frame: x: %5.3f, z: %5.3f, y: %5.3f, psi: %4.1f, "
                "rot1: %0.1f, extent: %.1f",
                jw_->name, jw_->library_id, jw_->door,
                njw.x_, njw.z_, njw.y_, njw.psi_, njw.door_rot1_, njw.door_extent_);
        plane->nearest_jws_[plane->n_nearest_jws_] = njw;
        plane->n_nearest_jws_++;

        // if full, sort by dist and trim down to kNearJwLimit
        if (plane->n_nearest_jws_ == kMaxNearest) {
            std::sort(plane->nearest_jws_.begin(), plane->nearest_jws_.end());
            plane->n_nearest_jws_ = kNearJwLimit;
            dist_threshold = plane->nearest_jws_[kNearJwLimit - 1].z_;
        }
    }
}

// find nearest jetways, order by z (= door number, hopefully)
// static member, called by Plane
int
JwCtrl::find_nearest_jws(Plane* plane)
{
    int n_door = plane->n_door_;
    if (n_door == 0) {
        log_msg("acf has no doors!");
        return 0;
    }

    // in case we move from a SAM airport to one with XP12 default
    // or autogate jetways this test never executes in the data accessors
    // so we may end up with a stale zc_jws table here
    check_ref_frame_shift();

    // compute the 'average' door location
    DoorInfo avg_di;
    avg_di.x = 0.0f;
    avg_di.z = 0.0f;
    for (int i = 0; i < n_door; i++) {
        avg_di.x += plane->door_info_[i].x;
        avg_di.z += plane->door_info_[i].z;
    }

    avg_di.x /= n_door;
    avg_di.z /= n_door;
    avg_di.y = plane->door_info_[0].y;

    plane->n_nearest_jws_ = 0;
    float dist_threshold = 1.0E10f;

    // custom jws
    for (auto sc : sceneries)
        filter_candidates(plane, sc->sam_jws, avg_di, dist_threshold);

    // and zero config jetways
    filter_candidates(plane, zc_jws, avg_di, dist_threshold);

    if (plane->n_nearest_jws_ > 1) { // final sort + trim down to limit
        plane->n_nearest_jws_ = std::min(plane->n_nearest_jws_, kMaxNearest);  // required to keep the compiler happy
        std::sort(plane->nearest_jws_.begin(), &plane->nearest_jws_[plane->n_nearest_jws_]);
        plane->n_nearest_jws_ = std::min(plane->n_nearest_jws_, kNearJwLimit);
    }

    // fake names for zc jetways
    for (int i = 0; i < plane->n_nearest_jws_; i++) {
        SamJw *jw_ = plane->nearest_jws_[i].jw_;
        if (jw_->is_zc_jw) {
            Stand *stand = jw_->stand;
            if (stand) {
                // stand->id can be eveything from "A11" to "A11 - Terminal 1 (cat C)"
                char buf[sizeof(stand->id)];
                strcpy(buf, stand->id);
                // truncate at ' ' or after 10 chars max
                char *cptr = strchr(buf, ' ');
                if (cptr)
                    *cptr = '\0';
                int len = strlen(buf);
                if (len > 10)
                    buf[10] = '\0';
                snprintf(jw_->name, sizeof(jw_->name) -1, "%s_%c", buf, i + 'A');
            } else
                snprintf(jw_->name, sizeof(jw_->name) -1, "zc_%c", i + 'A');
        }
    }

    return plane->n_nearest_jws_;
}

// det of 2 column vectors x,y
static inline float
det(float x1, float x2, float y1, float y2)
{
    return x1 * y2 - x2 * y1;
}

// check whether extended nearest jw_ i would crash into parked jw_ j
static
int jw_collision_check(Plane *plane, int i, int j)
{
    // S = start, E = extended, P = parked; all (x, z) vectors
    // we solve
    //  S1 + s * (E1 - S1) = S2 + t * (P2 - S2)
    //  s * (E1 - S1) + t * -(P2 - S2) = S2 - S1
    //          A                B          C
    // if the solutions for s, t are in [0,1] there is collision

    const JwCtrl &njw1 = plane->nearest_jws_[i];
    const JwCtrl &njw2 = plane->nearest_jws_[j];

    // x, z in the door frame
    float A1 = njw1.door_x_ - njw1.x_;
    float A2 =              - njw1.z_;   // door_z is 0 in the door frame

    float B1 = -(njw2.parked_x_ - njw2.x_);
    float B2 = -(njw2.parked_z_ - njw2.z_);

    float C1 = njw2.x_ - njw1.x_;
    float C2 = njw2.z_ - njw1.z_;

    float d = det(A1, A2, B1, B2);
    if (fabsf(d) < 0.2f)
        return 0;

    float s = det(C1, C2, B1, B2) / d;
    float t = det(A1, A2, C1, C2) / d;
    log_msg("check between jw_ %d and %d, s = %0.2f, t = %0.2f", i, j, s, t);

    if (BETWEEN(t, 0.0f, 1.0f) && BETWEEN(s, 0.0f, 1.0f)) {
        log_msg("collision between jw_ %d and %d, s = %0.2f, t = %0.2f", i, j, s, t);
        return 1;
    }

    return 0;
}

// static member auto select active jetways
void
JwCtrl::select_jws(Plane* plane)
{
    if (plane->n_door_ == 0)
        return;

    int have_hard_match = 0;
    for (int i = 0; i < plane->n_nearest_jws_; i++)
        if (!plane->nearest_jws_[i].soft_match_) {
            have_hard_match = 1;
            break;
        }

    int i_door = 0;
    int i_jw = 0;
    while (i_jw < plane->n_nearest_jws_) {
        if (have_hard_match && plane->nearest_jws_[i_jw].soft_match_)
            goto skip;

        // skip over collisions
        for (int j = i_jw + 1; j < plane->n_nearest_jws_; j++)
            if (jw_collision_check(plane, i_jw, j))
                goto skip;

        plane->nearest_jws_[i_jw].door_ = i_door;
        plane->active_jws_.push_back(plane->nearest_jws_[i_jw]);
        log_msg("active jetway for door %d: %s", i_door, plane->active_jws_.back().jw_->name);
        i_door++;
        if (i_door >= plane->n_door_)
            break;

      skip:
        i_jw++;
    }

    if (plane->active_jws_.size() == 0)
        log_msg("Oh no, no active jetways left in select_jws()!");
}

// all the animation methods
bool
JwCtrl::rotate_wheel_base(float dt)
{
    float delta_rot = RA(wb_rot_ - jw_->wheelrotatec);

    // optimize rotation
    if (delta_rot > 90.0f)
        delta_rot -= 180.0f;
    else if (delta_rot < -90.0f)
        delta_rot += 180.0f;

    //log_msg("wb_rot_: %0.2f, delta_rot: %0.2f, wheelrotatec: %0.2f",
    //        wb_rot_, delta_rot, jw_->wheelrotatec);


    // wheel base rotation
    bool done = true;
    float d_rot;
    if (fabsf(delta_rot) > 2.0f) {
        d_rot = dt * kTurnSpeed;
        //log_msg("turning wheel base by %0.2f°", d_rot);
        if (delta_rot < 0.0f)
            d_rot = -d_rot;

        jw_->wheelrotatec += d_rot;

        done = false;    // must wait
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
void
JwCtrl::rotate_1_extend()
{
    xz_to_sam_dr(cabin_x_, cabin_z_, jw_->rotate1, jw_->extent, nullptr, nullptr);
    jw_->set_wheels();
}

// rotation 3
bool
JwCtrl::rotate_3(float rot3, float dt)
{
    if (fabsf(jw_->rotate3 - rot3) > 0.1) {
        float d_rot3 = (dt * kHeightSpeed / (jw_->cabinPos + jw_->extent)) / D2R;  // strictly it's atan
        if (jw_->rotate3 >= rot3)
            jw_->rotate3 = std::max(jw_->rotate3 - d_rot3, rot3);
        else
            jw_->rotate3 = std::min(jw_->rotate3 + d_rot3, rot3);
    }

    jw_->set_wheels();

    if (fabsf(jw_->rotate3 - rot3) > 0.1f)
        return 0;

    jw_->rotate3 = rot3;
    return 1;
}

// rotation 2
bool
JwCtrl::rotate_2(float rot2, float dt)
{
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
void
JwCtrl::animate_wheels(float ds)
{
    if (fabsf(RA(wb_rot_ - jw_->wheelrotatec)) > 90.0f)
        ds = -ds;
    //log_msg("wb_rot_: %0.2f, wheelrotatec: %0.2f, ds: 0.3f", wb_rot_, jw_->wheelrotatec, ds);

    float da_ds = (ds / jw_->wheelDiameter) / D2R;

    jw_->wheelrotatel += da_ds;
    jw_->wheelrotater += da_ds;
}

// drive jetway to the door
// return 1 when done
bool
JwCtrl::dock_drive()
{
    if (state_ == DOCKED)
        return true;

    if (now < start_ts_)
        return false;

    // guard against a hung animation
    if (now > timeout_) {
        log_msg("dock_drive() timeout!");
        state_ = DOCKED;
        jw_->rotate1 = door_rot1_;
        jw_->rotate2 = door_rot2_;
        jw_->rotate3 = door_rot3_;
        jw_->extent = door_extent_;
        jw_->warnlight = 0;
        alert_off();
        return true;   // -> done
    }

    float dt = now - last_step_ts_;
    last_step_ts_ = now;

    float rot1_d = RA((jw_->rotate1 + psi_) - 90.0f);    // door frame

    //float wheel_x = x_ + (jw_->extent + jw_->wheelPos) * cosf(rot1_d * D2R);
    //float wheel_z = z_ + (jw_->extent + jw_->wheelPos) * sinf(rot1_d * D2R);

    if (state_ == TO_AP) {
        if (wait_wb_rot_) {
            //log_msg("TO_AP: waiting for wb rotation");
            if (! rotate_wheel_base(dt))
                return false;
            wait_wb_rot_ = false;
        }

        float tgt_x = ap_x_;

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(cabin_z_) < eps)  {
            state_ = AT_AP;
            log_msg("align point reached reached");
            return false;
        }

        double ds = dt * kDriveSpeed;

        // Well, the wheels are somewhat behind the cabin so this is only approximate
        // but doesn't make much of a difference.
        double drive_angle = atan2(-cabin_z_, tgt_x - cabin_x_) / D2R;

        // wb_rot_ is drive_angle in the 'tunnel frame'
        wb_rot_ = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw_->extent <= jw_->minExtent && wb_rot_ < -90.0f) {
            wb_rot_ = -90.0f;
            drive_angle = RA(rot1_d + -90.0f);
        }

        cabin_x_ += cos(drive_angle * D2R) * ds;
        cabin_z_ += sin(drive_angle * D2R) * ds;

        //log_msg("to ap: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, drive_angle: %0.2f, wb_rot_: %0.2f",
        //        rot1_d, cabin_x_, cabin_z_, drive_angle, wb_rot_);

        if (! rotate_wheel_base(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        // rotation2
        float tgt_rot2 = door_rot2_;
        if (cabin_x_ < (tgt_x - 1.0f) || cabin_z_ < -2.0f) {
            float angle_to_door = atan2f(-cabin_z_, door_x_ - cabin_x_) / D2R;
            tgt_rot2 = RA(angle_to_door + 90.0f - psi_ - jw_->rotate1); // point to door
        }
        //log_msg("jw_->rotate2: %0.1f, tgt_rot2: %0.1f, tgt_rot2: %0.1f", jw_->rotate2, tgt_rot2, tgt_rot2);

        rotate_2(tgt_rot2, dt);
        rotate_1_extend();
        rotate_3(door_rot3_, dt);
        animate_wheels(ds);
    }

    if (state_ == AT_AP) {
        // use the time to rotate the wheel base towards the door
        wb_rot_ = RA(-rot1_d);
        rotate_wheel_base(dt);

        // rotation 2 + 3 must be at target now
        if (rotate_2(door_rot2_, dt) && rotate_3(door_rot3_, dt))
            state_ = TO_DOOR;
    }

    if (state_ == TO_DOOR) {
        if (wait_wb_rot_) {
            // log_msg("TO_AP: waiting for wb rotation");
            if (! rotate_wheel_base(dt))
                return false;
            wait_wb_rot_ = false;
        }

        double tgt_x = door_x_;

        cabin_x_ = std::min(cabin_x_, tgt_x); // don't drive beyond the target point

        //log_msg("to door: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f", rot1_d, cabin_x_, cabin_z_);

        // ramp down speed when approaching the plane
        float drive_speed = kDriveSpeed;
        if (cabin_x_ >= (tgt_x - 0.8f))
            drive_speed = kDriveSpeed * (0.1f + 0.9f * std::max(0.0f, float((tgt_x - cabin_x_)) / 0.8f));

        float ds = dt * drive_speed;

        cabin_x_ += ds;
        //log_msg("cabin_x_: %0.3f, cabin_z_: %0.3f", cabin_x_, cabin_z_);

        wb_rot_ = RA(-rot1_d);
        if (! rotate_wheel_base(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        rotate_1_extend();
        animate_wheels(ds);

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.05f);
        //log_msg("eps: %0.3f, d_x: %0.3f", eps, fabs(tgt_x - cabin_x_));
        if (fabs(tgt_x - cabin_x_) < eps) {
            state_ = DOCKED;
            log_msg("door reached");
            jw_->warnlight = 0;
            alert_off();
            return true;   // done
        }
    }

    alert_setpos();
    return false;
}


// drive jetway to parked position
bool
JwCtrl::undock_drive()
{
    if (state_ == PARKED)
        return true;

    if (now < start_ts_)
        return false;

    // guard against a hung animation
    if (now > timeout_) {
        log_msg("undock_drive() timeout!");
        state_ = PARKED;
        jw_->reset();
        alert_off();
        return true;   // -> done
    }

    float dt = now - last_step_ts_;
    last_step_ts_ = now;

    float rot1_d = RA((jw_->rotate1 + psi_) - 90.0f);    // door frame

    //float wheel_x = x + (jw_->extent + jw_->wheelPos) * cosf(rot1_d * D2R);
    //float wheel_z = z + (jw_->extent + jw_->wheelPos) * sinf(rot1_d * D2R);

    if (state_ == TO_AP) {
        if (wait_wb_rot_) {
            //log_msg("TO_AP: waiting for wb rotation");
            if (! rotate_wheel_base(dt)) {
                return false;
            }
            wait_wb_rot_ = false;
        }

        float tgt_x = ap_x_;

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(cabin_z_) < eps)  {
            state_ = AT_AP;
            log_msg("align point reached reached");
            return false;
        }

        double ds = dt * 0.5 * kDriveSpeed;
        double drive_angle = atan2(-cabin_z_, tgt_x - cabin_x_) / D2R;

        cabin_x_ += cos(drive_angle * D2R) * ds;
        cabin_z_ += sin(drive_angle * D2R) * ds;
        //log_msg("to ap: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle: %0.2f",
        //        rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z, drive_angle);

        wb_rot_ = RA(drive_angle - rot1_d);
        if (! rotate_wheel_base(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        rotate_1_extend();
        animate_wheels(ds);
    }

    if (state_ == AT_AP) {
        // nothing for now
        state_ = TO_PARK;
    }

    if (state_ == TO_PARK) {
        if (wait_wb_rot_) {
            // log_msg("TO_AP: waiting for wb rotation");
            if (! rotate_wheel_base(dt)) {
                return false;
            }
            wait_wb_rot_ = false;
        }

        float tgt_x = parked_x_;
        float tgt_z = parked_z_;

        //log_msg("to park: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f",
        //        rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z);

        double ds = dt * kDriveSpeed;
        double drive_angle = atan2(tgt_z - cabin_z_, tgt_x - cabin_x_) / D2R;

        // wb_rot_ is drive_angle in the 'tunnel frame'
        wb_rot_ = RA(drive_angle - rot1_d);

        // avoid compression of jetway
        if (jw_->extent <= jw_->minExtent && wb_rot_ > 90.0f) {
            wb_rot_ = 90.0f;
            drive_angle = RA(rot1_d + 90.0f);
        }

        cabin_x_ += cos(drive_angle * D2R) * ds;
        cabin_z_ += sin(drive_angle * D2R) * ds;
        //log_msg("to parked: rot1_d: %.2f, cabin_x_: %0.3f, cabin_z_: %0.3f, wheel_x: %0.3f, wheel_z: %0.3f, drive_angle: %0.2f",
        //       rot1_d, cabin_x_, cabin_z_, wheel_x, wheel_z, drive_angle);

        if (! rotate_wheel_base(dt)) {
            wait_wb_rot_ = true;
            return false;
        }
        wait_wb_rot_ = false;

        rotate_2(jw_->initialRot2, dt);
        rotate_3(jw_->initialRot3, dt);
        rotate_1_extend();
        animate_wheels(ds);

        float eps = std::max(2.0f * dt * kDriveSpeed, 0.1f);
        //log_msg("eps: %0.3f, %0.3f, %0.3f", eps, fabs(tgt_x - cabin_x_), fabs(tgt_z - cabin_z_));
        if (fabs(tgt_x - cabin_x_) < eps && fabs(tgt_z -cabin_z_) < eps)  {
            state_ = PARKED;
            jw_->warnlight = 0;
            alert_off();
            log_msg("park position reached");
            return true;   // done
        }
    }

    alert_setpos();
    return false;
}

void
JwCtrl::setup_dock_undock(float start_time)
{
    state_ = TO_AP;
    start_ts_ = start_time;
    last_step_ts_ = start_ts_;
    timeout_ = start_ts_ + kAnimTimeout;
    alert_on();
    jw_->warnlight = 1;
}

void
JwCtrl::reset()
{
    alert_off(); jw_->reset();
}

// static
void
JwCtrl::sound_init()
{
    // load alert sound
    read_wav(base_dir + "sound/alert.wav", alert_);
    if (alert_.data)
        log_msg("alert sound loaded, channels: %d, bit_rate: %d, size: %d",
                alert_.num_channels, alert_.sample_rate, alert_.size);
    else
        throw OsEx("Could not load sound");

    if (!sound_dev_init())
        throw OsEx("Could not init sound");
}

// static
void
JwCtrl::init()
{
    XPLMRegisterDataAccessor("opensam/jetway/number", xplmType_Int, 0, jw_status_acc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/status", xplmType_Int, 0, jw_status_acc,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, (void *)1, NULL);

    XPLMRegisterDataAccessor("opensam/jetway/door/status", xplmType_IntArray, 0, NULL, NULL,
                             NULL, NULL, NULL, NULL, jw_door_status_acc, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL);
}

