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

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "opensam.h"
#include "samjw.h"      // for CheckRefFrameShift
#include "os_anim.h"

static constexpr float kSam2ObjMax = 2.5;   // m, max delta between coords in sam.xml and object
static constexpr float kSam2ObjHdgMax = 5;  // °, likewise for heading

static Scenery* cur_sc;            // the current scenery determined by dref access
static float cur_sc_ts = -100.0f;  // timestamp of selection of cur_sc
Scenery* anim_sc;                  // the scenery active for animation

//
// Accessor for the "sam/..." custom animation datarefs
//
// This function is called from draw loops, efficient coding required.
//
// ref is uint64_t is the index into the global dref table
//
float SamAnim::AnimAcc(void* ref) {
    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    if (obj_x == 0.0f && obj_z == 0.0f && obj_psi == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    stat_anim_acc_called++;

    CheckRefFrameShift();

    int drf_idx = (uint64_t)ref;

    for (auto& sc : sceneries) {
        for (auto anim : sc.sam_anims_) {
            if (drf_idx != anim->drf_idx)
                continue;

            SamObj* obj = sc.sam_objs_[anim->obj_idx];

            if (fabsf(fem::RA(obj->heading - obj_psi)) > kSam2ObjHdgMax)
                continue;

            if (obj->xml_ref_gen < ref_gen) {
                double x, y, z;
                XPLMWorldToLocal(obj->latitude, obj->longitude, obj->elevation, &x, &y, &z);

                obj->xml_x = x;
                obj->xml_y = y;
                obj->xml_z = z;
                obj->xml_ref_gen = ref_gen;
            }

            if (fabs(obj_x - obj->xml_x) > kSam2ObjMax || fabs(obj_z - obj->xml_z) > kSam2ObjMax) {
                stat_near_skip++;
                continue;
            }

            SamDrf* drf = sam_drfs[drf_idx];
            // LogMsg("acc %s called, %s %s", drf->name, anim->label, anim->title);

            if (now > cur_sc_ts + 20.0f) {  // avoid high freq flicker
                cur_sc = &sc;
                cur_sc_ts = now;
            }

            if (anim->state == kOff2On || anim->state == kOn2Off) {
                now = XPLMGetDataf(total_running_time_sec_dr);
                float dt = now - anim->start_ts;

                if (anim->state == kOn2Off)
                    dt = drf->t[drf->n_tv - 1] - dt;  // downwards

                if (dt < 0.0f)
                    anim->state = kOff;
                else if (dt > drf->t[drf->n_tv - 1])
                    anim->state = kOn;
                else {
                    for (int j = 1; j < drf->n_tv; j++)
                        if (dt < drf->t[j])
                            return drf->v[j - 1] + drf->s[j] * (dt - drf->t[j - 1]);
                }
            }

            if (anim->state == kOff)
                return drf->v[0];
            if (anim->state == kOn)
                return drf->v[drf->n_tv - 1];
        }
    }

    return 0.0;
}

//
// Accessor for the "sam/..." autoplay datarefs
//
// This function is called from draw loops, efficient coding required.
//
// ref is a ptr to the dref
//
static float AutoDrfAcc(void* ref) {
    stat_auto_drf_called++;

    const SamDrf* drf = (const SamDrf*)ref;

    float t = XPLMGetDataf(total_running_time_sec_dr);

    if (drf->randomize_phase) {
        // that should give a deterministic per object random value
        float obj_x = XPLMGetDataf(draw_object_x_dr);
        float obj_y = XPLMGetDataf(draw_object_y_dr);
        t += fabsf(obj_x * 0.5f + obj_y);
    }

    float dt = fmodf(t, drf->t[drf->n_tv - 1]);

    for (int j = 1; j < drf->n_tv; j++)
        if (dt < drf->t[j])
            return drf->v[j - 1] + drf->s[j] * (dt - drf->t[j - 1]);

    return 0.0f;  // should never be reached
}

void SamAnim::SetState(bool on) {
    if (is_on() == on)
        return;

    LogMsg("SamAnim::SetState: label: %s, on: %d", label.c_str(), on);

    bool reverse;

    if (is_on()) {
        reverse = (state == SamAnim::kOff2On);
        state = SamAnim::kOn2Off;
    } else {
        reverse = (state == SamAnim::kOn2Off);
        state = SamAnim::kOff2On;
    }

    now = XPLMGetDataf(total_running_time_sec_dr);

    if (reverse) {
        SamDrf* drf = sam_drfs[drf_idx];
        float t_rel = now - start_ts;
        float dt = drf->t[drf->n_tv - 1] - t_rel;
        start_ts = now - dt;
    } else
        start_ts = now;
}

// not much state here, just regularly check for a current scenery
// and maintain the menu
float AnimStateMachine(void) {
    // check whether we have recently seen a scenery
    if (cur_sc && now > cur_sc_ts + 180.0f) {
        LogMsg("have not seen a custom animated scenery recently");
        cur_sc = NULL;
    }

    // keep building the ui_line outside of imgui
    if (cur_sc != anim_sc) {
        anim_sc = cur_sc;
        if (anim_sc) {
            for (auto& anim : anim_sc->sam_anims_) {
                if (anim->ui_line.empty())
                    anim->ui_line = anim->label + " " + anim->title;
            }
        }
    }

    return 5.0f;
}

bool AnimInit() {
    for (unsigned i = 0; i < sam_drfs.size(); i++) {
        const SamDrf* drf = sam_drfs[i];
        if (drf->autoplay)
            XPLMRegisterDataAccessor(drf->name.c_str(), xplmType_Float, 0, NULL, NULL, AutoDrfAcc, NULL, NULL, NULL,
                                     NULL, NULL, NULL, NULL, NULL, NULL, (void*)drf, NULL);
        else
            XPLMRegisterDataAccessor(drf->name.c_str(), xplmType_Float, 0, NULL, NULL, SamAnim::AnimAcc, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)i, NULL);
    }

    return true;
}
