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

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "openSAM.h"
#include "os_anim.h"

static const float SAM_2_OBJ_MAX = 2.5;     // m, max delta between coords in sam.xml and object
static const float SAM_2_OBJ_HDG_MAX = 5;   // °, likewise for heading

static Scenery* cur_sc;           // the current scenery determined by dref access
static float cur_sc_ts = -100.0f; // timestamp of selection of cur_sc
static Scenery* menu_sc;          // the scenery the menu is built of

//
// Accessor for the "sam/..." custom animation datarefs
//
// This function is called from draw loops, efficient coding required.
//
// ref is uint64_t is the index into the global dref table
//
static float
anim_acc(void *ref)
{
    stat_anim_acc_called++;

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    // check for shift of reference frame
    float lat_r = XPLMGetDataf(lat_ref_dr);
    float lon_r = XPLMGetDataf(lon_ref_dr);

    if (lat_r != lat_ref || lon_r != lon_ref) {
        lat_ref = lat_r;
        lon_ref = lon_r;
        ref_gen++;
        log_msg("reference frame shift");
    }

    int drf_idx = (uint64_t)ref;

    for (auto sc : sceneries) {
        for (auto anim : sc->sam_anims) {
            if (drf_idx != anim->drf_idx)
                continue;

            SamObj *obj = sc->sam_objs[anim->obj_idx];

            if (fabsf(RA(obj->heading - obj_psi)) > SAM_2_OBJ_HDG_MAX)
                continue;

            if (obj->xml_ref_gen < ref_gen) {
                double  x, y ,z;
                XPLMWorldToLocal(obj->latitude, obj->longitude, obj->elevation, &x, &y, &z);

                obj->xml_x = x;
                obj->xml_y = y;
                obj->xml_z = z;
                obj->xml_ref_gen = ref_gen;
            }

            if (fabs(obj_x - obj->xml_x) > SAM_2_OBJ_MAX || fabs(obj_z - obj->xml_z) > SAM_2_OBJ_MAX) {
                stat_near_skip++;
                continue;
            }

            SamDrf *drf = sam_drfs[drf_idx];
            //log_msg("acc %s called, %s %s", drf->name, anim->label, anim->title);

            if (now > cur_sc_ts + 20.0f) {  // avoid high freq flicker
                cur_sc = sc;
                cur_sc_ts = now;
            }

            if (anim->state == ANIM_OFF_2_ON || anim->state == ANIM_ON_2_OFF) {
                now = XPLMGetDataf(total_running_time_sec_dr);
                float dt = now - anim->start_ts;

                if (anim->state == ANIM_ON_2_OFF)
                    dt = drf->t[drf->n_tv - 1] - dt;     // downwards

                if (dt < 0.0f)
                    anim->state = ANIM_OFF;
                else if (dt > drf->t[drf->n_tv - 1])
                    anim->state = ANIM_ON;
                else {
                    for (int j = 1; j < drf->n_tv; j++)
                        if (dt < drf->t[j])
                            return drf->v[j-1] + drf->s[j] * (dt - drf->t[j-1]);
                }
            }

            if (anim->state == ANIM_OFF)
                return drf->v[0];
            if (anim->state == ANIM_ON)
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
static float
auto_drf_acc(void *ref)
{
    stat_auto_drf_called++;

    const SamDrf *drf = (const SamDrf *)ref;

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
            return drf->v[j-1] + drf->s[j] * (dt - drf->t[j-1]);

    return 0.0f;  // should never be reached
}

void
anim_menu_cb(void *menu_ref, void *item_ref)
{
    UNUSED(menu_ref);

    if (NULL == menu_sc)    // just in case
        return;

    unsigned int idx = (uint64_t)item_ref;
    SamAnim *anim = menu_sc->sam_anims[idx];

    log_msg("anim_menu_cb: label: %s, menu_item: %d", anim->label, anim->menu_item);
    now = XPLMGetDataf(total_running_time_sec_dr);

    bool reverse;

    if (anim->state == ANIM_OFF || anim->state == ANIM_ON_2_OFF) {
        XPLMCheckMenuItem(anim_menu, anim->menu_item, xplm_Menu_Checked);
        reverse = (anim->state == ANIM_ON_2_OFF);
        anim->state = ANIM_OFF_2_ON;
    } else {
        XPLMCheckMenuItem(anim_menu, anim->menu_item, xplm_Menu_Unchecked);
        reverse = (anim->state == ANIM_OFF_2_ON);
        anim->state = ANIM_ON_2_OFF;
    }

    if (reverse) {
       SamDrf *drf = sam_drfs[anim->drf_idx];
       float t_rel = now - anim->start_ts;
       float dt = drf->t[drf->n_tv - 1] - t_rel;
       anim->start_ts = now - dt;
    } else
        anim->start_ts = now;
}

static void
build_menu(Scenery* sc)
{
    log_msg("build menu for scenery %s", sc->name);
    XPLMClearAllMenuItems(anim_menu);

    for (unsigned i = 0; i < sc->sam_anims.size(); i++) {
        SamAnim *anim = sc->sam_anims[i];
        XPLMMenuCheck chk = (anim->state == ANIM_OFF || anim->state == ANIM_ON_2_OFF) ?
                                xplm_Menu_Unchecked : xplm_Menu_Checked;

        char menu_line[70];
        snprintf(menu_line, sizeof(menu_line) - 1, "%s %s", anim->label, anim->title);
        log_msg("%s", menu_line);
        anim->menu_item = XPLMAppendMenuItem(anim_menu, menu_line, (void *)(uint64_t)i, 0);
        XPLMCheckMenuItem(anim_menu, anim->menu_item, chk);
    }
}

// not much state here, just regularly check for a current scenery
// and maintain the menu
float
anim_state_machine(void)
{
    // check whether we have recently seen a scenery
    if (cur_sc && now > cur_sc_ts + 180.0f) {
        log_msg("have not seen a custom animated scenery recently");
        cur_sc = NULL;
    }

    if (cur_sc != menu_sc) {
        menu_sc = cur_sc;
        if (menu_sc)
            build_menu(menu_sc);
        else {
            log_msg("clear menu");
            XPLMClearAllMenuItems(anim_menu);
        }
    }

    return 5.0f;
}

int
anim_init()
{
    for (unsigned i = 0; i < sam_drfs.size(); i++) {
        const SamDrf *drf = sam_drfs[i];
        if (drf->autoplay)
            XPLMRegisterDataAccessor(drf->name, xplmType_Float, 0, NULL,
                                     NULL, auto_drf_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void *)drf, NULL);
        else
            XPLMRegisterDataAccessor(drf->name, xplmType_Float, 0, NULL,
                                     NULL, anim_acc, NULL, NULL, NULL, NULL, NULL, NULL,
                                     NULL, NULL, NULL, (void *)(uint64_t)i, NULL);
    }

    return 1;
}

