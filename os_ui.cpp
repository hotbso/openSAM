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

#include <cstdlib>
#include <cstdio>
#include "openSAM.h"
#include "plane.h"
#include "samjw.h"
#include "jwctrl.h"

#include "XPLMDisplay.h"
#include "XPStandardWidgets.h"

typedef struct _widget_ctx
{
    XPWidgetID widget;
    int in_vr;          /* currently in vr */
    int l, t, w, h;     /* last geometry before bringing into vr */
} widget_ctx_t;

static widget_ctx_t ui_widget_ctx;

static XPWidgetID ui_widget, jw_btn[kMaxDoor][kNearJwLimit],
    auto_btn, dock_btn, undock_btn;

int ui_unlocked; // the ui is unlocked for jw_selection

static void
show_widget(widget_ctx_t *ctx)
{
    if (XPIsWidgetVisible(ctx->widget))
        return;

    /* force window into visible area of screen
       we use modern windows under the hut so UI coordinates are in boxels */

    /* Note that (0,0) is the top left while for widgets it's bottom left
     * so we pass the y* arguments that the outcome is in widget coordinates */

    int xl, yl, xr, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, &xr, &yl);

    ctx->l = (ctx->l + ctx->w < xr) ? ctx->l : xr - ctx->w - 50;
    ctx->l = (ctx->l <= xl) ? 20 : ctx->l;

    ctx->t = (ctx->t - ctx->h > yl) ? ctx->t : (yr - ctx->h - 50);
    ctx->t = (ctx->t >= ctx->h) ? ctx->t : (yr / 2);

    log_msg("show_widget: s: (%d, %d) -> (%d, %d), w: (%d, %d) -> (%d,%d)",
           xl, yl, xr, yr, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);

    XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
    XPShowWidget(ctx->widget);

    int in_vr = XPLMGetDatai(vr_enabled_dr);
    if (in_vr) {
        log_msg("VR mode detected");
        XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
        XPLMSetWindowPositioningMode(window, xplm_WindowVR, -1);
        ctx->in_vr = 1;
    } else {
        if (ctx->in_vr) {
            log_msg("widget now out of VR, map at (%d,%d)", ctx->l, ctx->t);
            XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
            XPLMSetWindowPositioningMode(window, xplm_WindowPositionFree, -1);

            /* A resize is necessary so it shows up on the main screen again */
            XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
            ctx->in_vr = 0;
        }
    }
}

static void
close_ui()
{
    XPGetWidgetGeometry(ui_widget_ctx.widget, &ui_widget_ctx.l, &ui_widget_ctx.t, NULL, NULL);
    XPHideWidget(ui_widget_ctx.widget);
}

// static
int
MyPlane::ui_widget_cb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2)
{
    UNUSED(param1);

    if (msg == xpMessage_CloseButtonPushed) {
        close_ui();
        return 1;
    }

    unsigned n_door = my_plane->n_door_;

    if (msg == xpMsg_PushButtonPressed && widget_id == dock_btn) {
        log_msg("Dock pressed");
        if (! auto_select_jws && ui_unlocked) {
            my_plane->active_jws_.resize(0);

            for (unsigned i = 0; i < n_door; i++) {
                // check for a selected button
                for (unsigned j = 0; j < my_plane->nearest_jws_.size(); j++) {
                    int state = (uint64_t)XPGetWidgetProperty(jw_btn[i][j], xpProperty_ButtonState, NULL);
                    if (state) {
                        log_msg("active jw for door %d is %s", i, my_plane->nearest_jws_[j].jw_->name);
                        my_plane->nearest_jws_[j].door_ = i;
                        my_plane->active_jws_.push_back(my_plane->nearest_jws_[j]);
                    }
                }
            }
        }

        XPLMCommandOnce(dock_cmdr);
        close_ui();
        return 1;
    }

    if (msg == xpMsg_PushButtonPressed && widget_id == undock_btn) {
        log_msg("Undock pressed");
        XPLMCommandOnce(undock_cmdr);
        close_ui();
        return 1;
    }

    if (msg == xpMsg_ButtonStateChanged && widget_id == auto_btn) {
        auto_select_jws = (int)(uint64_t)param2;
        log_msg("auto_select_jws now: %d", auto_select_jws);

        my_plane->auto_mode_change();  // start over with new setting
        return 1;
    }

    // one of the jw select buttons
    if (msg == xpMsg_ButtonStateChanged) {
        // find index of button
        int idoor = -1, ijw = -1;
        for (unsigned i = 0; i < n_door; i++)
            for (unsigned j = 0; j < my_plane->nearest_jws_.size(); j++)
                if (jw_btn[i][j] == widget_id) {
                    idoor = i;
                    ijw = j;
                    break;
                }

        if (idoor < 0 || ijw < 0) {
            log_msg("invalid button selection???");
            return 1;
        }

        int new_state = (int)(uint64_t)param2;
        log_msg("button door: %d, jw: %d pressed, name: %s, new_state: %d", idoor, ijw,
                my_plane->nearest_jws_[ijw].jw_->name, new_state);

        // unselect all other buttons for the selected door
        for (unsigned j = 0; j < my_plane->nearest_jws_.size(); j++)
            if ((int)j != ijw)
                XPSetWidgetProperty(jw_btn[idoor][j], xpProperty_ButtonState, 0);

        // unselect selected jw for other doors
        for (unsigned i = 0; i < n_door; i++)
            if ((int)i != idoor)
                XPSetWidgetProperty(jw_btn[i][ijw], xpProperty_ButtonState, 0);

        return 1;
    }

    return 0;
}

void
MyPlane::update_ui(bool only_if_visible)
{
    if (ui_widget == NULL || (only_if_visible && !XPIsWidgetVisible(ui_widget))) {
        log_msg("update_ui: widget is not visible");
        return;
    }

    log_msg("update_ui started");
    XPSetWidgetProperty(auto_btn, xpProperty_ButtonState, auto_select_jws);

    // hide everything
    for (unsigned i = 0; i < kMaxDoor; i++)
        for (unsigned j = 0; j < kNearJwLimit; j++)
            XPHideWidget(jw_btn[i][j]);

    // if manual selection set label and unhide
    if (ui_unlocked && !auto_select_jws) {
        for (unsigned i = 0; i < n_door_; i++)
            for (unsigned j = 0; j < nearest_jws_.size(); j++) {
                JwCtrl& njw = nearest_jws_[j];
                XPWidgetID btn = jw_btn[i][j];
                XPSetWidgetDescriptor(btn, njw.jw_->name);
                XPSetWidgetProperty(btn, xpProperty_ButtonState, 0);
                XPShowWidget(btn);
            }
    }

    log_msg("update_ui finished");
}

static void
create_ui()
{
    // Note that (0,0) is the top left while for widgets it's bottom left
    // so we pass the y* arguments that the outcome is in widget coordinates

    int xl, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, NULL, NULL);

    static const int margin = 20;
    static const int col_spacing = 60;

    int left = xl + 50;
    int top = yr - 100;
    int width = 2 * margin + kMaxDoor * col_spacing;
    int height = 240;
    int left1;

    ui_widget_ctx.l = left;
    ui_widget_ctx.t = top;
    ui_widget_ctx.w = width;
    ui_widget_ctx.h = height;

    ui_widget = XPCreateWidget(left, top, left + width, top - height,
                               0, "openSAM " VERSION, 1, NULL, xpWidgetClass_MainWindow);
    ui_widget_ctx.widget = ui_widget;

    XPSetWidgetProperty(ui_widget, xpProperty_MainWindowHasCloseBoxes, 1);
    XPAddWidgetCallback(ui_widget, MyPlane::ui_widget_cb);


    top -= 20;
    left1 = left + 60;
    XPCreateWidget(left1, top, left1 + 50, top - 20,
                   1, "Jetway selection", 0, ui_widget, xpWidgetClass_Caption);

    top -= 30;
    left1 = left + margin;

    auto_btn = XPCreateWidget(left1, top, left1 + 100, top - 20, 1, "Automatic Mode", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(auto_btn, xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);
    XPAddWidgetCallback(auto_btn, MyPlane::ui_widget_cb);

    top -= 30;
    for (int i = 0; i < kMaxDoor; i++) {
        char label[50];
        sprintf(label, "Door %d", i + 1);
        XPCreateWidget(left1, top, left1 + 50, top - 20,
                                  1, label, 0, ui_widget, xpWidgetClass_Caption);
        left1 += col_spacing;
    }

    top -= 20;
    for (int j = 0; j < kNearJwLimit; j++) {
        left1 = left + margin;
        for (int i = 0; i < kMaxDoor; i++) {
             XPWidgetID btn = jw_btn[i][j] =
                XPCreateWidget(left1, top, left1 + 50, top - 20, 1, "Jw", 0, ui_widget, xpWidgetClass_Button);

            XPSetWidgetProperty(btn, xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);
            XPAddWidgetCallback(btn, MyPlane::ui_widget_cb);

            left1 += col_spacing;
        }
        top -= 20;
    }

    top -= 20;
    left1 = left + margin;
    dock_btn = XPCreateWidget(left1, top, left1 + 50, top - 20, 1, "Dock", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(dock_btn, xpProperty_ButtonType, xpPushButton);
    XPSetWidgetProperty(dock_btn, xpProperty_ButtonBehavior, xpButtonBehaviorPushButton);
    XPAddWidgetCallback(dock_btn, MyPlane::ui_widget_cb);

    left1 = left + 2 * margin + 50;
    undock_btn = XPCreateWidget(left1, top, left1 + 50, top - 20, 1, "Undock", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(undock_btn, xpProperty_ButtonType, xpPushButton);
    XPSetWidgetProperty(undock_btn, xpProperty_ButtonBehavior, xpButtonBehaviorPushButton);
    XPAddWidgetCallback(undock_btn, MyPlane::ui_widget_cb);

}

void
toggle_ui(void) {
    log_msg("toggle_ui called");

    if (ui_widget == NULL)
        create_ui();

    if (XPIsWidgetVisible(ui_widget)) {
        close_ui();
        return;
    }

    if (! my_plane->is_helicopter_) {
        my_plane->update_ui(0);
        show_widget(&ui_widget_ctx);
    }
}
