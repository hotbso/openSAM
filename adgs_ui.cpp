//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023, 2025 Holger Teutsch
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

#include <cstring>
#include <cassert>

#include "opensam.h"
#include "autodgs_airport.h"
#include <my_plane.h>
#include "dgs/plane.h"

#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include "XPListBox.h"

#include "widget_ctx.h"
#include "version.h"

static WidgetCtx ui_widget_ctx;

static XPWidgetID ui_widget, list_box, status_line, marshaller_label, vdgs_label, marshaller_btn, vdgs_btn,
    activate_btn, move_btn;

// current status of ui
static std::string ui_arpt_icao;
static int ui_selected_stand;

static inline bool IsActive() {
    return (adgs_arpt && adgs_arpt->state() >= AdgsAirport::ARRIVAL);
}

static void ShowTypeButtons() {
    XPShowWidget(vdgs_label);
    XPShowWidget(marshaller_label);
    XPShowWidget(vdgs_btn);
    XPShowWidget(marshaller_btn);

    XPShowWidget(move_btn);
    XPHideWidget(activate_btn);
}

static void HideTypeButtons() {
    XPHideWidget(vdgs_label);
    XPHideWidget(marshaller_label);
    XPHideWidget(vdgs_btn);
    XPHideWidget(marshaller_btn);

    XPHideWidget(move_btn);
}

static int WidgetCb(XPWidgetMessage msg, XPWidgetID widget_id, [[maybe_unused]] intptr_t param1,
                    [[maybe_unused]] intptr_t param2) {
    if (msg == xpMessage_CloseButtonPushed) {
        ui_widget_ctx.Hide();
        return 1;
    }

    if (msg == xpMsg_PushButtonPressed) {
        if (widget_id == activate_btn) {
            if (!(dgs::plane->BeaconOn() && my_plane->on_ground()))
                XPSetWidgetDescriptor(status_line, "Beacon off or not on ground");
            else if (adgs_arpt)
                adgs_arpt->SetArrival();
        } else if (widget_id == move_btn && IsActive())
            adgs_arpt->DgsMoveCloser();

        return 1;
    }

    if (msg == xpMessage_ListBoxItemSelected) {
        ui_selected_stand = XPGetWidgetProperty(list_box, xpProperty_ListBoxCurrentItem, NULL);
        assert(ui_selected_stand >= 0);  // be paranoid
        ui_selected_stand--;             // 0 is "Automatic"

        if (ui_selected_stand < 0) {
            HideTypeButtons();
            std::string txt = "Automatic @ " + (ui_arpt_icao.empty() ? "unknown" : ui_arpt_icao);
            XPSetWidgetDescriptor(status_line, txt.c_str());
            if (IsActive())
                adgs_arpt->SetSelectedStand(-1);  // unselect
            return 1;
        }

        char ss_name[100];
        ss_name[99] = '\0';
        XPGetWidgetDescriptor(list_box, ss_name, sizeof(ss_name) - 1);
        std::string txt{ss_name + 2};  // skip type, blank
        LogMsg("selected ramp is '%s'", txt.c_str());
        txt += " @ " + (ui_arpt_icao.empty() ? "unknown" : ui_arpt_icao);
        XPSetWidgetDescriptor(status_line, txt.c_str());

        ShowTypeButtons();
        if (IsActive()) {  // be paranoid, otherwise we should never be here
            adgs_arpt->SetSelectedStand(ui_selected_stand);
            int dgs_type = adgs_arpt->GetDgsType();
            XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 0);
            XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 0);
            if (dgs_type == kMarshaller)
                XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);
            else
                XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 1);
        }

        return 1;
    }

    if (msg != xpMsg_ButtonStateChanged)
        return 0;

    // From here on the radio buttons only

    // radio buttons get this message only when they are clicked to "selected"
    int dgs_type = -1;
    if (widget_id == marshaller_btn) {
        dgs_type = kMarshaller;
        XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, 0);
    } else if (widget_id == vdgs_btn) {
        dgs_type = kVDGS;
        XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 0);
    }

    assert(dgs_type != -1);

    if (IsActive()) {
        adgs_arpt->SetDgsType(dgs_type);
        int selected_stand = XPGetWidgetProperty(list_box, xpProperty_ListBoxCurrentItem, NULL);
        selected_stand--;  // 0 is "Automatic"
        if (selected_stand >= 0) {
            auto [dgs_type, name] = adgs_arpt->GetStand(selected_stand);
            char entry[100];
            snprintf(entry, sizeof(entry), "%c %s", (dgs_type == kMarshaller ? 'M' : 'V'), name.c_str());
            // currently there is no replace op
            XPSetWidgetProperty(list_box, xpProperty_ListBoxDeleteItem, 1);
            XPSetWidgetDescriptor(list_box, entry);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxInsertItem, 1);
        }
    }

    return 1;
}

void AdgsUpdateUI(bool only_if_visible) {
    if (ui_widget == NULL || (only_if_visible && !XPIsWidgetVisible(ui_widget))) {
        LogMsg("update_ui: widget is not visible");
        return;
    }

    LogMsg("update_ui started");

    if (!IsActive()) {
        ui_selected_stand = -1;
        XPSetWidgetProperty(list_box, xpProperty_ListBoxClear, 1);
        ui_arpt_icao.clear();
        HideTypeButtons();
        XPShowWidget(activate_btn);
        XPSetWidgetDescriptor(status_line, "");
        return;
    }

    // active and newly loaded
    if (adgs_arpt->name() != ui_arpt_icao) {
        LogMsg("airport changed to %s", adgs_arpt->name().c_str());
        ui_arpt_icao = adgs_arpt->name();
        XPHideWidget(activate_btn);
        LogMsg("load ramps");
        ui_selected_stand = adgs_arpt->selected_stand(); // preselected via OFP or -1
        XPSetWidgetDescriptor(list_box, "Automatic");
        XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItemsWithClear, 1);

        for (int i = 0; i < adgs_arpt->nstands(); i++) {
            auto [dgs_type, name] = adgs_arpt->GetStand(i);
            char entry[100];
            snprintf(entry, sizeof(entry), "%c %s", (dgs_type == kMarshaller ? 'M' : 'V'), name.c_str());
            XPSetWidgetDescriptor(list_box, entry);
            XPSetWidgetProperty(list_box, xpProperty_ListBoxAddItem, 1);
        }
    }

    // that's cheap so we do it always
    if (ui_selected_stand == -1) {
        XPHideWidget(activate_btn);
        HideTypeButtons();
        std::string txt = "Automatic @ " + ui_arpt_icao;
        XPSetWidgetDescriptor(status_line, txt.c_str());
    } else if (IsActive()) {
        ShowTypeButtons();
        auto [dgs_type, name] = adgs_arpt->GetStand(ui_selected_stand);
        XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, (dgs_type == kMarshaller ? 1 : 0));
        XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonState, (dgs_type == kVDGS ? 1 : 0));
        std::string txt = name + " @ " + ui_arpt_icao;
        XPSetWidgetDescriptor(status_line, txt.c_str());
    }
}

static void CreateUI() {
    // Note that (0,0) is the top left while for widgets it's bottom left
    // so we pass the y* arguments that the outcome is in widget coordinates

    int xl, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, NULL, NULL);

    int left = xl + 50;
    int top = yr - 100;
    int width = 250;
    int height = 450;
    int lb_height = 350;

    ui_widget = XPCreateWidget(left, top, left + width, top - height, 0, "AutoDGS " VERSION_SHORT, 1, NULL,
                               xpWidgetClass_MainWindow);
    ui_widget_ctx.Set(ui_widget, left, top, width, height);

    XPSetWidgetProperty(ui_widget, xpProperty_MainWindowHasCloseBoxes, 1);
    XPAddWidgetCallback(ui_widget, WidgetCb);
    left += 5;
    int left1 = left + 60;

    top -= 20;
    int top_btn = top - 20;
    int left_btn = left + (width - 60) / 2;
    // "Activate" button
    activate_btn = XPCreateWidget(left_btn, top_btn, left_btn + 60, top_btn - 20, 1, "Activate", 0, ui_widget,
                                  xpWidgetClass_Button);
    XPSetWidgetProperty(activate_btn, xpProperty_ButtonType, xpPushButton);
    XPSetWidgetProperty(activate_btn, xpProperty_ButtonBehavior, xpButtonBehaviorPushButton);
    XPAddWidgetCallback(activate_btn, WidgetCb);

    // "Move closer" button
    left_btn = left1 + 60;
    move_btn = XPCreateWidget(left_btn, top_btn, left_btn + 80, top_btn - 20, 1, "Move closer", 0, ui_widget,
                              xpWidgetClass_Button);
    XPSetWidgetProperty(move_btn, xpProperty_ButtonType, xpPushButton);
    XPSetWidgetProperty(move_btn, xpProperty_ButtonBehavior, xpButtonBehaviorPushButton);
    XPAddWidgetCallback(move_btn, WidgetCb);

    // Type radio buttons
    marshaller_label =
        XPCreateWidget(left, top, left + 50, top - 20, 1, "Marshaller", 0, ui_widget, xpWidgetClass_Caption);
    marshaller_btn = XPCreateWidget(left1, top, left1 + 20, top - 20, 1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(marshaller_btn, WidgetCb);
    XPSetWidgetProperty(marshaller_btn, xpProperty_ButtonState, 1);

    top -= 20;
    vdgs_label = XPCreateWidget(left, top, left + 50, top - 20, 1, "VDGS", 0, ui_widget, xpWidgetClass_Caption);
    vdgs_btn = XPCreateWidget(left1, top, left1 + 20, top - 20, 1, "", 0, ui_widget, xpWidgetClass_Button);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(vdgs_btn, xpProperty_ButtonBehavior, xpButtonBehaviorRadioButton);
    XPAddWidgetCallback(vdgs_btn, WidgetCb);

    top -= 20;
    status_line = XPCreateWidget(left, top, left + width - 30, top - 20, 1, "", 0, ui_widget, xpWidgetClass_Caption);

    top -= 30;
    list_box = XPCreateListBox(left, top, left + width - 10, top - lb_height, 1, "Automatic", ui_widget);
}

void AdgsToggleUI(void) {
    LogMsg("AdgsToggleUI called");

    if (ui_widget == NULL)
        CreateUI();

    if (XPIsWidgetVisible(ui_widget)) {
        ui_widget_ctx.Hide();
        return;
    }

    AdgsUpdateUI(false);
    ui_widget_ctx.Show();
}
