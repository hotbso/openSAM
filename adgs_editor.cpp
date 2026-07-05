//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
//
//      based on example code from imgui4xp by William Good
//      published under MIT license, see README.html for details
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

#include <string>
#include <vector>

#include "XPLMDisplay.h"
#include "XPLMProcessing.h"

#include "imgui.h"
#include "ImgWindow.h"

#include "opensam.h"
#include "autodgs_airport.h"
//#include "scenery.h"
#include "version.h"

#include "adgs_editor.h"
#include "log_msg.h"

static constexpr int kWinWidth = 400;
static constexpr int kWinHeight = 650;
static constexpr int kWinPad = 75;
//static constexpr float kFontSize = 13.0f;

std::unique_ptr<ImgWindow> editor;
int editor_left = -1, editor_top, editor_right, editor_bottom;  // -1 = not loaded from prefs
bool editor_active;

// Our own class defining our own UI
class Editor : public ImgWindow {
    int arpt_seqno_ = 0;                      // for detecting changes in the airport data
    std::vector<AdgsStandParams> lb_stands_;  // for the listbox content
    int lb_item_ = -1;                        // for the listbox selection

    XPLMFlightLoopID flt_id_ = nullptr;

    bool filter_jw_ = false;       // to store the state of the "Filter jetways" checkbox
    int params_changed_idx_ = -1;  // delayed processing in flightloop ctx

    // Main function: creates the window's UI
    void BuildInterface() override;

    // flight loop callback for delayed actions prohibited in drawloops
    static float FlightLoopCb(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter,
                              void* inRefcon);

   public:
    Editor(int left, int top, int right, int bot);
    ~Editor() override;
};

void CreateEditor() {
    if (editor_left == -1) {
        LogMsg("Creating editor window with default geometry");
        int sc_left, sc_top;
        XPLMGetScreenBoundsGlobal(&sc_left, &sc_top, nullptr, nullptr);

        editor_left = sc_left + kWinPad;
        editor_right = editor_left + kWinWidth;
        editor_top = sc_top - kWinPad;
        editor_bottom = editor_top - kWinHeight;
    } else
        LogMsg("Creating editor window with geometry %d,%d,%d,%d", editor_left, editor_top, editor_right, editor_bottom);

    editor = std::make_unique<Editor>(editor_left, editor_top, editor_right, editor_bottom);
}

///////////////////////////////////////////////////////////////////////////////////////////
Editor::Editor(int left, int top, int right, int bot)
    : ImgWindow(left, top, right, bot, xplm_WindowDecorationRoundRectangle, xplm_WindowLayerFloatingWindows) {

    // is currently not really supported, hopefully with 12.5
    ImGui::GetIO().IniFilename = strdup((user_cfg_dir + "imgui.ini").c_str());

    // Create a flight loop id, but don't schedule it yet
    XPLMCreateFlightLoop_t loop_params = {
        sizeof(loop_params),                      // structSize
        xplm_FlightLoop_Phase_BeforeFlightModel,  // phase
        FlightLoopCb,                             // callbackFunc
        (void*)this,                              // refcon
    };

    flt_id_ = XPLMCreateFlightLoop(&loop_params);

    SetWindowTitle("openSAM " VERSION);
    SetWindowResizingLimits(100, 100, 1024, 1024);
    SetVisible(true);
}

Editor::~Editor() {
    GetWindowGeometry(editor_left, editor_top, editor_right, editor_bottom);  // save geometry for next time
    if (flt_id_)
        XPLMDestroyFlightLoop(flt_id_);

    editor_active = false;
    if (adgs_arpt)
        adgs_arpt->Reset();
}

void Editor::BuildInterface() {
    ImGui::TextUnformatted("DGS Editor:");

    //float col_2 = ImGui::GetCursorPosX() + 0.6f * kFontSize * 16.0f;

    if (adgs_arpt == nullptr) {
        ImGui::TextUnformatted("No AutoDGS airport loaded");
        return;
    }

    if (arpt_seqno_ != adgs_arpt->seqno_) {
        arpt_seqno_ = adgs_arpt->seqno_;
        // Airport data has changed since last time, so we update the listbox content.
        // We do this once because it generates a lot of allocations and may be costly per frame.
        lb_stands_.clear();
        lb_stands_.reserve(adgs_arpt->nstands());
        for (int i = 0; i < adgs_arpt->nstands(); i++)
            lb_stands_.push_back(adgs_arpt->GetStandParams(i));

        lb_item_ = -1;  // reset selection
    }

    ImGui::Checkbox("Edit Mode", &editor_active);
    if (!editor_active)
        return;

    assert(editor_active);

    int height = ImGui::GetContentRegionAvail().y;
    height -= 10.0f * ImGui::GetTextLineHeightWithSpacing();

    ImGui::Checkbox("Filter jetways", &filter_jw_);

    ImGui::BeginListBox("Stands", ImVec2(-FLT_MIN, height));
    for (int i = 0; i < (int)lb_stands_.size(); i++) {
        bool is_selected = (lb_item_ == i);

        // Render the selectable item
        if (ImGui::Selectable(lb_stands_[i].name.c_str(), is_selected)) {
            is_selected = !is_selected;  // Toggle selection state
            if (is_selected)
                lb_item_ = i;  // Update selection state on click
            else
                lb_item_ = -1;  // Deselect if clicked again
        }

        // Set the initial focus when opening the combo/listbox (optional)
        if (is_selected) {
            ImGui::SetItemDefaultFocus();
        }
    }

    ImGui::EndListBox();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (lb_item_ < 0) {
        ImGui::TextUnformatted("No stand selected");
        return;
    }

    ImGui::Text("Editing Stand: %s", lb_stands_[lb_item_].name.c_str());

    ImGui::TextUnformatted("Distance");
    bool changed = false;
    float& distance = lb_stands_[lb_item_].dgs_dist;
    if (ImGui::SliderFloat("Distance", &distance, 10.0f, 50.0f, "%.1f m")) {
        LogMsg("Distance slider changed to %.1f m", distance);
        changed = true;
    }

    if (ImGui::SliderFloat("Height", &lb_stands_[lb_item_].dgs_height, 1.0f, 10.0f, "%.1f m")) {
        LogMsg("Height slider changed to %.1f m", lb_stands_[lb_item_].dgs_height);
        changed = true;
    }

    if (changed) {
        params_changed_idx_ = lb_item_;  // delayed processing in flightloop ctx
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
    }

#if 0

    if (ImGui::TreeNode("Settings")) {
        //--------------------------------------------------
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextUnformatted("Default VDGS for standard airports:");

        int current_vdgs_type = default_vdgs_type;
        if (ImGui::RadioButton("Safedock T2-24", default_vdgs_type == kVdgsSafedock_T2_24))
            default_vdgs_type = kVdgsSafedock_T2_24;

        if (ImGui::RadioButton("Safedock-X", default_vdgs_type == kVdgsSafedock_X))
            default_vdgs_type = kVdgsSafedock_X;

        if (default_vdgs_type != current_vdgs_type)
            LogMsg("Default VDGS type changed to %d", default_vdgs_type);

        //--------------------------------------------------
        ImGui::Spacing();
        ImGui::Separator();
        int radio = Seasons::auto_season ? 4 : Seasons::season;
        ImGui::TextUnformatted("SAM legacy Season: ");
        ImGui::SameLine();
        if (ImGui::RadioButton("Auto", radio == 4))
            radio = 4;

        if (ImGui::RadioButton("Spring", radio == 0))
            radio = 0;
        ImGui::SameLine();
        if (ImGui::RadioButton("Summer", radio == 1))
            radio = 1;
        ImGui::SameLine();
        if (ImGui::RadioButton("Autumn", radio == 2))
            radio = 2;
        ImGui::SameLine();
        if (ImGui::RadioButton("Winter", radio == 3))
            radio = 3;

        if (radio == 4) {
            Seasons::auto_season = true;
        } else {
            Seasons::auto_season = false;
            Seasons::season = radio;
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("DGS state:");
    ImGui::SameLine();
    ImGui::SetCursorPosX(col_2);
    ImVec4 red = ImColor(255, 0, 0, 255);
    ImGui::PushStyleColor(ImGuiCol_Text, red);
    ImGui::TextUnformatted(dgs_arpt->state_str());
    ImGui::PopStyleColor();

    if (dgs_arpt->state() == dgs::Airport::kIdle) {
        int pax_no = my_plane->PaxNo();
        if (pax_no > 0) {
            ImGui::SameLine();
            ImGui::Text("  (Pax on board: %d)", pax_no);
        }
    }

    int as = dgs_arpt->active_stand();
    int ss = dgs_arpt->selected_stand();
    if (as >= 0) {
        const auto& stand = dgs_arpt->stand(as);
        ImGui::TextUnformatted("Active stand:");
        ImGui::SameLine();
        ImGui::SetCursorPosX(col_2);
        ImGui::Text("'%s'", stand.name().c_str());
    }

    if (ss >= 0 && ss != as) {
        const auto& stand = dgs_arpt->stand(ss);
        ImGui::TextUnformatted("Selected stand:");
        ImGui::SameLine();
        ImGui::SetCursorPosX(col_2);
        ImGui::Text("'%s'", stand.name().c_str());
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (dgs_arpt->state() == dgs::Airport::kIdle && my_plane->beacon_on()) {
        if (ImGui::Button("Set mode ARRIVAL")) {
            set_mode_arrival_requested_ = true;
            XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
        }
        return;
    }

    // for AutoDGS we have additional buttons to interact with the DGS
    // show them as long as we have an active stand
    if (adgs_arpt && as >= 0) {
        auto [dgs_type, name] = adgs_arpt->GetStand(as);
        int idx = as + 1;                                            // +1 due to "<automatic>"
        lb_stands_[idx][0] = (dgs_type == kMarshaller ? 'M' : 'V');  // set current indicator

        new_dgs_type_ = dgs_type;

        ImGui::Columns(2);  // 2 columns: one for the radio buttons, one for the "Move closer" button
        if (ImGui::RadioButton("Marshaller", new_dgs_type_ == 0)) {
            new_dgs_type_ = 0;
        }

        ImGui::NextColumn();  // column 2 for the button
        if (ImGui::Button("Move closer")) {
            move_closer_requested_ = true;
            XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
        }

        ImGui::NextColumn();  // column 1 for the radio button
        if (ImGui::RadioButton("VDGS", new_dgs_type_ == 1)) {
            new_dgs_type_ = 1;
        }
        ImGui::Columns();

        if (new_dgs_type_ != dgs_type) {
            new_dgs_type_stand_ = as;
            params_changed = true;
            XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
            LogMsg("Flight loop scheduled to apply new DGS type %d for active stand index %d", new_dgs_type_, as);
        }
    }

    if (dgs::Airport::kArrival <= dgs_arpt->state() && dgs_arpt->state() <= dgs::Airport::kBad) {
        lb_item_ = dgs_arpt->selected_stand() + 1;  // +1 due to "<automatic>"

        int height = ImGui::GetContentRegionAvail().y;
        if (os_arpt)
            height -= ImGui::GetTextLineHeightWithSpacing();    // jw selection is below the stand listbox

        ImGui::BeginListBox("Stands", ImVec2(-FLT_MIN, height));
        for (int i = 0; i < (int)lb_stands_.size(); i++) {
            const bool is_selected = (lb_item_ == i);

            // Render the selectable item
            if (ImGui::Selectable(lb_stands_[i].c_str(), is_selected)) {
                lb_item_ = i;  // Update selection state on click
                new_selected_stand_ = i - 1;
                selected_stand_changed_ = true;
                XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
                LogMsg("Flight loop scheduled to apply new selected stand index %d (listbox index %d)",
                       new_selected_stand_, lb_item_);
            }

            // Set the initial focus when opening the combo/listbox (optional)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndListBox();
    }

    if (os_arpt == nullptr)
        return;

    // openSAM jetway UI
    jw_auto_mode_ = my_plane->auto_mode();

    ImGui::TextUnformatted("Jetway selection mode:");
    ImGui::SameLine();
    if (ImGui::RadioButton("Automatic", jw_auto_mode_))
        jw_auto_mode_ = true;

    ImGui::SameLine();
    if (ImGui::RadioButton("Manual", !jw_auto_mode_))
        jw_auto_mode_ = false;

    if (jw_auto_mode_ != my_plane->auto_mode()) {
        jw_auto_mode_changed_ = true;
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
    }

    if (my_plane->state() == Plane::kSelectJws) {
        if (nearest_jws_seqno_ != my_plane->nearest_jws_seqno_) {
            // nearest jetway list has changed since last time, so we reset the selection
            memset(jw_selected_, 0, sizeof(jw_selected_));
            nearest_jws_seqno_ = my_plane->nearest_jws_seqno_;
        }

        int n_jws = my_plane->nearest_jws_.size();
        int n_doors = my_plane->door_info_.size();

        ImGui::Spacing();
        ImGui::Columns(n_doors + 1);
        ImGui::NextColumn();
        for (int d = 0; d < n_doors; d++) {
            ImGui::Text("Door %d", d + 1);
            ImGui::NextColumn();
        }
        ImGui::Separator();
        int id = 0;
        for (int j = 0; j < n_jws; j++) {
            ImGui::TextUnformatted(my_plane->nearest_jws_[j].name());
            ImGui::NextColumn();
            for (int d = 0; d < n_doors; d++) {
                ImGui::PushID(id++);  // ensure unique ID for each checkbox
                bool selected = jw_selected_[j][d];
                if (ImGui::Checkbox("", &selected)) {
                    jw_selected_[j][d] = selected;
                    LogMsg("JW %d door %d selection changed to %s", j + 1, d + 1,
                           jw_selected_[j][d] ? "SELECTED" : "DESELECTED");

                    if (selected) {
                        for (int d1 = 0; d1 < n_doors; d1++)
                            if (d1 != d)
                                jw_selected_[j][d1] = false;  // only one door can be selected per JW

                        for (int j1 = 0; j1 < n_jws; j1++)
                            if (j1 != j)
                                jw_selected_[j1][d] = false;  // only one JW can be selected per door
                    }
                }
                ImGui::PopID();
                ImGui::NextColumn();
            }
        }
        ImGui::Columns();

        ImGui::Separator();
        ImGui::Spacing();
        bool have_selection = false;
        for (int i = 0; i < n_doors; i++) {
            for (int j = 0; j < n_jws; j++) {
                if (jw_selected_[j][i]) {
                    have_selection = true;
                    break;
                }
            }
        }

        if (have_selection) {
            if (ImGui::Button("Dock")) {
                // make selected jetways active for docking, and fire a docking request.
                my_plane->active_jws_.clear();
                for (int i = 0; i < n_doors; i++) {
                    for (int j = 0; j < n_jws; j++) {
                        if (jw_selected_[j][i]) {
                            auto& njw = my_plane->nearest_jws_[j];
                            njw.selected_ = true;
                            njw.door_ = i;
                            my_plane->active_jws_.push_back(j);
                            LogMsg("JW %d door %d selected for docking", j + 1, i + 1);
                        }
                    }
                }
                my_plane->dock_requested_ = true;
            }
        }
    } else if (my_plane->state() == Plane::kCanDock) {
        int n_jws = my_plane->nearest_jws_.size();
        int n_doors = my_plane->door_info_.size();

        ImGui::Spacing();
        ImGui::Columns(n_doors + 1);
        ImGui::NextColumn();
        for (int d = 0; d < n_doors; d++) {
            ImGui::Text("Door %d", d + 1);
            ImGui::NextColumn();
        }
        ImGui::Separator();
        for (int j = 0; j < n_jws; j++) {
            ImGui::TextUnformatted(my_plane->nearest_jws_[j].name());
            ImGui::NextColumn();
            for (int d = 0; d < n_doors; d++) {
                for (int ajw_idx : my_plane->active_jws_) {
                    if (ajw_idx == j && my_plane->nearest_jws_[j].door_ == d) {
                        ImGui::TextUnformatted((const char*)ICON_FA_CHECK);
                        ImGui::SameLine();
                        break;
                    }
                }
                ImGui::NextColumn();
            }
        }
        ImGui::Columns();

        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button("Dock"))
            my_plane->dock_requested_ = true;
    } else if (my_plane->state() == Plane::kCantDock) {
        ImGui::TextUnformatted("Cannot dock: no suitable jetways found!");
    } else if (my_plane->state() == Plane::kDocking) {
        ImGui::TextUnformatted("Docking in progress...");
    } else if (my_plane->state() == Plane::kUndocking) {
        ImGui::TextUnformatted("Undocking in progress...");
    } else if (my_plane->state() == Plane::kDocked) {
        if (ImGui::Button("Undock"))
            my_plane->undock_requested_ = true;
    }
#endif
}

// Delayed actions that require FlightLoop context
float Editor::FlightLoopCb(float, float, int, void* inRefcon) {
    LogMsg("FlightLoopCb called with inRefcon=%p", inRefcon);

    Editor& editor = *reinterpret_cast<Editor*>(inRefcon);

    if (adgs_arpt == nullptr || adgs_arpt->seqno_ != editor.arpt_seqno_) {
        // stale request
        editor.params_changed_idx_ = -1;
        return 0.0f;
    }

    if (editor.params_changed_idx_ != -1) {
        LogMsg("Changing DGS params of stand index %d", editor.params_changed_idx_);
        adgs_arpt->SetStandParams(editor.params_changed_idx_, editor.lb_stands_[editor.params_changed_idx_]);
        editor.lb_stands_[editor.params_changed_idx_] = adgs_arpt->GetStandParams(editor.params_changed_idx_);
        editor.params_changed_idx_ = -1;
    }

    return 0.0f;
}
