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
#include "version.h"

#include "ui.h"
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

    ImGuiSelectionBasicStorage selection_storage_;  // for the listbox selection

    // bool filter_jw_ = false;       // to store the state of the "Filter jetways" checkbox
    std::vector<int> selected_idx_;  // for processing multiple selected stands

    // Main function: creates the window's UI, runs in flightloop ctx
    void BuildInterface() override;

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

    ImGuiStyle& style = ImGui::GetStyle();
    style.FontSizeBase = kFontSize;

    SetWindowTitle("openSAM Airport Editor");
    SetWindowResizingLimits(100, 100, 1024, 1024);
    SetVisible(true);
    LogMsg("Editor window created");
}

Editor::~Editor() {
    GetWindowGeometry(editor_left, editor_top, editor_right, editor_bottom);  // save geometry for next time
    editor_active = false;  // clear the global flag
    if (adgs_arpt)
        adgs_arpt->SetEditorMode(false);
}

void Editor::BuildInterface() {
    if (adgs_arpt == nullptr) {
        ImGui::TextUnformatted("No AutoDGS airport loaded");
        return;
    }

    bool was_active = editor_active;
    if (ImGui::Checkbox("Edit Mode", &editor_active)) {
        LogMsg("Edit Mode checkbox changed to %s", editor_active ? "ON" : "OFF");
        selection_storage_.Clear();  // clear selection when switching modes
        LogMsg("Setting edit mode in FlightLoop context");
        adgs_arpt->SetEditorMode(editor_active);
    }

    if (!editor_active)
        return;

    // if the airport changed or we switched to active we have to rebuild the listbox content
    if (arpt_seqno_ != adgs_arpt->seqno_ || !was_active) {
        LogMsg("Airport data changed or editor activated, rebuilding listbox content");
        arpt_seqno_ = adgs_arpt->seqno_;
        // Airport data has changed since last time, so we update the listbox content.
        // We do this once because it generates a lot of allocations and may be costly per frame.
        lb_stands_.clear();
        lb_stands_.reserve(adgs_arpt->nstands());
        for (int i = 0; i < adgs_arpt->nstands(); i++)
            lb_stands_.push_back(adgs_arpt->GetStandParams(i));
        selected_idx_.clear();
    }

    int height = ImGui::GetContentRegionAvail().y;
    height -= 10.0f * ImGui::GetTextLineHeightWithSpacing();

    //ImGui::Checkbox("Filter jetways", &filter_jw_);
    bool selection_changed = false;
    if (ImGui::BeginListBox("##Stands", ImVec2(-FLT_MIN, height))) {
        ImGuiMultiSelectIO* ms_io =
            ImGui::BeginMultiSelect(ImGuiMultiSelectFlags_None, selection_storage_.Size, (int)lb_stands_.size());
        selection_storage_.ApplyRequests(ms_io);

        for (int i = 0; i < (int)lb_stands_.size(); i++) {
            ImGui::PushID(i);  // Ensure unique ID for each item, stand names may have duplicates
            bool is_selected = selection_storage_.Contains((ImGuiID)i);

            // Tell ImGui that this widget corresponds to index 'i'
            ImGui::SetNextItemSelectionUserData((ImGuiSelectionUserData)i);

            // Render the selectable item
            if (ImGui::Selectable(lb_stands_[i].name.c_str(), is_selected)) {
                // imgui magic
                selection_changed = true;
            }

            // Set the initial focus when opening the combo/listbox (optional)
            // if (is_selected) {
            //    ImGui::SetItemDefaultFocus();
            //}
            ImGui::PopID();
        }
        // Capture mouse clicks/drags from this frame and apply them
        ms_io = ImGui::EndMultiSelect();
        selection_storage_.ApplyRequests(ms_io);
        ImGui::EndListBox();
    }

    if (ImGui::Button("Clear Selection")) {
        selection_storage_.Clear();
        selection_changed = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("Select VDGS")) {
        selection_storage_.Clear();
        for (int i = 0; i < (int)lb_stands_.size(); i++) {
            if (lb_stands_[i].dgs_type != kMarshaller)
                selection_storage_.SetItemSelected((ImGuiID)i, true);
        }
        selection_changed = true;
    }

    if (selection_changed) {
        LogMsg("Listbox selection changed, selected items: %d", selection_storage_.Size);
        selected_idx_.clear();
        void* it = NULL;
        ImGuiID id;
        while (selection_storage_.GetNextSelectedItem(&it, &id)) {
            selected_idx_.push_back((int)id);
            //LogMsg("Selected stand index: %d", (int)id);
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (selection_storage_.Size == 0) {
        ImGui::TextUnformatted("No stand selected");
        return;
    }

    auto cur_params = lb_stands_[selected_idx_[0]];  // use the first selected stand as template for editing
    if (selection_storage_.Size > 1)
        ImGui::Text("Editing %d stands, using first selected as template", selection_storage_.Size);
    else
        ImGui::Text("Editing stand: %s", cur_params.name.c_str());

    int dgs_type = cur_params.dgs_type;
    bool pole = cur_params.pole;
    float dgs_dist = cur_params.dgs_dist;
    float dgs_height = cur_params.dgs_height;
    float dgs_left_right = cur_params.dgs_left_right;

    if (ImGui::RadioButton("default VDGS", dgs_type == kDefaultVDGS))
        dgs_type = kDefaultVDGS;

    ImGui::SameLine();
    if (ImGui::RadioButton("Marshaller", dgs_type == kMarshaller)) {
        dgs_type = kMarshaller;
        pole = false;   // no stairs by default for marshaller
    }

    ImGui::SameLine();
    if (ImGui::RadioButton("Safedock-T2", dgs_type == kVdgsSafedock_T2_24))
        dgs_type = kVdgsSafedock_T2_24;

    ImGui::SameLine();
    if (ImGui::RadioButton("Safedock-X", dgs_type == kVdgsSafedock_X))
        dgs_type = kVdgsSafedock_X;

    if (dgs_type == kVdgsSafedock_T2_24 || dgs_type == kVdgsSafedock_X)
        ImGui::Checkbox("Pole", &pole);
    else if (dgs_type == kMarshaller)
        ImGui::Checkbox("Stairs", &pole);  // for marshaller, pole = stairs

    if (dgs_type != cur_params.dgs_type || pole != cur_params.pole) {
        for (int idx : selected_idx_) {
            LogMsg("Changing DGS type of stand index %d to %d (pole=%s)", idx, dgs_type,
                   pole ? "true" : "false");
            adgs_arpt->SetDgsType(idx, dgs_type, pole);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
    }

    if (ImGui::SliderFloat("Distance", &dgs_dist, 8.0f, 50.0f, "%.1f m")) {
        for (int idx : selected_idx_) {
            LogMsg("Changing DGS distance of stand index %d to %.1f m", idx, dgs_dist);
            adgs_arpt->SetDgsDistance(idx, dgs_dist);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
    }

    if (cur_params.dgs_type != kMarshaller)
        if (ImGui::SliderFloat("Height", &dgs_height, 1.0f, 10.0f, "%.1f m")) {
            for (int idx : selected_idx_) {
                LogMsg("Changing DGS height of stand index %d to %.1f m", idx, dgs_height);
                adgs_arpt->SetDgsHeight(idx, dgs_height);
                lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
            }
        }

    ImGui::SliderFloat("Left/Right", &dgs_left_right, -10.0f, 10.0f, "%.1f m");

    ImGui::SameLine();
    if (ImGui::Button("Center"))
        dgs_left_right = 0.0f;

    if (dgs_left_right != cur_params.dgs_left_right)
        for (int idx : selected_idx_) {
            LogMsg("Changing DGS left/right of stand index %d to %.1f m", idx, dgs_left_right);
            adgs_arpt->SetDgsLeftRight(idx, dgs_left_right);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
}
