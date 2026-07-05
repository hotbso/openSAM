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

    //bool filter_jw_ = false;       // to store the state of the "Filter jetways" checkbox
    int params_changed_idx_ = -1;  // delayed processing in flightloop ctx
    bool request_set_edit_mode_ = false;  // delayed processing in flightloop ctx

    // Main function: creates the window's UI
    void BuildInterface() override;

    void ProcessChangedParams();  // process changed DGS params in flightloop context

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
    ImGui::GetIO().IniFilename = nullptr;  // disable imgui.ini file, it's not compatible with imWindow

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

    editor_active = false;  // clear the global flag
    if (adgs_arpt)
        adgs_arpt->SetEditorMode(false);
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

    if (ImGui::Checkbox("Edit Mode", &editor_active)) {
        LogMsg("Edit Mode checkbox changed to %s", editor_active ? "ON" : "OFF");
        request_set_edit_mode_ = true;  // request to set the edit mode in the flight loop context
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);  // schedule the flight loop callback to process the request
    }

    if (!editor_active)
        return;

    int height = ImGui::GetContentRegionAvail().y;
    height -= 10.0f * ImGui::GetTextLineHeightWithSpacing();

    //ImGui::Checkbox("Filter jetways", &filter_jw_);

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
}

void Editor::ProcessChangedParams() {
    if (adgs_arpt == nullptr || adgs_arpt->seqno_ != arpt_seqno_) {
        // stale request
        params_changed_idx_ = -1;
        request_set_edit_mode_ = false;
        return;
    }

    if (params_changed_idx_ != -1) {
        LogMsg("Changing DGS params of stand index %d", params_changed_idx_);
        adgs_arpt->SetStandParams(params_changed_idx_, lb_stands_[params_changed_idx_]);
        lb_stands_[params_changed_idx_] = adgs_arpt->GetStandParams(params_changed_idx_);
        params_changed_idx_ = -1;
    }

    if (request_set_edit_mode_) {
        LogMsg("Setting edit mode in FlightLoop context");
        adgs_arpt->SetEditorMode(editor_active);
        request_set_edit_mode_ = false;
    }
}

// Delayed actions that require FlightLoop context
float Editor::FlightLoopCb(float, float, int, void* inRefcon) {
    reinterpret_cast<Editor*>(inRefcon)->ProcessChangedParams();
    return 0.0f;
}
