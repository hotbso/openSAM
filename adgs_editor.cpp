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

    ImGuiSelectionBasicStorage selection_storage_;  // for the listbox selection
    XPLMFlightLoopID flt_id_ = nullptr;

    // bool filter_jw_ = false;       // to store the state of the "Filter jetways" checkbox
    std::vector<int> selected_idx_;  // for processing multiple selected stands

    // background processing in flightloop context
    AdgsStandParams changed_sp_;
    std::vector<int> changed_idx_;  // for processing multiple selected stands
    bool request_set_edit_mode_ = false;
    bool request_set_dgs_type_ = false;
    bool request_set_dgs_dist_ = false;
    bool request_set_dgs_height_ = false;
    bool request_set_dgs_left_right_ = false;

    // Main function: creates the window's UI
    void BuildInterface() override;

    void ProcessChangedParams();  // process changed DGS params in flightloop context

    // flight loop callback for background actions prohibited in drawloops
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

    SetWindowTitle("openSAM Airport Editor");
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
    //float col_2 = ImGui::GetCursorPosX() + 0.6f * kFontSize * 16.0f;

    if (adgs_arpt == nullptr) {
        ImGui::TextUnformatted("No AutoDGS airport loaded");
        return;
    }

    bool was_active = editor_active;
    if (ImGui::Checkbox("Edit Mode", &editor_active)) {
        LogMsg("Edit Mode checkbox changed to %s", editor_active ? "ON" : "OFF");
        selection_storage_.Clear();  // clear selection when switching modes
        request_set_edit_mode_ = true;  // request to set the edit mode in the flight loop context
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);  // schedule the flight loop callback to process the request
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
        changed_idx_.clear();
        selected_idx_.clear();
    }

    int height = ImGui::GetContentRegionAvail().y;
    height -= 10.0f * ImGui::GetTextLineHeightWithSpacing();

    //ImGui::Checkbox("Filter jetways", &filter_jw_);
    bool selection_changed = false;
    if (ImGui::BeginListBox("Stands", ImVec2(-FLT_MIN, height))) {
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

    changed_sp_ = lb_stands_[selected_idx_[0]];  // use the first selected stand as template for editing
    if (selection_storage_.Size > 1)
        ImGui::Text("Editing %d stands, using first selected as template", selection_storage_.Size);
    else
        ImGui::Text("Editing stand: %s", changed_sp_.name.c_str());

    int dgs_type = changed_sp_.dgs_type;
    bool pole = changed_sp_.pole;
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

    if (dgs_type != changed_sp_.dgs_type || pole != changed_sp_.pole) {
        changed_sp_.dgs_type = dgs_type;
        changed_sp_.pole = pole;
        changed_idx_ = selected_idx_;  // delayed processing in flightloop ctx
        request_set_dgs_type_ = true;
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
    }

    if (ImGui::SliderFloat("Distance", &changed_sp_.dgs_dist, 8.0f, 50.0f, "%.1f m"))
        request_set_dgs_dist_ = true;

    if (changed_sp_.dgs_type != kMarshaller)
        if (ImGui::SliderFloat("Height", &changed_sp_.dgs_height, 1.0f, 10.0f, "%.1f m"))
            request_set_dgs_height_ = true;

    if (ImGui::SliderFloat("Left/Right", &changed_sp_.dgs_left_right, -10.0f, 10.0f, "%.1f m"))
        request_set_dgs_left_right_ = true;

    ImGui::SameLine();
    if (ImGui::Button("Center")) {
        changed_sp_.dgs_left_right = 0.0f;
        request_set_dgs_left_right_ = true;
    }

    if (request_set_dgs_dist_ || request_set_dgs_height_ || request_set_dgs_left_right_) {
        changed_idx_ = selected_idx_;  // delayed processing in flightloop ctx
        XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// background processing in flightloop context
void Editor::ProcessChangedParams() {
    if (adgs_arpt == nullptr || adgs_arpt->seqno_ != arpt_seqno_) {
        // stale request
        changed_idx_.clear();
        request_set_edit_mode_ = request_set_dgs_type_ = request_set_dgs_dist_ = request_set_dgs_height_ = false;
        return;
    }

    if (request_set_edit_mode_) {
        LogMsg("Setting edit mode in FlightLoop context");
        adgs_arpt->SetEditorMode(editor_active);
        request_set_edit_mode_ = false;
        return;
    }

    for (int idx : changed_idx_)
        assert(0 <= idx && idx < adgs_arpt->nstands());

    if (request_set_dgs_dist_) {
        for (int idx : changed_idx_) {
            LogMsg("Changing DGS distance of stand index %d to %.1f m", idx, changed_sp_.dgs_dist);
            adgs_arpt->SetDgsDistance(idx, changed_sp_.dgs_dist);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
        changed_idx_.clear();
        request_set_dgs_dist_ = false;
        return;
    }

    if (request_set_dgs_left_right_) {
        for (int idx : changed_idx_) {
            LogMsg("Changing DGS left/right of stand index %d to %.1f m", idx, changed_sp_.dgs_left_right);
            adgs_arpt->SetDgsLeftRight(idx, changed_sp_.dgs_left_right);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
        changed_idx_.clear();
        request_set_dgs_left_right_ = false;
        return;
    }

    if (request_set_dgs_height_) {
        for (int idx : changed_idx_) {
            LogMsg("Changing DGS height of stand index %d to %.1f m", idx, changed_sp_.dgs_height);
            adgs_arpt->SetDgsHeight(idx, changed_sp_.dgs_height);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
        changed_idx_.clear();
        request_set_dgs_height_ = false;
        return;
    }

    if (request_set_dgs_type_) {
        for (int idx : changed_idx_) {
            LogMsg("Changing DGS type of stand index %d to %d (pole=%s)", idx, changed_sp_.dgs_type,
                   changed_sp_.pole ? "true" : "false");
            adgs_arpt->SetDgsType(idx, changed_sp_.dgs_type, changed_sp_.pole);
            lb_stands_[idx] = adgs_arpt->GetStandParams(idx);
        }
        changed_idx_.clear();
        request_set_dgs_type_ = false;
        return;
    }
}

// Delayed actions that require FlightLoop context
float Editor::FlightLoopCb(float, float, int, void* inRefcon) {
    reinterpret_cast<Editor*>(inRefcon)->ProcessChangedParams();
    return 0.0f;
}
