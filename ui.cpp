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
#include "opensam_airport.h"
#include "jwctrl.h"
#include "my_plane.h"
#include "seasons.h"
#include "os_anim.h"
#include "scenery.h"
#include "version.h"

#include "ui.h"
#include "adgs_editor.h"
#include "log_msg.h"

#include "fa-solid-900.inc"
#include "IconsFontAwesome5.h"

static constexpr int kWinWidth = 400;
static constexpr int kWinHeight = 450;
static constexpr int kWinPad = 75;
static constexpr float kFontSize = 13.0f;

std::unique_ptr<ImgWindow> ui;
int ui_left = -1, ui_top, ui_right, ui_bottom;  // -1 = not loaded from prefs

// Our own class defining our own UI
class Ui : public ImgWindow {
    int arpt_seqno_ = 0;                  // for detecting changes in the airport data
    std::vector<std::string> lb_stands_;  // for the listbox content
    int lb_item_ = -1;                    // for the listbox selection

    bool jw_auto_mode_ = false;  // to store the state of the "Automatic mode" checkbox
    bool jw_selected_[kNearJwLimit][kMaxDoor] = {};
    int nearest_jws_seqno_ = 0;  // for detecting changes in the nearest jetway list by the UI

    XPLMFlightLoopID flt_id_ = nullptr;

    bool selected_stand_changed_ = false;  // to detect changes in the listbox selection
    int new_selected_stand_;  // to store the new selected stand index until we can apply it in the flight loop callback

    bool set_mode_arrival_requested_ = false;  // to detect if the "Set mode ARRIVAL" button has been pressed

    bool move_closer_requested_ = false;  // to detect if the "Move closer" button has been pressed

    bool dgs_type_changed_ = false;  // to detect if the DGS type radio button selection has been changed
    int new_dgs_type_;
    int new_dgs_type_stand_;

    bool jw_auto_mode_changed_ = false;  // to detect if the "Automatic mode" checkbox has been changed

    // Main function: creates the window's UI
    void BuildInterface() override;

    // flight loop callback for delayed actions prohibited in drawloops
    static float FlightLoopCb(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter,
                              void* inRefcon);

   public:
    Ui(int left, int top, int right, int bot);
    ~Ui() override;
};

void CreateUi() {
    if (ui_left == -1) {
        LogMsg("Creating UI window with default geometry");
        int sc_left, sc_top;
        XPLMGetScreenBoundsGlobal(&sc_left, &sc_top, nullptr, nullptr);

        ui_left = sc_left + kWinPad;
        ui_right = ui_left + kWinWidth;
        ui_top = sc_top - kWinPad;
        ui_bottom = ui_top - kWinHeight;
    } else
        LogMsg("Creating UI window with geometry %d,%d,%d,%d", ui_left, ui_top, ui_right, ui_bottom);

    ui = std::make_unique<Ui>(ui_left, ui_top, ui_right, ui_bottom);
}

void ImgWindowIni() {
    LogMsg("Initializing Imgui Window...");
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    // load from X-Plane's default font directory
    if (ImgWindow::sFontAtlas->AddFontFromFileTTF("./Resources/fonts/DejaVuSans.ttf", kFontSize) == nullptr) {
        LogMsg("Failed to load font DejaVuSans from file, falling back to default font");
    }

    // Now we merge some icons from the OpenFontsIcons font into the above font
    // (see `imgui/docs/FONTS.txt`)
    ImFontConfig config;
    config.MergeMode = true;

    // We only read very selectively the individual glyphs we are actually using
    // to safe on texture space
    static ImVector<ImWchar> icon_ranges;
    ImFontGlyphRangesBuilder builder;
    // Add all icons that are actually used (they concatenate into one string)
    builder.AddText((const char*)ICON_FA_CHECK);
    builder.BuildRanges(&icon_ranges);

    // Merge the icon font with the text font
    ImgWindow::sFontAtlas->AddFontFromMemoryCompressedTTF(fa_solid_900_compressed_data,
                                                          fa_solid_900_compressed_size,
                                                          kFontSize,
                                                          &config,
                                                          icon_ranges.Data);
    LogMsg("Imgui Window initialized");
}

void ImgWindowFini() {
    ui = nullptr; // just in case ...
    ImgWindow::sFontAtlas.reset();
}

///////////////////////////////////////////////////////////////////////////////////////////
Ui::Ui(int left, int top, int right, int bot)
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

Ui::~Ui() {
    GetWindowGeometry(ui_left, ui_top, ui_right, ui_bottom);  // save geometry for next time
    if (flt_id_)
        XPLMDestroyFlightLoop(flt_id_);
}

void Ui::BuildInterface() {
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

    //--------------------------------------------------
    if (anim_sc && anim_sc->sam_anims_.size()) {
        if (ImGui::TreeNode("Remote Control")) {
            ImGui::Spacing();
            ImGui::Separator();

            for (auto& anim : anim_sc->sam_anims_) {
                bool on = anim.is_on();
                if (ImGui::Checkbox(anim.ui_line.c_str(), &on)) {
                    anim.SetState(on);
                    LogMsg("Animation '%s' state changed to %s", anim.ui_line.c_str(), on ? "ON" : "OFF");
                }
            }

            ImGui::TreePop();
        }
    }

    //--------------------------------------------------
    ImGui::Spacing();
    ImGui::Separator(); ImGui::Separator();
    ImGui::Spacing();

    dgs::Airport* dgs_arpt = nullptr;
    float col_2 = ImGui::GetCursorPosX() + 0.6f * kFontSize * 16.0f;

    if (adgs_arpt) {
        dgs_arpt = adgs_arpt.get();
        ImGui::TextUnformatted("AutoDGS Airport:");
        ImGui::SameLine();
        ImGui::SetCursorPosX(col_2);
        ImGui::Text("'%s'", dgs_arpt->name().c_str());
    } else if (os_arpt) {
        dgs_arpt = os_arpt.get();
        const char* xp12_jw_str = os_arpt->has_xp12_jws() ? ", has (some) XP12 default jetways" : "";
        ImGui::TextUnformatted("SAM Airport:");
        ImGui::SameLine();
        ImGui::SetCursorPosX(col_2);
        ImGui::Text("'%s'%s", dgs_arpt->name().c_str(), xp12_jw_str);
    } else {
        ImGui::Text("No airport loaded");
        return;
    }

    assert(dgs_arpt);

    if (arpt_seqno_ != dgs_arpt->seqno_) {
        // Airport data has changed since last time, so we update the listbox content.
        // We do this once because it generates a lot of allocations and may be costly per frame.
        lb_stands_.clear();
        lb_stands_.reserve(dgs_arpt->nstands() + 1);
        lb_stands_.push_back("<automatic>");  // first entry is always "<automatic>"

        // for AutoDGS we add the DGS type
        if (adgs_arpt) {
            for (int i = 0; i < dgs_arpt->nstands(); i++) {
                auto dgs_params = adgs_arpt->GetStandParams(i);
                lb_stands_.push_back((dgs_params.dgs_type == kMarshaller ? "M " : "V ") + dgs_params.name);
            }
        } else {
            for (int i = 0; i < dgs_arpt->nstands(); i++)
                lb_stands_.push_back(dgs_arpt->stand(i).name());
        }
        arpt_seqno_ = dgs_arpt->seqno_;
    }

    ImGui::Spacing();
    if (editor_active) {
        ImGui::TextUnformatted("Edit mode is ON");
        return;
    }

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
        auto dgs_params = adgs_arpt->GetStandParams(as);
        int idx = as + 1;                                            // +1 due to "<automatic>"
        lb_stands_[idx][0] = (dgs_params.dgs_type == kMarshaller ? 'M' : 'V');  // set current indicator

        new_dgs_type_ = dgs_params.dgs_type;

        ImGui::Columns(2);  // 2 columns: one for the radio buttons, one for the "Move closer" button
        if (ImGui::RadioButton("Marshaller", new_dgs_type_ == kMarshaller)) {
            new_dgs_type_ = kMarshaller;
        }

        ImGui::NextColumn();  // column 2 for the button
        if (ImGui::Button("Move closer")) {
            move_closer_requested_ = true;
            XPLMScheduleFlightLoop(flt_id_, -1.0, 1);
        }

        ImGui::NextColumn();  // column 1 for the radio button
        if (ImGui::RadioButton("VDGS", new_dgs_type_ != kMarshaller)) {
            new_dgs_type_ = kDefaultVDGS;
        }
        ImGui::Columns();

        if (new_dgs_type_ != dgs_params.dgs_type) {
            new_dgs_type_stand_ = as;
            dgs_type_changed_ = true;
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
}

// Delayed actions that require FlightLoop context
float Ui::FlightLoopCb(float, float, int, void* inRefcon) {
    LogMsg("FlightLoopCb called with inRefcon=%p", inRefcon);

    Ui& ui = *reinterpret_cast<Ui*>(inRefcon);

    dgs::Airport* dgs_arpt = nullptr;
    if (adgs_arpt)
        dgs_arpt = adgs_arpt.get();
    else if (os_arpt)
        dgs_arpt = os_arpt.get();

    if (dgs_arpt == nullptr || dgs_arpt->seqno_ != ui.arpt_seqno_) {
        // stale request
        ui.selected_stand_changed_ = ui.set_mode_arrival_requested_ = ui.move_closer_requested_ = ui.dgs_type_changed_ =
            ui.jw_auto_mode_changed_ = false;
        return 0.0f;
    }

    if (ui.selected_stand_changed_) {
        LogMsg("Setting selected stand to %d", ui.new_selected_stand_);
        dgs_arpt->SetSelectedStand(ui.new_selected_stand_);
        ui.selected_stand_changed_ = false;
    }

    if (ui.set_mode_arrival_requested_) {
        LogMsg("Setting airport mode to ARRIVAL");
        dgs_arpt->SetArrival();
        ui.set_mode_arrival_requested_ = false;
    }

    if (ui.move_closer_requested_) {
        LogMsg("Moving plane closer to the stand");
        if (adgs_arpt)
            adgs_arpt->DgsMoveCloser();
        ui.move_closer_requested_ = false;
    }

    if (ui.dgs_type_changed_) {
        LogMsg("Changing DGS type of stand index %d to %d", ui.new_dgs_type_stand_, ui.new_dgs_type_);
        if (adgs_arpt) {
            int as = adgs_arpt->active_stand();
            if (as == ui.new_dgs_type_stand_)
                adgs_arpt->SetDgsType(ui.new_dgs_type_);
        }

        ui.dgs_type_changed_ = false;
    }

    if (ui.jw_auto_mode_changed_) {
        LogMsg("Setting automatic jetway selection to %s", ui.jw_auto_mode_ ? "ON" : "OFF");
        my_plane->AutoModeSet(ui.jw_auto_mode_);
        ui.jw_auto_mode_changed_ = false;
    }

    return 0.0f;
}
