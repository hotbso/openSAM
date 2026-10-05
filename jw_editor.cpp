//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
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

#include <cassert>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <print>
#include <filesystem>
#include <chrono>
#include <stdexcept>

#include "XPLMGraphics.h"
#include "XPLMCamera.h"
#include "XPLMUtilities.h"

#include "imgui.h"
#include "imgui_stdlib.h"
#include "ImgWindow.h"

#include "opensam.h"
#include "scenery.h"
#include "os_airport.h"
#include "samjw.h"
#include "flat_earth_math.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "ui.h"
#include "jw_editor.h"
#include "log_msg.h"

namespace fem = flat_earth_math;
namespace fs = std::filesystem;         // that is so long...

static constexpr int kWinWidth = 450;
static constexpr int kWinHeight = 650;
static constexpr int kWinPad = 150;

std::unique_ptr<ImgWindow> jw_editor;
int jw_editor_left = -1, jw_editor_top, jw_editor_right, jw_editor_bottom;  // -1 = not loaded from prefs
bool jw_editor_active;

static XPLMObjectRef marker_obj_ = nullptr;
static XPLMCommandRef free_camera_cmdr_ = nullptr;

// Jetway Editor for fine tuning library jetway instances
class JwEditor : public ImgWindow {
    int arpt_seqno_ = 0;                         // for detecting changes in the airport data
    fem::LLPos cam_pos_ = fem::LLPos(0.0, 0.0);  // for detecting changes in the camera position
    double cam_pos_alt_{};                         // altitude of camera

    Scenery* scenery_;  // ptr to current scenery

    // we refresh on significant changes of the camera or after a certain interval
    // to keep the (un)seen state in the ui in sync with the actual scene
    static constexpr float kRefreshInterval = 10.0f;  // refresh interval in seconds
    float refresh_ts_{0.0f};                          // timestamp of the last refresh
    std::vector<SamJw*> jw_set_;                      // jetways of this airport
    std::vector<std::string> jw_lb_labels_;           // listbox labels for the jetways, e.g. "Jetway 1 (configured)"
    std::vector<SamJwModel*> model_set_;              // models for this scenery

    int selected_idx_ = -1;           // for processing the selected jetway
    int selected_model_idx_ = -1;     // for processing the selected jetway's model
    SamJwModel selected_model_copy_;  // for editing the selected model's parameters
    bool model_changed_ = false;      // whether the user has changed the model parameters

    bool unsaved_changes_ = false;  // whether the user has made changes that are not yet saved to opensam.xml

    std::string msg_line1_, msg_line2_;  // for displaying messages to the user
    fs::path xml_path_;
    std::string pathname_short_;

    int imgui_id_;

    // background processing in flightloop context, e.g. creating instances etc.
    XPLMInstanceRef marker_inst_ = nullptr;
    bool request_place_marker_ = false;
    bool request_remove_marker_ = false;
    XPLMDrawInfo_t marker_draw_info_{};

    std::string MkJwLbEntry(const SamJw* jw);
    bool EditModels();
    void EditJetways();

    // Main function: creates the window's UI
    void BuildInterface() override;

    // background processing in flightloop context, e.g. creating instances etc.
    void FlightLoopUserCb() noexcept override;

    // turning to the selected jw is one shot only
    int camera_steps_ = 0;
    static int CameraCtrlCb(XPLMCameraPosition_t* inCameraPosition, int loosing_ctrl, void* inRefcon);

   public:
    JwEditor(int left, int top, int right, int bot);
    ~JwEditor() override;
};

void CreateJwEditor() {
    static bool init_done;
    if (!init_done) {
        init_done = true;
        free_camera_cmdr_ = XPLMFindCommand("sim/view/free_camera");

        std::string marker = base_dir + "resources/marker.obj";
        marker_obj_ = XPLMLoadObject(marker.c_str());
        if (marker_obj_ == nullptr) {
            LogMsg("Failed to load marker object from path: '%s'", marker.c_str());
            return;
        }
    }

    if (jw_editor_left == -1) {
        LogMsg("Creating editor window with default geometry");
        int sc_left, sc_top;
        XPLMGetScreenBoundsGlobal(&sc_left, &sc_top, nullptr, nullptr);

        jw_editor_left = sc_left + kWinPad;
        jw_editor_right = jw_editor_left + kWinWidth;
        jw_editor_top = sc_top - kWinPad;
        jw_editor_bottom = jw_editor_top - kWinHeight;
    } else
        LogMsg("Creating os editor window with geometry %d,%d,%d,%d", jw_editor_left, jw_editor_top, jw_editor_right, jw_editor_bottom);

    jw_editor = std::make_unique<JwEditor>(jw_editor_left, jw_editor_top, jw_editor_right, jw_editor_bottom);
}

///////////////////////////////////////////////////////////////////////////////////////////
JwEditor::JwEditor(int left, int top, int right, int bot)
    : ImgWindow(left, top, right, bot, xplm_WindowDecorationRoundRectangle, xplm_WindowLayerFloatingWindows) {
    ImGui::GetIO().IniFilename = nullptr;  // disable imgui.ini file, it's not compatible with imWindow

    SetWindowTitle("openSAM Jetway Editor");
    SetWindowResizingLimits(100, 100, 1024, 1024);
    SetVisible(true);
    jw_set_.reserve(100);  // avoid reallocation when we fill the listbox
    jw_lb_labels_.reserve(100);
    marker_draw_info_.structSize = sizeof(marker_draw_info_);
    LogMsg("Editor window created");
}

JwEditor::~JwEditor() {
    GetWindowGeometry(jw_editor_left, jw_editor_top, jw_editor_right, jw_editor_bottom);  // save geometry for next time
    jw_editor_active = false;  // clear the global flag
    if (marker_inst_)
        XPLMDestroyInstance(marker_inst_);
}

// create entry for jw listbox, e.g. "Jetway 1 configured 'jetway1'"
std::string JwEditor::MkJwLbEntry(const SamJw* jw) {
    std::string_view type, model_id, conf_status, seen;
    if (jw->obj_ref_gen_ == ref_gen)
        seen = "seen";
    else
        seen = "unseen";

    if (jw->is_lib_jw_inst_) {
        type = "LIB";
        if (0 < jw->library_id_ && jw->library_id_ < (int)lib_jw.size())
            model_id = lib_jw[jw->library_id_]->model_id;
        else {
            model_id = "(undetermined)";
        }
        conf_status = jw->is_zc_jw_ ? "(zero config)" : "configured";
    } else {
        type = "LCL";
        model_id = jw->model_id_;
        conf_status = model_id.empty() ? "undefined" : "configured";
    }

    return std::format("{} {:16} {:16} {:8s} '{}'", type, jw->name_, conf_status, seen, model_id);
}

// return true if editing of models in progress
bool JwEditor::EditModels() {
    bool editing = false;
    if (ImGui::TreeNode("Edit Model Parameters")) {
        editing = true;
        ImGui::TextUnformatted("Jetway Models");
        int height = 8.0f * ImGui::GetTextLineHeightWithSpacing();
        if (ImGui::BeginListBox("##Models", ImVec2(-FLT_MIN, height))) {
            for (int i = 0; i < (int)scenery_->jw_models_.size(); i++) {
                ImGui::PushID(imgui_id_++);  // Ensure unique ID for each item, stand names may have duplicates

                // Render the selectable item
                bool is_selected = (selected_model_idx_ == i);
                if (ImGui::Selectable(model_set_[i]->model_id.c_str(), is_selected)) {
                    if (is_selected) {
                        is_selected = false;
                        selected_model_idx_ = -1;
                    } else {
                        is_selected = true;
                        selected_model_idx_ = i;
                        selected_model_copy_ = *model_set_[i];  // make a copy for editing
                        model_changed_ = false;
                    }
                }
                // Set the initial focus when opening the combo/listbox (optional)
                if (is_selected)
                    ImGui::SetItemDefaultFocus();

                ImGui::PopID();
            }
            ImGui::EndListBox();
        }

        // allow to add max 1 new model, with default parameters, if it doesn't exist yet
        if (!scenery_->jw_models_.contains("new_model")) {
            ImGui::Spacing();
            if (ImGui::Button("Add New Model")) {
                // Create a new model with default parameters
                SamJwModel* new_model = new SamJwModel();
                new_model->model_id = "new_model";
                new_model->name = "New Model";
                new_model->height = 4.0f;
                new_model->wheel_pos = 10.0f;
                new_model->cabin_pos = 15.0f;
                new_model->cabin_length = 2.0f;
                new_model->wheel_diameter = 1.0f;
                new_model->wheel_distance = 3.0f;
                new_model->min_rot2 = -90.0f;
                new_model->max_rot2 = 5.0f;
                new_model->min_rot3 = -5.0f;
                new_model->max_rot3 = 5.0f;
                new_model->min_extent = 0.0f;
                new_model->max_extent = 20.0f;

                // Add the new model to the scenery's model set
                scenery_->jw_models_[new_model->model_id] = new_model;
                model_set_.push_back(new_model);
                selected_model_idx_ = -1;  // reset selection when adding a new model
                request_remove_marker_ = true;
            }
        }

        ImGui::Spacing();

        // nothing selected
        if (selected_model_idx_ < 0 || selected_model_idx_ >= (int)model_set_.size()) {
            msg_line1_.clear();
            msg_line2_.clear();
            ImGui::TreePop();
            return false;
        }

        SamJwModel& smc = selected_model_copy_;  // for convenience

        if (ImGui::InputText("Model Id", &smc.model_id))
            model_changed_ = true;
        if (ImGui::InputText("Name", &smc.name))
            model_changed_ = true;
        if (ImGui::SliderFloat("Height", &smc.height, 4.0f, 10.0f, "%.1f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Wheel Position", &smc.wheel_pos, 4.0f, 25.0f, "%.1f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Cabin Position", &smc.cabin_pos, 5.0f, 30.0f, "%.1f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Cabin Length", &smc.cabin_length, 1.0f, 3.0f, "%.1f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Wheel Diameter", &smc.wheel_diameter, 0.5f, 2.0f, "%.2f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Wheel Distance", &smc.wheel_distance, 1.0f, 5.0f, "%.2f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Minimum Rotate 2", &smc.min_rot2, -180.0f, 180.0f, "%.f deg"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Maximum Rotate 2", &smc.max_rot2, -180.0f, 180.0f, "%.f deg"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Minimum Rotate 3", &smc.min_rot3, -90.0f, 90.0f, "%.f deg"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Maximum Rotate 3", &smc.max_rot3, -90.0f, 90.0f, "%.f deg"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Minimum Extend", &smc.min_extent, 0.0f, 5.0f, "%.1f m"))
            model_changed_ = true;
        if (ImGui::SliderFloat("Maximum Extend", &smc.max_extent, 0.0f, 30.0f, "%.1f m"))
            model_changed_ = true;

        if (model_changed_) {
            bool id_collision = false;
            for (int i = 0; i < (int)model_set_.size(); i++) {
                if (i != selected_model_idx_ && model_set_[i]->model_id == smc.model_id) {
                    id_collision = true;
                    break;
                }
            }

            if (id_collision) {
                msg_line1_ = "Error: Model Id collision with another model";
                msg_line2_ = "Please choose a unique Model Id";
            } else if (smc.model_id.empty()) {
                msg_line1_ = "Error: Model Id cannot be empty";
                msg_line2_ = "Please enter a valid Model Id";
            } else {
                msg_line1_ = "Model parameters changed, click 'Commit Changes to Model' to save";
                msg_line2_.clear();

                if (ImGui::Button("Commit Changes to Model")) {
                    LogMsg("Committed changes to model '%s'", smc.model_id.c_str());
                    SamJwModel* ljw = model_set_[selected_model_idx_];
                    std::string old_id = ljw->model_id;
                    *ljw = smc;     // update from working copy
                    // replace in map, which may have changed the key if the id changed
                    scenery_->jw_models_.erase(old_id);
                    scenery_->jw_models_[smc.model_id] = ljw;

                    // and propagate the changes to all jetways that are using this model
                    for (int i = 0; i < (int)jw_set_.size(); i++) {
                        SamJw* jw = jw_set_[i];
                        if (jw->model_id_ == old_id) {
                            jw->model_id_ = smc.model_id;
                            jw->FillModelValues(smc);
                            jw_lb_labels_[i] = MkJwLbEntry(jw);  // update the listbox label
                        }
                    }

                    unsaved_changes_ = true;
                    model_changed_ = false;
                    msg_line1_.clear();
                    msg_line2_.clear();
                }
            }
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Spacing();
    return editing;
}

void JwEditor::EditJetways() {
    int height = ImGui::GetContentRegionAvail().y;
    height -= 18.0f * ImGui::GetTextLineHeightWithSpacing();

    ImGui::Text("Jetways configured and/or in view: %d", (int)jw_set_.size());
    // use monospaced font with 12.4.4
    if (ImGui::BeginListBox("##Jetways", ImVec2(-FLT_MIN, height))) {
        for (int i = 0; i < (int)jw_set_.size(); i++) {
            ImGui::PushID(imgui_id_++);  // Ensure unique ID for each item, stand names may have duplicates

            // Render the selectable item
            bool is_selected = (selected_idx_ == i);
            if (ImGui::Selectable(jw_lb_labels_[i].c_str(), is_selected)) {
                if (is_selected) {
                    is_selected = false;
                    selected_idx_ = -1;
                } else {
                    is_selected = true;
                    selected_idx_ = i;

                    const SamJw* jw = jw_set_[i];
                    float height = jw->height_;
                    if (height == 0.0f)  // undefined jw
                        height = 4.5f;

                    if (jw->obj_ref_gen_ == ref_gen) {
                        marker_draw_info_.x = jw->x_;
                        marker_draw_info_.y = jw->y_ + height + 4.0f;
                        marker_draw_info_.z = jw->z_;
                    } else {
                        // jetway was not seen yet, we use camera_alt_ as a starting point for the probe
                        double x, y, z;
                        XPLMWorldToLocal(jw->latitude_, jw->longitude_, cam_pos_alt_, &x, &y, &z);
                        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo))
                            throw std::runtime_error("XPLMProbeTerrainXYZ failed");

                        marker_draw_info_.x = probeinfo.locationX;
                        marker_draw_info_.y = probeinfo.locationY + height + 4.0f;
                        marker_draw_info_.z = probeinfo.locationZ;
                    }
                    request_place_marker_ = true;
                }
            }

            // Set the initial focus when opening the combo/listbox (optional)
            if (is_selected)
                ImGui::SetItemDefaultFocus();
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (selected_idx_ < 0) {
        request_remove_marker_ = true;
        ImGui::TextUnformatted("No jetway selected");
        return;
    }

    assert(0 <= selected_idx_ && selected_idx_ < (int)jw_set_.size());
    SamJw* jw = jw_set_[selected_idx_];

    ImGui::Text("Selected jetway: %s", jw->name_.c_str());

    ImGui::Spacing();

    // jetway is not rendered or even stale, offer delete
    if (jw->obj_ref_gen_ != ref_gen) {
        if (ImGui::Button("Delete Jetway")) {
            jw->is_deleted_ = true;
            request_remove_marker_ = true;
            unsaved_changes_ = true;
            jw_set_.erase(jw_set_.begin() + selected_idx_);
            jw_lb_labels_.erase(jw_lb_labels_.begin() + selected_idx_);
            selected_idx_ = -1;
            return;
        }

        ImGui::Spacing();
    }

    int model_idx = -1;
    bool changed = false;

    if (!jw->is_lib_jw_inst_) {
        // find the model index for the selected jetway
        for (int i = 0; i < (int)model_set_.size(); i++) {
            if (model_set_[i]->model_id == jw->model_id_) {
                model_idx = i;
                break;
            }
        }

        if (ImGui::BeginCombo("Model", model_idx >= 0 ? model_set_[model_idx]->model_id.c_str() : "")) {
            for (int i = 0; i < (int)model_set_.size(); ++i) {
                const bool is_selected = (model_idx == i);
                if (ImGui::Selectable(model_set_[i]->model_id.c_str(), is_selected)) {
                    model_idx = i;
                    changed = true;
                    jw->model_id_ = model_set_[model_idx]->model_id;
                    jw->FillModelValues(*model_set_[model_idx]);
                    if (jw->is_undefined_) {
                        jw->is_undefined_ = false;  // mark as defined now that a model is assigned
                        jw->min_rot1_ = -90.0f;
                        jw->max_rot1_ = 90.0f;
                    }
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (model_idx < 0)
            ImGui::TextUnformatted("Assign a model first");
    }

    // additional fields only for valid models or library instances
    if (jw->is_lib_jw_inst_ || model_idx >= 0) {
        if (ImGui::InputText("Name", &jw->name_)) {
            LogMsg("Jetway name set to '%s'", jw->name_.c_str());
            jw->base_name_ = jw->name_;  // keep base name in sync
            changed = true;
        }

        if (ImGui::SliderFloat("Initial Extend", &jw->initial_extent_, jw->min_extent_, jw->max_extent_, "%.1f m")) {
            changed = true;
            jw->extent_ = jw->initial_extent_;
        }

        if (ImGui::SliderFloat("Minimum Rotate 1", &jw->min_rot1_, -180.0f, 180.0f, "%.f deg")) {
            changed = true;
            if (jw->rotate1_ < jw->min_rot1_)
                jw->rotate1_ = jw->min_rot1_;
            if (jw->initial_rot1_ < jw->min_rot1_)
                jw->initial_rot1_ = jw->min_rot1_;
        }

        if (ImGui::SliderFloat("Maximum Rotate 1", &jw->max_rot1_, -180.0f, 180.0f, "%.f deg")) {
            changed = true;
            if (jw->rotate1_ > jw->max_rot1_)
                jw->rotate1_ = jw->max_rot1_;
            if (jw->initial_rot1_ > jw->max_rot1_)
                jw->initial_rot1_ = jw->max_rot1_;
        }

        if (ImGui::SliderFloat("Initial Rotate 1", &jw->initial_rot1_, -90.0f, 90.0f, "%.1f °")) {
            changed = true;
            jw->rotate1_ = jw->initial_rot1_;
            if (jw->rotate1_ < jw->min_rot1_)
                jw->min_rot1_ = jw->initial_rot1_;
            if (jw->rotate1_ > jw->max_rot1_)
                jw->max_rot1_ = jw->initial_rot1_;
        }

        if (ImGui::SliderFloat("Initial Rotate 2", &jw->initial_rot2_, jw->min_rot2_, jw->max_rot2_, "%1.0f °")) {
            changed = true;
            jw->rotate2_ = jw->initial_rot2_;
        }

        if (ImGui::SliderFloat("Initial Rotate 3", &jw->initial_rot3_, jw->min_rot3_, jw->max_rot3_, "%.1f °")) {
            changed = true;
            jw->rotate3_ = jw->initial_rot3_;
            jw->SetWheels();
        }

        if (changed) {
            jw->is_zc_jw_ = false;                           // no longer zero configured
            jw_lb_labels_[selected_idx_] = MkJwLbEntry(jw);  // update listbox content
            jw->Reset();
            unsaved_changes_ = true;
            msg_line1_.clear();
            msg_line2_.clear();
        }
    }

    ImGui::Separator();
}

void JwEditor::BuildInterface() {
    if (os_arpt == nullptr) {
        ImGui::TextUnformatted("No SAM airport loaded");
        return;
    }

    ImGui::TextUnformatted("EXPERIMENTAL backport of the jetway editor for XP 12.4.4");

    bool was_active = jw_editor_active;
    ImGui::Checkbox("Edit Mode", &jw_editor_active);
    if (!jw_editor_active) {
        request_remove_marker_ = true;
        return;
    }

    bool must_reload = false;
    // if the airport changed or we switched to "Edit Mode"" we have to rebuild the listbox content...
    if (arpt_seqno_ != os_arpt->seqno_ || !was_active) {
        LogMsg("Airport data changed or editor activated, rebuilding listbox content");
        arpt_seqno_ = os_arpt->seqno_;
        scenery_ = os_arpt->apt_airport_.scenery_;
        assert(scenery_);

        must_reload = true;
        // save as fs::path for later and create a short version for logging
        xml_path_ = scenery_->sam_xml_pathname_;
        if (xml_path_.has_parent_path() && !xml_path_.parent_path().filename().empty())
            pathname_short_ = (xml_path_.parent_path().filename() / xml_path_.filename()).generic_string();
        else
            pathname_short_ = xml_path_.generic_string();
        LogMsg("opensam.xml: %s", scenery_->sam_xml_pathname_.c_str());
    }

    // .. or if the camera position changed significantly, we do this as well
    XPLMCameraPosition_t cam_pos_lcl;
    XPLMReadCameraPosition(&cam_pos_lcl);

    fem::LLPos cam_ll;
    XPLMLocalToWorld(cam_pos_lcl.x, cam_pos_lcl.y, cam_pos_lcl.z, &cam_ll.lat, &cam_ll.lon, &cam_pos_alt_);
    if (fem::len(cam_ll - cam_pos_) > 100.0f) {
        LogMsg("Camera position changed, ll: (%.6f, %.6f), reloading jws", cam_ll.lat, cam_ll.lon);
        cam_pos_ = cam_ll;
        must_reload = true;
    }

    if (must_reload || (now - refresh_ts_ > kRefreshInterval)) {
        refresh_ts_ = now;
        // we try to restore a selection after reloading
        SamJw* selected_jw = nullptr;
        if (selected_idx_ >= 0)
            selected_jw = jw_set_[selected_idx_];

        std::unordered_map<SamJw*, bool> near_jws = jw_quadtree.FindInBox(
            os_arpt->apt_airport_.bounds(), [](const SamJw* jw) { return jw->class_code() == SamJw::kSamJw; });

        jw_set_.clear();
        selected_idx_ = -1;  // reset selection when reloading

        for (const auto [jw, _] : near_jws) {
            // LogMsg("Found jetway '%s' at ll: (%0.6f, %0.6f), is_undefined: %d", jw->name.c_str(), jw->latitude,
            // jw->longitude, jw->is_undefined);
            //  lib jws in view
            if (jw->is_lib_jw_inst_) {
                if (jw->is_zc_jw_ && !jw->stand_retrieved_) {
                    jw->stand_retrieved_ = true;  // one shot only

                    const OsStand* stand = os_arpt->FindStandForJw(jw->x_, jw->z_);
                    if (stand) {
                        jw->base_name_ = stand->name();
                        // delta = cabin points perpendicular to stand
                        float delta = fem::RA((stand->hdgt() + 90.0f) - jw->psi_);
                        // randomize
                        float delta_r = (0.2f + 0.8f * (0.01f * (rand() % 100))) * delta;
                        jw->initial_rot2_ = delta_r;
                    } else
                        jw->base_name_ = "zc_jw";  // fallback name for zero config jetways
                }

                if (jw->name_.empty()) {
                    LogMsg("Assigning name to jetway '%s' (lib id %d) at ll: (%0.6f, %0.6f)", jw->base_name_.c_str(), jw->library_id_, jw->latitude_, jw->longitude_);
                    if (jw->base_name_.length() > 10)
                        jw->name_ = jw->base_name_.substr(0, 10);
                    else
                        jw->name_ = jw->base_name_;
                }
            } else if (jw->name_.empty() && !jw->stand_retrieved_) {
                jw->stand_retrieved_ = true;  // one shot only
                LogMsg("Assigning name to undefined jetway at ll: (%0.6f, %0.6f)", jw->latitude_, jw->longitude_);
                const OsStand* stand = os_arpt->FindStandForJw(jw->x_, jw->z_);
                if (stand) {
                    LogMsg("Found stand '%s' for undefined jetway at ll: (%0.6f, %0.6f)", stand->name().c_str(),
                           jw->latitude_, jw->longitude_);
                    jw->base_name_ = stand->name();
                    jw->name_ = jw->base_name_;
                    if (jw->name_.length() > 10)
                        jw->name_.resize(10);
                }
            }

            jw_set_.push_back(jw);
        }

        if (jw_set_.empty()) {
            LogMsg("no jetways found on airport");
            return;
        }

        std::sort(jw_set_.begin(), jw_set_.end(), [](const SamJw* a, const SamJw* b) { return a->name_ < b->name_; });
        jw_lb_labels_.clear();
        for (const SamJw* jw : jw_set_)
            jw_lb_labels_.push_back(MkJwLbEntry(jw));

        model_set_.clear();
        selected_model_idx_ = -1;
        for (auto& [_, ljw] : scenery_->jw_models_)
            model_set_.push_back(ljw);

        if (selected_jw)
            selected_idx_ = std::distance(jw_set_.begin(), std::find(jw_set_.begin(), jw_set_.end(), selected_jw));
    }

    // build ui
    imgui_id_ = 1;

    if (!EditModels())
        EditJetways();

    if (unsaved_changes_) {
        ImGui::Spacing();
        ImGui::TextUnformatted("Unsaved changes, click 'Save to opensam.xml' to save");
        ImGui::Spacing();
        if (ImGui::Button("Save to opensam.xml")) {
            auto ftime = fs::last_write_time(xml_path_);
            auto stime = std::chrono::file_clock::to_sys(ftime);

            // Truncate sub-seconds and format as ISO 8601 UTC (e.g., 2026-08-18T20:34:46Z)
            std::string iso_str = std::format("{:%FT%TZ}", std::chrono::floor<std::chrono::seconds>(stime));
            std::ranges::replace(iso_str, ':', '.');  // replace ':' with '.' for filename safety

            // backup pathname: opensam.xml_2026-08-18T20.34.46Z.xml
            fs::path bpn = xml_path_.parent_path() / std::format("{}_{}.xml", xml_path_.stem().string(), iso_str);
            LogMsg("Backing up '%s' to '%s'", xml_path_.generic_string().c_str(), bpn.generic_string().c_str());
            try {
                fs::copy_file(xml_path_, bpn, fs::copy_options::overwrite_existing);
            } catch (const std::exception& e) {
                LogMsg("Failed to backup '%s' to '%s': %s", xml_path_.generic_string().c_str(),
                        bpn.generic_string().c_str(), e.what());
                msg_line1_ = std::format("Failed to backup '{}': {}", xml_path_.generic_string(), e.what());
                msg_line2_.clear();
                return;
            }

            // create a short version of the backup pathname for logging and display
            std::string bpn_short;
            if (bpn.has_parent_path() && !bpn.parent_path().filename().empty())
                bpn_short = (bpn.parent_path().filename() / bpn.filename()).generic_string();
            else
                bpn_short = bpn.generic_string();
            msg_line1_ = std::format("Backed up '../{}'", bpn_short.c_str());

            LogMsg("Saving changes to %s", xml_path_.generic_string().c_str());
            if (scenery_->UpdateOpenSamXml(jw_set_)) {
                unsaved_changes_ = false;
                msg_line2_ = std::format("Saved changes to '../{}'", pathname_short_);
            }
        }
    }

    ImGui::Spacing();
    if (!msg_line1_.empty())
        ImGui::TextUnformatted(msg_line1_.c_str());

    if (!msg_line2_.empty())
        ImGui::TextUnformatted(msg_line2_.c_str());
}

// static
// asynchronous camera control callback, beware of stale data
int JwEditor::CameraCtrlCb(XPLMCameraPosition_t* cam_pos, int loosing_ctrl, [[maybe_unused]] void* inRefcon) {
    if (loosing_ctrl) {
        LogMsg("CameraCtrlCb losing control");
        return 0;
    }

    JwEditor* jwe = dynamic_cast<JwEditor*>(jw_editor.get());    // for convenience
    if (jwe == nullptr || jwe->marker_inst_ == nullptr)   // editor closed, spurious call?
        return 0;   // give back control

    LogMsg("CameraCtrlCb called, step: %d", jwe->camera_steps_);

    XPLMReadCameraPosition(cam_pos);
    float dx = cam_pos->x - jwe->marker_draw_info_.x;
    float dy = cam_pos->y - jwe->marker_draw_info_.y;
    float dz = cam_pos->z - jwe->marker_draw_info_.z;
    cam_pos->heading = 90.0f + std::atan2(dz, dx) / kD2R - 180.0f;
    cam_pos->pitch = -std::atan2(dy, std::sqrt(dx * dx + dz * dz)) / kD2R;
    //LogMsg("CameraCtrlCb: dx=%0.2f, dy=%0.2f, dz=%0.2f, heading=%0.2f, pitch=%0.2f", dx, dy, dz, cam_pos->heading, cam_pos->pitch);
    jwe->camera_steps_++;  // indicate that the camera has been turned

    if (jwe->camera_steps_ == 3) {
        LogMsg("Releasing camera control after 3 steps");
        XPLMCommandOnce(free_camera_cmdr_);
        // continue until we are called with loosing_ctrl set to true
    }

    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// background processing in flightloop context
void JwEditor::FlightLoopUserCb() noexcept {
    static const char* null_dlist[] = {nullptr};

    if (os_arpt == nullptr || os_arpt->seqno_ != arpt_seqno_) {
        request_place_marker_ = request_remove_marker_ = false;
        // stale request
        return;
    }

    try {
        if (request_place_marker_) {
            request_place_marker_ = false;  // reset the request flag
            LogMsg("Handling request to place marker");
            if (marker_inst_ == nullptr)
                marker_inst_ = XPLMCreateInstance(marker_obj_, null_dlist);
            if (marker_inst_)
                XPLMInstanceSetPosition(marker_inst_, &marker_draw_info_, nullptr);
            camera_steps_ = 0;                  // one shot only
            XPLMControlCamera(xplm_ControlCameraUntilViewChanges, CameraCtrlCb, nullptr);
        }

        if (request_remove_marker_) {
            request_remove_marker_ = false;  // reset the request flag
            if (marker_inst_) {
                LogMsg("Destroying marker");
                XPLMDestroyInstance(marker_inst_);
               marker_inst_ = nullptr;
            }
        }

    } catch (const std::exception& e) {
        LogMsg("Exception in Editor::FlightLoopUserCb: %s", e.what());
        error_disabled = true;  // soft disable the plugin
    }
}

