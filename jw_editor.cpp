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
#include <print>
#include <filesystem>
#include <chrono>

#include "XPLMGraphics.h"
#include "XPLMCamera.h"

#include "imgui.h"
#include "imgui_stdlib.h"
#include "ImgWindow.h"

#include "opensam.h"
#include "scenery.h"
#include "os_airport.h"
#include "samjw.h"
#include "my_plane.h"
#include "flat_earth_math.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "ui.h"
#include "jw_editor.h"
#include "log_msg.h"

#include "version.h"

namespace fem = flat_earth_math;
namespace fs = std::filesystem;         // that is so long...

static constexpr int kWinWidth = 450;
static constexpr int kWinHeight = 650;
static constexpr int kWinPad = 150;

std::unique_ptr<ImgWindow> jw_editor;
int jw_editor_left = -1, jw_editor_top, jw_editor_right, jw_editor_bottom;  // -1 = not loaded from prefs
bool jw_editor_active;

// Jetway Editor for fine tuning library jetway instances
class JwEditor : public ImgWindow {
    int arpt_seqno_ = 0;                         // for detecting changes in the airport data
    fem::LLPos cam_pos_ = fem::LLPos(0.0, 0.0);  // for detecting changes in the camera position

    std::vector<SamJw*> jw_set_;          // jetways of this airport
    std::vector<std::string> lb_labels_;  // listbox labels for the jetways, e.g. "Jetway 1 (configured)"

    int selected_idx_ = -1;  // for processing the selected jetway
    bool unsaved_changes_ = false;  // whether the user has made changes that are not yet saved to sam.xml
    std::string msg_line1_, msg_line2_;  // for displaying messages to the user
    fs::path sam_xml_path_;
    std::string pathname_short_;

    // Main function: creates the window's UI
    void BuildInterface() override;

    // background processing in flightloop context, e.g. creating instances etc.
    // void FlightLoopUserCb() noexcept override;

   public:
    JwEditor(int left, int top, int right, int bot);
    ~JwEditor() override;
};

void CreateJwEditor() {
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
    lb_labels_.reserve(100);
    LogMsg("Editor window created");
}

JwEditor::~JwEditor() {
    GetWindowGeometry(jw_editor_left, jw_editor_top, jw_editor_right, jw_editor_bottom);  // save geometry for next time
    jw_editor_active = false;  // clear the global flag
}

void JwEditor::BuildInterface() {
    if (os_arpt == nullptr) {
        ImGui::TextUnformatted("No SAM airport loaded");
        return;
    }

    ImGui::TextUnformatted("Jetway Editor for fine tuning (zero config) library jetway instances");
    ImGui::TextUnformatted("EXPERIMENTAL backport of the jetway editor for XP 12.4.4");

    bool was_active = jw_editor_active;
    ImGui::Checkbox("Edit Mode", &jw_editor_active);
    if (!jw_editor_active)
        return;

    // create entry for listbox, e.g. "Jetway 1 configured 'jetway1'"
    auto MkLbEntry = [](const SamJw* jw) -> std::string {
        const char* lib_id = "(not seen)";
        if (0 < jw->library_id && jw->library_id < (int)lib_jw.size())
            lib_id = lib_jw[jw->library_id]->id.c_str();
        return std::format("{:16} {:16} '{}'", jw->name, jw->is_zc_jw ? "(zero config)" : "configured", lib_id);
    };

    bool must_reload = false;
    // if the airport changed or we switched to active we have to rebuild the listbox content
    if (arpt_seqno_ != os_arpt->seqno_ || !was_active) {
        LogMsg("Airport data changed or editor activated, rebuilding listbox content");
        arpt_seqno_ = os_arpt->seqno_;
        must_reload = true;
        // save as fs::path for later and create a short version for logging
        sam_xml_path_ = os_arpt->sam_xml_pathname_;
        if (sam_xml_path_.has_parent_path() && !sam_xml_path_.parent_path().filename().empty())
            pathname_short_ = (sam_xml_path_.parent_path().filename() / sam_xml_path_.filename()).generic_string();
        else
            pathname_short_ = sam_xml_path_.generic_string();
        LogMsg("sam.xml: %s", os_arpt->sam_xml_pathname_.c_str());
    }

    // .. or if the camera position changed significantly, too
    XPLMCameraPosition_t cam_pos_lcl;
    XPLMReadCameraPosition(&cam_pos_lcl);

    double alt;
    fem::LLPos cam_ll;
    XPLMLocalToWorld(cam_pos_lcl.x, cam_pos_lcl.y, cam_pos_lcl.z, &cam_ll.lat, &cam_ll.lon, &alt);
    if (fem::len(cam_ll - cam_pos_) > 100.0f) {
        LogMsg("Camera position changed, ll: (%.6f, %.6f), reloading jws", cam_ll.lat, cam_ll.lon);
        cam_pos_ = cam_ll;
        must_reload = true;
    }

    if (must_reload) {
        std::unordered_map<SamJw*, bool> near_jws = jw_quadtree.FindInBox(os_arpt->apt_airport_.bounds());
        jw_set_.clear();
        selected_idx_ = -1;  // reset selection when reloading

        for (const auto [jw, _] : near_jws) {
            // lib jws in view
            if (jw->is_lib_jw_inst) {
                if (jw->is_zc_jw && !jw->zc_stand_done) {
                    jw->zc_stand_done = true;  // one shot only

                    const OsStand* stand = os_arpt->FindStandForJw(jw->x, jw->z);
                    if (stand) {
                        jw->base_name = stand->name();
                        // delta = cabin points perpendicular to stand
                        float delta = fem::RA((stand->hdgt() + 90.0f) - jw->psi);
                        // randomize
                        float delta_r = (0.2f + 0.8f * (0.01f * (rand() % 100))) * delta;
                        jw->initialRot2 = delta_r;
                    } else
                        jw->base_name = "zc_jw";  // fallback name for zero config jetways
                }

                if (jw->name.empty()) {
                    LogMsg("Assigning name to jetway '%s' (lib id %d) at ll: (%0.6f, %0.6f)", jw->base_name.c_str(), jw->library_id, jw->latitude, jw->longitude);
                    if (jw->base_name.length() > 10)
                        jw->name = jw->base_name.substr(0, 10);
                    else
                        jw->name = jw->base_name;
                }

                jw_set_.push_back(jw);
            }
        }

        if (jw_set_.empty()) {
            LogMsg("no jetways found on airport");
            return;
        }

        std::sort(jw_set_.begin(), jw_set_.end(), [](const SamJw* a, const SamJw* b) { return a->name < b->name; });
        lb_labels_.clear();
        for (const SamJw* jw : jw_set_)
            lb_labels_.push_back(MkLbEntry(jw));
    }

    int height = ImGui::GetContentRegionAvail().y;
    height -= 15.0f * ImGui::GetTextLineHeightWithSpacing();

    ImGui::Text("Library Jetways configured and/or in view: %d", (int)jw_set_.size());
    // use monospaced font with 12.4.4
    if (ImGui::BeginListBox("##Jetways", ImVec2(-FLT_MIN, height))) {
        for (int i = 0; i < (int)jw_set_.size(); i++) {
            ImGui::PushID(i);  // Ensure unique ID for each item, stand names may have duplicates

            // Render the selectable item
            bool is_selected = (selected_idx_ == i);
            if (ImGui::Selectable(lb_labels_[i].c_str(), selected_idx_ == i)) {
                // imgui magic
                selected_idx_ = i;
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
        ImGui::TextUnformatted("No jetway selected");
        return;
    }

    assert(0 <= selected_idx_ && selected_idx_ < (int)jw_set_.size());
    SamJw* jw = jw_set_[selected_idx_];
    ImGui::Text("Editing jetway: %s", jw->name.c_str());

    bool changed = false;
    if (ImGui::InputText("Name", &jw->name)) {
        LogMsg("Jetway name set to '%s'", jw->name.c_str());
        jw->base_name = jw->name;  // keep base name in sync
        changed = true;
    }

    if (ImGui::SliderFloat("Initial Extend", &jw->initialExtent, 0.0f, 10.0f, "%.1f m")) {
        changed = true;;
        jw->extent = jw->initialExtent;
    }

    if (ImGui::SliderFloat("Initial Rotate 1", &jw->initialRot1, -90.0f, 90.0f, "%.1f °")) {
        changed = true;
        jw->rotate1 = jw->initialRot1;
    }

    if (ImGui::SliderFloat("Initial Rotate 2", &jw->initialRot2, -90.0f, 90.0f, "%1.0f °")) {
        changed = true;
        jw->rotate2 = jw->initialRot2;
    }

    if (ImGui::SliderFloat("Initial Rotate 3", &jw->initialRot3, -5.0f, 5.0f, "%.1f °")) {
        changed = true;
        jw->rotate3 = jw->initialRot3;
        jw->SetWheels();
    }

    if (changed) {
        jw->is_zc_jw = false;       // no longer zero configured
        lb_labels_[selected_idx_] = MkLbEntry(jw);  // update listbox content
        unsaved_changes_ = true;
        msg_line1_.clear(); msg_line2_.clear();
    }

    ImGui::Separator();

    if (unsaved_changes_) {
        ImGui::Spacing();
        ImGui::TextUnformatted("Unsaved changes, click 'Save to sam.xml' to save");
        ImGui::Spacing();
        if (ImGui::Button("Save to sam.xml")) {
            auto ftime = fs::last_write_time(sam_xml_path_);
            auto stime = std::chrono::file_clock::to_sys(ftime);

            // Truncate sub-seconds and format as ISO 8601 UTC (e.g., 2026-08-18T20:34:46Z)
            std::string iso_str = std::format("{:%FT%TZ}", std::chrono::floor<std::chrono::seconds>(stime));
            std::ranges::replace(iso_str, ':', '.');  // replace ':' with '.' for filename safety

            // backup pathname: sam.xml_2026-08-18T20.34.46Z.xml
            fs::path bpn = sam_xml_path_.parent_path() / std::format("{}_{}.xml", sam_xml_path_.stem().string(), iso_str);
            LogMsg("Backing up '%s' to '%s'", os_arpt->sam_xml_pathname_.c_str(), bpn.generic_string().c_str());
            try {
                fs::copy_file(sam_xml_path_, bpn, fs::copy_options::overwrite_existing);
            } catch (const std::exception& e) {
                LogMsg("Failed to backup '%s' to '%s': %s", os_arpt->sam_xml_pathname_.c_str(),
                       bpn.generic_string().c_str(), e.what());
                msg_line1_ = std::format("Failed to backup '{}': {}", os_arpt->sam_xml_pathname_, e.what());
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

            LogMsg("Saving changes to %s", os_arpt->sam_xml_pathname_.c_str());
            if (UpdateSamXml(jw_set_, os_arpt->sam_xml_pathname_)) {
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

#if 0
///////////////////////////////////////////////////////////////////////////////////////////
// background processing in flightloop context
void JwEditor::FlightLoopUserCb() noexcept {
    return;

    if (os_arpt == nullptr || os_arpt->seqno_ != arpt_seqno_) {
        // stale request
        return;
    }

    return;

    try {
        LogMsg("FlightLoopUserCb");
    } catch (const std::exception& e) {
        LogMsg("Exception in Editor::FlightLoopUserCb: %s", e.what());
        error_disabled = true;  // soft disable the plugin
    }
}
#endif
