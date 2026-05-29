//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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

#ifndef UI_IMGUI_H_
#define UI_IMGUI_H_

#include "ImgWindow.h"
#include <vector>

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

// Configure and Cleanup
extern void ImgWindowIni();
extern void ImgWindowFini();

extern void CreateUi();

extern std::unique_ptr<ImgWindow> ui;

#endif
