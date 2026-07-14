//
//    AutoDGS / openSAM: Manage DGS
//
//    Copyright (C) 2026 Holger Teutsch
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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "dgs/dgs.h"

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

namespace dgs {

// A helper class to manage flexible datarefs, e.g. for pca, chocks etc, which can be configured in planes.cfg .
// We use late mapping as often these are defined late and are not yet available in the plane loaded callback.
//
// All config and conversion errors are thrown as these are configuration errors and should be fixed by the user, not
// silently ignored.
//
class FlexDref {
    std::string name_;
    XPLMDataRef dr_ = nullptr;
    XPLMDataTypeID type_id_;

    bool mapped_ = false;
    int cmp_type_{};  // 0: eq, 1: ge, 2: gt, 3: le, 4: lt
    float cmp_value_ = 0.0;

    void MapDref();

   public:
    FlexDref() = default;

    void Clear();
    bool empty() const;

    // name_cmp_value is e.g. "AirbusFBW/Chocks ge 1", cmp can be "eq", "ge", "gt", "le", "lt"
    void Set(const std::string& name_cmp_value);
    int GetTriggered(); // -> 0 or 1, if value >= cmp_value_
    float GetValue();
};

class FlexCmd {
    std::string name_;
    bool is_cmd_ = false; // or a dref
    XPLMCommandRef cmd_ = nullptr;
    XPLMDataRef dr_ = nullptr;
    float dr_value_ = 1.0f;
    bool mapped_ = false;

    void MapCmd();

   public:
    FlexCmd() = default;

    void Clear();
    bool empty() const;

    // "dref AirbusFBW/Chocks 1" or "cmd any_plane_set_chocks_on"
    void Set(const std::string& name);
    void Execute();
};

// A container of flags and query functions related to the plane, e.g. beacon state, chocks, gpu, pca, pbb status
// and a BeaconOn debounced of power transients.
class Plane {
    int beacon_state_{}, beacon_last_pos_{};  // beacon state, last switch_pos, ts of last switch actions
    float beacon_off_ts_{}, beacon_on_ts_{};

    FlexDref pax_no_fdr_, chk_fdr_, gpu_fdr_, pca_fdr_;  // for flexible datarefs configured in planes.cfg
    FlexCmd set_chocks_fcmdr_;                           // for flexible chocks command configured in planes.cfg

   protected:
    // plane specific config from planes.cfg, e.g. chocks dref, door pos, etc.
    std::unordered_map<std::string, std::string> cfg_;

    virtual EqStatusVal GetPbbEqStatus() = 0;

   public:
    std::string acf_icao_;
    std::string callsign_;

    bool use_engines_on_{};       // instead of beacon, e.g. MD11
    bool dont_connect_jetway_{};  // e.g. for ZIBO with own ground service
    float nw_z_, mw_z_;           // z value of plane's 0 to nose wheel, main wheel

    float pe_y_0;  // pilot eye y to plane's 0 point
    bool pe_y_0_valid{};

    bool is_helicopter_{};

    void PlaneLoadedCb();  // callback for XPLM_MSG_PLANE_LOADED
    void ResetBeacon();    // e.g. after a teleportation

    bool SetChocks();  // chocks on, where supported after parking, -> supported or not

    // queries
    bool EnginesOn();
    bool BeaconOn();                             // debounced state
    int PaxNo();                                 // -1: n/a, >=0: # of pax
    void GetEqStatus(dgs::EqStatus& eq_status);  // chocks, gpu, pca, pbb status
};

// global plane object, initialized in XPluginStart after the plane is loaded
// use shared_Ptr for coexistence with openSAM
extern std::shared_ptr<Plane> plane;

} // namespace dgs
