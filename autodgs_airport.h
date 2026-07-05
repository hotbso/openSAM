//
//    openSAM: manage DGS and jetways for X Plane
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

#pragma once

#include <memory>
#include <tuple>

#include "dgs/airport.h"

// DGS types per stand, AutoDGS mode
// preserve order as this is used in cfg files
enum VDgsType {
    kAutomatic = 0,       // automatic selection based on stand's has_xp12_jw
    kMarshaller,
    kDefaultVDGS,         // default VDGS type, can be overridden by user config
    kVdgsSafedock_T2_24,  // specific VDGS type, can be overridden by user config
    kVdgsSafedock_X       // specific VDGS type, can be overridden by user config
};

extern int default_vdgs_type;

enum OperationMode { kAuto, kManual };
extern const char * const opmode_str[];
extern OperationMode operation_mode;

class AdgsAirport;

// dgs::Stand augmented for AutoDGS
class AdgsStand : public dgs::Stand {
   protected:
    friend class AdgsAirport;

    int dgs_type_;
    float dgs_dist_;         // distance to dgs
    float dgs_height_;       // height of dgs (AGL) (only relevant for VDGS)
    float dgs_left_right_;   // left/right lateral offset of the DGS from the stand centerline, in m (positive = right)
    bool dgs_pole_;          // whether the DGS is on a pole or not (for VDGS only)

    float marshaller_max_dist_;  // max distance, actual can be lower according to PE

    void CalcDgsPosition();

   public:
    AdgsStand(AdgsStand&&) = default;
    AdgsStand& operator=(AdgsStand&&) = delete;

    AdgsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation, int dgs_type, float dgs_dist,
              float dgs_height, float dgs_left_right, bool pole);
    ~AdgsStand();

    void SetDgsType(int dgs_type);
    void CycleDgsType();
    void DgsMoveCloser();  // with wrap around
    void SetDistanceHeight(float dgs_dist, float dgs_height);
    bool has_jw() const override;
};

// for the interaction with the UI and Editor
struct AdgsStandParams {
    // these are read-only
    int idx;           // index of this slot in the airport's stands_ vector
    std::string name;
    bool has_xp12_jw;

    // these are settable by the user and are saved in the airport's .cfg file
    int dgs_type;
    float dgs_dist;
    float dgs_height;
    float dgs_left_right{};  // left/right lateral offset of the DGS from the stand centerline, in m (positive = right)
    bool pole = true;  // whether the DGS is on a pole or not (for VDGS only)
};

// dgs::Airport augmented for AutoDGS
class AdgsAirport : public dgs::Airport {
    bool user_cfg_changed_;

    void FlushUserCfg();

   public:
    AdgsAirport() = delete;
    AdgsAirport(const dgs::AptAirport&);
    ~AdgsAirport();

    void ResetState(State new_state);

    AdgsStandParams GetStandParams(int idx) const;
    void SetStandParams(int idx, const AdgsStandParams& params);

    // these act onto the selected or active stand and are for tactical use only,
    // e.g. by the ui or commands
    void DgsMoveCloser();
    void SetDgsType(int dgs_type);
    void CycleDgsType();

    // auto set chocks and connect jetway when parking ?
    bool auto_post_parkbrake() const override;
    void ConnectJetway() override;
};

extern std::unique_ptr<AdgsAirport> adgs_arpt;
