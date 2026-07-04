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
static constexpr int kMarshaller = 0;
static constexpr int kVDGS = 1;
static constexpr int kAutomatic = 2;

enum VDgsType {
    kVdgsSafedock_T2_24,
    kVdgsSafedock_X
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
    float dgs_dist_;            // distance to dgs
    float last_used_dgs_dist_;  // last dgs_dist_ used in drawinfo_

    float marshaller_max_dist_; // max distance, actual can be lower according to PE

    void CalcDgsPosition();

  public:
    AdgsStand(AdgsStand&&) = default;
    AdgsStand& operator=(AdgsStand&&) = delete;

    AdgsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation, int dgs_type, float dgs_dist);
    ~AdgsStand();

    void SetDgsType(int dgs_type);
    void CycleDgsType();
    void DgsMoveCloser();           // with wrap around
    void SetDistance(float dgs_dist);
    bool has_jw() const override;
};

// for intraction with the UI and Editor
struct AdgsStandParams {
    // these are read-only
    int idx;           // index of this slot in the airport's stands_ vector
    std::string name;
    bool has_xp12_jw;

    // these are settable by the user and are saved in the airport's .cfg file
    int dgs_type;
    float dgs_dist;
    // more to come
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
