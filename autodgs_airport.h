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

typedef enum { MODE_AUTO, MODE_MANUAL } opmode_t;
extern const char * const opmode_str[];
extern opmode_t operation_mode;

class AdgsAirport;  // forward declaration for dgs::Stand

// dgs::Stand augmented for AutoDGS
class AdgsStand : public dgs::Stand {
  protected:
    friend class AdgsAirport;
    friend class ::AdgsAirport;

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

    bool has_jw() const override;

    // accessors
    int dgs_type() const { return dgs_type_; }
};

// dgs::Airport augmented for AutoDGS
class AdgsAirport : public dgs::Airport {
    bool user_cfg_changed_;

    void FlushUserCfg();

   public:
    AdgsAirport() = delete;
    AdgsAirport(const dgs::AptAirport&);
    ~AdgsAirport();

    void ResetState(state_t new_state);

    std::tuple<int, const std::string> GetStand(int idx) const;  // dgs_type, name

    // these act onto the selected or active stand
    void DgsMoveCloser();
    void SetDgsType(int dgs_type);
    int GetDgsType() const;
    void CycleDgsType();

    // auto set chocks and connect jetway when parking ?
    bool auto_post_parkbrake() const override;
    void ConnectJetway() override;
};

extern std::unique_ptr<AdgsAirport> adgs_arpt;
