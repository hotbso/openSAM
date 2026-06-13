//
//    AutoDGS / openSAM: Manage VDGS and jetways
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

#include "dgs/dgs.h"
#include "dgs/dgs_impl.h"
#include "dgs/apt_airport.h"

#include "XPLMUtilities.h"

// AptStand augmented
namespace dgs {

// AptStand augmented with local coordinates, DGS instance, and related methods
class Stand {
   protected:
    friend class Airport;
    const AptStand& as_;

    const std::string& arpt_icao_;
    std::unique_ptr<dgs::DGS> dgs_;

    double elevation_;      // ground elevation of stand [m] (starts as an estimate from plane at touchdown)
    bool elevation_is_estimate_;  // whether elevation is still an estimate
    unsigned int ref_gen_;  // reference frame generation number
    float x_, y_, z_;       // position of stand in current reference frame
    float sin_hdgt_, cos_hdgt_;

    // dgs position
    bool dgs_position_set_;
    double dgs_lat_, dgs_lon_, dgs_altitude_;

    XPLMDrawInfo_t drawinfo_;  // for the DGS
    bool is_wet_;

    void CheckRefFrame();  // make sure all local coordintes are current in reference frame

   public:
    Stand(Stand&&) = default;
    Stand& operator=(Stand&&) = delete;

    // at first elevation is only an estimate from plane at touchdown
    Stand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation);
    virtual ~Stand() = 0;

    bool isVdgs() const;
    virtual bool has_jw() const = 0;

    void SetIdle();

    // set DGS position from lat, lon, elevation AND x, y, z in the current ref frame
    // in order to reduce multiple calls to XPLMWorldToLocal
    void SetDgsPosition(double lat, double lon, double elevation, float x, float y, float z, float psi);

    void Local2Stand(float x, float z, float& x_stand_local, float& z_stand_local);

    // accessors
    const std::string& name() const { return as_.name; };
    const char* cname() const { return as_.name.c_str(); };
    float hdgt() const { return as_.hdgt; }
    double lat() const { return as_.lat; }
    double lon() const { return as_.lon; }
};

class Airport {
   public:
    typedef enum {
        INACTIVE = 0, DEPARTURE, BOARDING,
        ARRIVAL, ENGAGED, TRACK, GOOD, BAD, PARKBRAKE_SET, BEACON_OFF, PARKED, DEBOARDING, DONE
    } state_t;

    static const char * const state_str_[];

    const int seqno_;  // sequence number

  protected:
    unsigned int ref_gen_;    // reference frame generation number

    std::string name_;
    state_t state_;

    std::vector<std::unique_ptr<Stand>> stands_;
    int active_stand_;      // -1 or index into stands_
    int selected_stand_;
    int departure_stand_;
    float departure_stand_ts_;
    int parked_pax_no_;  // for detecting changes while parked

    // values that must survive a single run of the state_machine
    dgs::GuidanceParams dgs_params_;
    float timestamp_, distance_, sin_wave_prev_;
    float nearest_stand_ts_, update_dgs_log_ts_;

    void FindNearestStand();
    int FindDepartureStand();   // index in to stands_

  public:
    Airport() = delete;
    Airport(const dgs::AptAirport&);
    virtual ~Airport();

    int nstands() const { return stands_.size(); }

    void ResetState(state_t new_state);
    void SetArrival();  // kick off guidance when we arrive at the airport
    void SetSelectedStand(int selected_stand);
    float StateMachine();

    virtual void ConnectJetway() = 0;

    // accessors and queries

    // auto set chocks and connect jetway when parking ?
    virtual bool auto_post_parkbrake() const = 0;

    const std::string& name() const { return name_; }
    state_t state() const { return state_; }
    const char* state_str() const { return state_str_[state_]; }
    int active_stand() const { return active_stand_; }
    int selected_stand() const { return selected_stand_; }
    const Stand& stand(int i) const { return *stands_[i]; } // you are expected to know what you are doing if you call this with an index that is out of bounds
    bool active_stand_has_xp12_jw() const;
};

   // destination stand from OFP remark
   extern std::string ofp_destination;
   extern std::string ofp_arrival_stand;

}  // namespace dgs

/////////////////////////////////////////////////////////////////////////////
// These are considered as shared infrastructure and mst be defined elsewhere
/////////////////////////////////////////////////////////////////////////////
extern XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, plane_elevation_dr, plane_true_psi_dr,
    parkbrake_dr, sin_wave_dr;
extern XPLMCommandRef toggle_jetway_cmdr;

extern float now;           // current timestamp

void CheckRefFrameShift();
extern unsigned int ref_gen;

extern XPLMProbeRef probe_ref;
