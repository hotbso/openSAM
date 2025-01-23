/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2025  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

*/

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <cassert>

#include "openSAM.h"
#include "plane.h"
#include "mpplane_tgxp.h"

constexpr int kSpawnPerRun = 10;    // new Planes per update run

static XPLMDataRef
    flight_phase_dr,
    traffic_type_dr,
    acf_type_dr, flight_id_dr,     // identity
    x_dr, y_dr, z_dr, psi_dr;      // position

enum FlightPhase
{
	FP_Unknown = -1,
	FP_Cruise = 0,
	FP_Approach,			// Positioning from cruise to the runway.
	FP_Final,				// Gear down on final approach.
	FP_TaxiIn,				// Any ground movement after touchdown.
	FP_Shutdown,			// Short period of spooling down engines/electrics.
	FP_Parked,				// Long period parked.
	FP_Startup,				// Short period of spooling up engines/electrics.
	FP_TaxiOut,				// Any ground movement from the gate to the runway.
	FP_Depart,				// Initial ground roll and first part of climb.
	FP_GoAround,			// Unplanned transition from approach to cruise.
	FP_Climbout,			// Remainder of climb, gear up.
	FP_Braking,				// Short period from touchdown to when fast-taxi speed is reached.
};

enum TrafficType
{
	PT_Airline = 0,
	PT_Cargo,
	PT_GA,
	PT_Military,
};

class MpPlane_tgxp : public Plane {
    const int slot_;
    std::string flight_id_;

    float scan_mp_planes();

  public:
    MpPlane_tgxp(int slot, const std::string& flight_id, const std::string& icao,
                 float x, float y, float z, float psi);
    ~MpPlane_tgxp() override {}

    void update(bool beacon);

    bool auto_mode() const override { return true; }
    bool dock_requested() override { return true; }
};


MpPlane_tgxp::MpPlane_tgxp(int slot, const std::string& flight_id, const std::string& acf_type,
                           float x, float y, float z, float psi) : slot_(slot)
{
    flight_id_ = flight_id;

    on_ground_ = true;  // otherwise we were not here
    parkbrake_set_ = true;

    log_msg("pid=%d, constructing MpPlane %s/%s", id_, flight_id_.c_str(), acf_type.c_str());

    n_door_ = 0;
    try {
        icao_ = acf_generic_type_map.at(acf_type);
        door_info_[0] = csl_door_info_map.at(icao_ + '1');
        n_door_++;

        // lateral adjustment
        constexpr float z_adjust = 1.0f;      // backwards
        x_ = x + -sinf(D2R * psi) * z_adjust;
        z_ = z + cosf(D2R * psi) * z_adjust;

        //get y for ground level
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
            log_msg("terrain probe failed???");
        }
        y_ = probeinfo.locationY;

        psi_ = psi;

        log_msg("pid=%d, icao: %s, found door 1 in door_info_map: x: %0.2f, y: %0.2f, z: %0.2f",
                id_, icao_.c_str(), door_info_[0].x, door_info_[0].y, door_info_[0].z);
    } catch(const std::out_of_range& ex) {
        log_msg("pid=%d, %s: door 1 is not defined in door_info_map, deactivating slot", id_, acf_type.c_str());
        state_ = DISABLED;
        return;
    }

    // door 2 +3 are optional
    try {
        door_info_[1] = csl_door_info_map.at(icao_ + '2');
        n_door_++;
        door_info_[2] = csl_door_info_map.at(icao_ + '3');
        n_door_++;
    } catch(const std::out_of_range& ex) {}

    state_ = IDLE;
}

void
MpPlane_tgxp::update(bool beacon)
{
    if (state_ == DISABLED)
        return;

    beacon_on_ = beacon;

    // Jetways are only dockable if they were rendered once.
    // As they come in view over time we just retry a docking attempt if the plane is stuck
    // in CANT_DOCK.
    if (!beacon_on_ && state_ == CANT_DOCK && now > state_change_ts_ + 60.0f)
        state_ = PARKED;

    log_msg("MP update: pid=%02d, slot: %02d, icao: %s, id: %s, beacon: %d, parkbrake_set: %d, state: %s",
            id_, slot_, icao_.c_str(), flight_id_.c_str(), beacon_on_, parkbrake_set_,
            state_str_[state_]);

}

//==============  MpAdapter_tgxp ========================================
bool MpAdapter_tgxp::probe()
{
    if (flight_phase_dr == nullptr)
        flight_phase_dr = XPLMFindDataRef("trafficglobal/ai/flight_phase");

    return (flight_phase_dr && (XPLMGetDatavi(flight_phase_dr, NULL, 0, 0) > 0));
}

MpAdapter_tgxp::MpAdapter_tgxp()
{
    log_msg("MpAdapter_tgxp constructor");
    static bool init_done{false};

    if (!init_done) {
        // flight_phase_dr is already done in probe
        acf_type_dr = XPLMFindDataRef("trafficglobal/ai/aircraft_code");
        flight_id_dr = XPLMFindDataRef("trafficglobal/ai/tail_number");
        x_dr = XPLMFindDataRef("trafficglobal/ai/position_x");
        y_dr = XPLMFindDataRef("trafficglobal/ai/position_y");
        z_dr = XPLMFindDataRef("trafficglobal/ai/position_z");
        psi_dr = XPLMFindDataRef("trafficglobal/ai/position_heading");
        traffic_type_dr = XPLMFindDataRef("trafficglobal/ai/ai_type");
        init_done = true;
    }
}

MpAdapter_tgxp::~MpAdapter_tgxp()
{
    log_msg("MpAdapter_tgxp destructor");
}

#define LOAD_DR(type, name) { \
    int l = XPLMGetDatav ## type ( name ## _dr,  name ## _val_.get(), 0, n_planes); \
    assert(l == n_planes); \
}

// static
float MpAdapter_tgxp::update()
{
    int n_planes = XPLMGetDatavi(flight_phase_dr, NULL, 0, 0);
    log_msg("MpPlane_tgxp drefs #: %d", n_planes);

    if (n_planes > vector_size_) {
        vector_size_ = std::max(n_planes + 50, 200);
        log_msg("allocated vector_size_ %d", vector_size_);
        flight_phase_val_ = std::make_unique_for_overwrite<int []>(vector_size_);
        traffic_type_val_ = std::make_unique_for_overwrite<int []>(vector_size_);
        x_val_ = std::make_unique_for_overwrite<float []>(vector_size_);
        y_val_ = std::make_unique_for_overwrite<float []>(vector_size_);
        z_val_ = std::make_unique_for_overwrite<float []>(vector_size_);
        psi_val_ = std::make_unique_for_overwrite<float []>(vector_size_);
    }

    int s = std::max(XPLMGetDatab(acf_type_dr, nullptr, 0, 0),
                     XPLMGetDatab(flight_id_dr, nullptr, 0, 0));
    if (s > byte_area_size_) {
        byte_area_size_ = s + 512;
        log_msg("allocated byte_area_size_ %d", byte_area_size_);
        acf_type_val_ = std::make_unique_for_overwrite<char []>(byte_area_size_);
        flight_id_val_ = std::make_unique_for_overwrite<char []>(byte_area_size_);
    }

    LOAD_DR(i, flight_phase);
    LOAD_DR(i, traffic_type);
    LOAD_DR(f, x);
    LOAD_DR(f, y);
    LOAD_DR(f, z);
    LOAD_DR(f, psi);

    char *acf_type_ptr = acf_type_val_.get();
    int acf_type_len = XPLMGetDatab(acf_type_dr, acf_type_ptr, 0, byte_area_size_);
    log_msg("acf_type_len: %d", acf_type_len);
    if (acf_type_len > 0)
        acf_type_ptr[acf_type_len - 1] = '\0';

    char *flight_id_ptr = flight_id_val_.get();
    int flight_id_len = XPLMGetDatab(flight_id_dr, flight_id_ptr, 0, byte_area_size_);
    log_msg("flight_id_len: %d", flight_id_len);
    if (flight_id_len > 0)
        flight_id_ptr[flight_id_len - 1] = '\0';

    std::unordered_map<std::string, int> dref_planes;
    dref_planes.reserve(n_planes);

    int spawn_remain = kSpawnPerRun;
    for (int i = 0; i < n_planes; i++) {
        if (flight_id_len <=0 || acf_type_len <= 0) {
            log_msg("ERROR: not enough values in byte arrays");
            break;
        }

        // process byte areas
        // save pointers and advance
        const char *fid_ptr = flight_id_ptr;
        const char *type_ptr = acf_type_ptr;

        int len = strlen(flight_id_ptr) + 1;
        flight_id_len -= len;
        flight_id_ptr += len;
        len = strlen(acf_type_ptr) + 1;
        acf_type_len -= len;
        acf_type_ptr += len;

        if (traffic_type_val_[i] != PT_Airline)
            continue;

        // filter out:
        // FP_Parked for docking, FP_Startup for undocking (=beacon on in openSAM logic)
        // and close enough
        FlightPhase flight_phase = (FlightPhase)flight_phase_val_[i];
        if (!(flight_phase == FP_Parked || flight_phase == FP_Startup)
            || len2f(x_val_[i] - my_plane.x(), z_val_[i] - my_plane.z()) > kMpMaxDist)
            continue;

        // flight_id
        std::string flight_id{fid_ptr}, acf_type{type_ptr};
        dref_planes[flight_id] = i;

        try {
            auto & pr = mp_planes_.at(flight_id);
            MpPlane_tgxp* mp_plane = static_cast<MpPlane_tgxp*>(pr.get());
            mp_plane->update(flight_phase == FP_Startup); // we take that as beacon on switch
        } catch(const std::out_of_range& ex) {
            if (flight_phase == FP_Parked) {    // new creation only in parked state
                if (--spawn_remain < 0)
                    break;
                mp_planes_.emplace(flight_id,
                                   new MpPlane_tgxp(i, flight_id, acf_type,
                                                    x_val_[i], y_val_[i], z_val_[i], psi_val_[i]));
            }
        }
        //log_msg("TGXP: %d, %d, %s, %s %0.2f, %0.2f, %0.2f",
        //        i, flight_phase, flight_id.c_str(), acf_type.c_str(), x_val_[i], y_val_[i], z_val_[i]);
    }

    // loop over mp_planes and delete the ones that are no longer in drefs
    for (auto & mp : mp_planes_) {
        const std::string& key = mp.first;
        Plane& plane = *(mp.second);

        try {
            dref_planes.at(key);
        } catch(const std::out_of_range& ex) {
            log_msg("pid=%d not longer exists, deleted", plane.id_);
            mp_planes_.erase(key);
        }
    }
    log_msg("------------------ MP active planes found: %d -----------------", (int)mp_planes_.size());
    return 2.0f;
}
