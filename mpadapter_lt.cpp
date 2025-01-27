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
#include "mpadapter_lt.h"

constexpr int kSpawnPerRun = 10;    // new Planes per update run
constexpr float kDefaultWait = 3.0; // s

class MpPlane_lt : public Plane {
    std::string flight_id_;

    float scan_mp_planes();

  public:
    MpPlane_lt(const std::string& flight_id, const std::string& icao,
               float x, float y, float z, float psi);
    ~MpPlane_lt() override {}

    void update(bool beacon);

    bool auto_mode() const override { return true; }
    bool dock_requested() override { return true; }
};


MpPlane_lt::MpPlane_lt(const std::string& flight_id, const std::string& icao,
                       float x, float y, float z, float psi)
{
    flight_id_ = flight_id;

    on_ground_ = true;  // otherwise we were not here
    parkbrake_set_ = true;

    log_msg("pid=%d, constructing MpPlane %s/%s", id_, flight_id_.c_str(), icao.c_str());

    n_door_ = 0;
    try {
        // first an optional translation to a generic icao code
        try {
            icao_ = acf_generic_type_map.at(icao);
        } catch(const std::out_of_range& ex) {
            icao_ = icao;
        }

        door_info_[0] = csl_door_info_map.at(icao_ + '1');
        n_door_++;

        x_ = x; z_ = z; psi_ = psi;

        // refine y for ground level
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
            log_msg("terrain probe failed???");
        }
        y_ = probeinfo.locationY;

        log_msg("pid=%d, icao: %s, found door 1 in door_info_map: x: %0.2f, y: %0.2f, z: %0.2f",
                id_, icao_.c_str(), door_info_[0].x, door_info_[0].y, door_info_[0].z);
    } catch(const std::out_of_range& ex) {
        log_msg("pid=%d, %s: door 1 is not defined in door_info_map, deactivating slot", id_, icao_.c_str());
        state_ = DISABLED;
        return;
    }

    // door 2 + 3 are optional
    try {
        door_info_[1] = csl_door_info_map.at(icao_ + '2');
        n_door_++;
        door_info_[2] = csl_door_info_map.at(icao_ + '3');
        n_door_++;
    } catch(const std::out_of_range& ex) {}

    state_ = IDLE;
}

void
MpPlane_lt::update(bool beacon)
{
    if (state_ == DISABLED)
        return;

    beacon_on_ = beacon;

    // Jetways are only dockable if they were rendered once.
    // As they come in view over time we just retry a docking attempt if the plane is stuck
    // in CANT_DOCK.
    if (!beacon_on_ && state_ == CANT_DOCK && now > state_change_ts_ + 60.0f)
        state_ = PARKED;

    log_msg("MP update: pid=%02d, icao: %s, id: %s, beacon: %d, parkbrake_set: %d, state: %s",
            id_, icao_.c_str(), flight_id_.c_str(), beacon_on_, parkbrake_set_,
            state_str_[state_]);

}

//==============  MpAdapter_lt ========================================
bool MpAdapter_lt::probe()
{
    return LTAPIConnect::isLTAvail();
}

MpAdapter_lt::MpAdapter_lt()
{
    log_msg("MpAdapter_lt constructor");
}

MpAdapter_lt::~MpAdapter_lt()
{
    log_msg("MpAdapter_lt destructor");
}

// static
float MpAdapter_lt::update()
{
    float my_lat = my_plane.lat();
    float my_lon = my_plane.lon();
    float my_cos_lat = cosf(my_lat * D2R);

    lt_connect_.UpdateAcList();
    const MapLTAPIAircraft& lt_planes = lt_connect_.getAcMap();

    int spawn_remain = kSpawnPerRun;
    for (auto & mltp : lt_planes) {
        const LTAPIAircraft& lt_plane = *mltp.second;

        // filter, optimize for reject, order cheap to expensive
        if (!(lt_plane.isOnGnd() && lt_plane.isVisible()))
            continue;

        LTAPIAircraft::LTFlightPhase flight_phase = lt_plane.getPhase();
        if (flight_phase != LTAPIAircraft::FPH_PARKED && flight_phase != LTAPIAircraft::FPH_TAXI)
            continue;

        const std::string& flight_id = lt_plane.getRegistration();
        if (flight_id.size() == 0)       // likely a ground vehicle
            continue;

        if (len2f((lt_plane.getLon() - my_lon) * my_cos_lat,
                   lt_plane.getLat() - my_lat) * LAT_2_M > kMpMaxDist)
            continue;

        //log_msg("LT lat/lon: %0.2f, %0.2f", lt_plane.getLat(), lt_plane.getLon());
        double x, y, z;
        lt_plane.getLocalCoord(x, y, z);
        float psi = lt_plane.getHeading();      // TODO: mag to true adjustment

        const std::string& icao = lt_plane.getModelIcao();
        const std::string& key = lt_plane.getKey();

        try {
            auto & pr = mp_planes_.at(key);
            MpPlane_lt* mp_plane = static_cast<MpPlane_lt*>(pr.get());
            mp_plane->update(lt_plane.getLights().beacon);
        } catch(const std::out_of_range& ex) {
            if (flight_phase == LTAPIAircraft::FPH_PARKED) {    // new creation only in parked state
                if (--spawn_remain < 0)
                    break;
                mp_planes_.emplace(key, new MpPlane_lt(flight_id, icao, x, y, z, psi));
            }
        }
        //log_msg("LT: %d, %s, %s %0.2f, %0.2f, %0.2f",
        //        flight_phase, flight_id.c_str(), icao.c_str(), x, y, z);
    }

    // loop over mp_planes and delete the ones that are no longer in drefs
    for (auto & mp : mp_planes_) {
        const std::string& key = mp.first;
        Plane& plane = *(mp.second);

        try {
            lt_planes.at(key);
        } catch(const std::out_of_range& ex) {
            log_msg("pid=%d not longer exists, deleted", plane.id_);
            mp_planes_.erase(key);
        }
    }
    log_msg("------------------ MP active planes found: %d -----------------", (int)mp_planes_.size());
    return kDefaultWait;
}
