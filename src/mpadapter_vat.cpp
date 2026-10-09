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

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <numbers>
#include <cmath>
#include <print>

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"

#include "opensam.h"
#include "mpadapter_vat.h"
#include "log_msg.h"

#define JSON_USE_IMPLICIT_CONVERSIONS 0
#include "nlohmann/json.hpp"
using json = nlohmann::json;

constexpr int kSpawnPerRun = 10;    // new Planes per update run

static XPLMDataRef planes_json_dr, json_freq_dr;

class MpPlane_vat : public OsPlane {
    const std::string flight_id_;
    const std::string flight_no_;
    const std::string icao_;

   public:
    MpPlane_vat(const std::string& flight_id, const std::string& flight_no, const std::string& acf_icao, float x, float y, float z, float psi);
    ~MpPlane_vat() override {}

    void update(bool beacon);

    bool auto_mode() const override { return true; }
    bool dock_requested() override { return true; }
};

MpPlane_vat::MpPlane_vat(const std::string& flight_id, const std::string& flight_no, const std::string& acf_icao, float x, float y, float z, float psi)
    : flight_id_(flight_id), flight_no_(flight_no), icao_(acf_icao) {

    LogMsg("pid=%d, constructing MpPlane %s/%s/%s", id_, flight_id_.c_str(), flight_no_.c_str(), acf_icao.c_str());

    on_ground_ = true;  // otherwise we were not here
    parkbrake_set_ = true;

    MpAdapter::LoadDoorInfo(*this, icao_);
    if (!door_info_.empty()) {
        // final terrain probe
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
            LogMsg("terrain probe failed???");
        }
        x_ = probeinfo.locationX;
        y_ = probeinfo.locationY;
        z_ = probeinfo.locationZ;
        psi_ = psi;

        LogMsg("pid=%d, icao: %s, found door 1 in door_info_map: x: %0.2f, y: %0.2f, z: %0.2f", id_, icao_.c_str(),
               door_info_[0].x, door_info_[0].y, door_info_[0].z);
    } else {
        LogMsg("pid=%d, %s: door 1 is not defined in door_info_map, deactivating slot", id_, acf_icao.c_str());
        state_ = kDisabled;
        return;
    }

    state_ = kIdle;
}

void MpPlane_vat::update(bool beacon) {
    if (state_ == kDisabled)
        return;

    beacon_on_ = beacon;

    // Jetways are only dockable if they were rendered once.
    // As they come in view over time we just retry a docking attempt if the plane is stuck
    // in CANT_DOCK.
    if (!beacon_on_ && state_ == kCantDock && now > state_change_ts_ + 60.0f)
        state_ = kParked;

    LogMsg("VAT update: pid=%02d, id: %s, flight_no: %s, icao: %s, beacon: %d, state: %s", id_, flight_id_.c_str(),
           flight_no_.c_str(), icao_.c_str(), beacon_on_, state_str_[state_]);
}

//==============  MpAdapter_vat ========================================
bool MpAdapter_vat::probe() {
    if (planes_json_dr == nullptr)
        planes_json_dr = XPLMFindDataRef("virtualairtraffic/traffic/planes_json");

    if (planes_json_dr)
        LogMsg("planes_json dataref found");

    return (planes_json_dr != nullptr);
}

MpAdapter_vat::MpAdapter_vat() {
    static bool init_done;
    LogMsg("MpAdapter_vat constructor");
    if (!init_done) {
        init_done = true;
        json_freq_dr = XPLMFindDataRef("virtualairtraffic/traffic/planes_freq_s");
    }

    XPLMSetDataf(json_freq_dr, 1.0f);
}

MpAdapter_vat::~MpAdapter_vat() {
    LogMsg("MpAdapter_vat destructor");
    XPLMSetDataf(json_freq_dr, 0.0f);
}

// static
float MpAdapter_vat::update() {
    int json_len = XPLMGetDatab(planes_json_dr, NULL, 0, 0);

    // Reallocate json_buf_ if necessary PLUS room for a trailing '\0'
    if (json_len >= json_buf_len_) {
        json_buf_len_ = json_len + 512;
        json_buf_ = std::make_unique_for_overwrite<char[]>(json_buf_len_);
        LogMsg("reallocated json_buf_ to size %d (json_len: %d)", json_buf_len_, json_len);
    }

    int n = XPLMGetDatab(planes_json_dr, json_buf_.get(), 0, json_buf_len_);
    if (n <= 0) {
        LogMsg("ERROR: Failed to read planes_json");
        return 0.0f;
    }

    assert(n < json_buf_len_);

    json_buf_[n] = '\0';
    // LogMsgRaw(json_buf_.get());

    json data_obj;
    try {
        data_obj = json::parse(json_buf_.get());
        // LogMsgRaw(data_obj.dump(4));
    } catch (const std::exception& e) {
        LogMsg("Invalid json from planes_json: %s", e.what());
    }

    if (data_obj.is_null()) {
        LogMsg("No valid data in planes_json");
        return 5.0f;
    }

    std::unordered_map<std::string, bool> json_planes;
    json_planes.reserve(200);
    try {
        int spawn_remain = kSpawnPerRun;

        for (const auto& plane : data_obj.at("planes")) {
            if (!plane.at("rendered").get<bool>())
                continue;

            const auto& state = plane.at("state").get<std::string>();
            if (state != "Static" && state != "Parked" && state != "Waiting")
                continue;

            int plane_id = plane.at("planeID").get<int>();
            const auto& acf_icao = plane.at("spawnedAircraftICAO").get<std::string>();
            const auto beacon_on = plane.at("beaconLightsOn").get<bool>();
            //LogMsg("Plane ID: %d, State: %s, Aircraft Type: %s, Beacon On: %s", plane_id, state.c_str(), acf_icao.c_str(), beacon_on ? "true" : "false");

            const std::string flight_id = std::format("ID{:05}", plane_id);

            json_planes[flight_id] = true;
            auto it = mp_planes_.find(flight_id);
            if (it != mp_planes_.end()) {
                MpPlane_vat* mp_plane = static_cast<MpPlane_vat*>(it->second.get());
                mp_plane->update(beacon_on);
            } else {
                if (!beacon_on) {  // new creation only in parked state
                    if (--spawn_remain < 0)
                        break;
                    float x = plane.at("localX").get<float>();
                    float y = plane.at("localY").get<float>();
                    float z = plane.at("localZ").get<float>();
                    float psi = plane.at("headingDeg").get<float>();
                    const auto& flight_no = plane.at("flightNumber").get<std::string>();
                    mp_planes_.emplace(flight_id, std::make_unique<MpPlane_vat>(flight_id, flight_no, acf_icao, x, y, z, psi));
                }
            }
        }
    } catch (const std::exception& e) {
        LogMsg("Error processing planes_json data: %s", e.what());
        return 5.0f;
    }

    // loop over mp_planes_ and delete the ones that are no longer in json_planes
    for (auto it = mp_planes_.begin(); it != mp_planes_.end();) {
        const std::string& key = it->first;

        if (!json_planes.contains(key)) {
            OsPlane& plane = *(it->second);
            LogMsg("pid=%02d key: %s not longer exists, deleted", plane.id_, key.c_str());
            it = mp_planes_.erase(it);
        } else
            ++it;
    }

    if (!mp_planes_.empty())
        LogMsg("------------------ MP active planes found: %d -----------------", (int)mp_planes_.size());


    return 2.0f;
}
