//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2025, 2026  Holger Teutsch
//    Copyright (C) 2026  Skirmish Mode Games Inc (mpadapter_batc: the BeyondATC traffic adapter)
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
#include <cmath>
#include <algorithm>

#include "XPLMPlugin.h"

#include "opensam.h"
#include "my_plane.h"
#include "mpadapter_batc.h"
#include "log_msg.h"

constexpr int kSpawnPerRun = 10;  // new Planes per update run
constexpr int kStringSlot = 8;    // bytes per slot of the flight_id / icao_type byte arrays

static const char* kBatcPluginSignature = "net.beyondatc.xplane_plugin";

static XPLMDataRef count_dr, id_dr, acf_type_dr, flight_id_dr,  // identity
    x_dr, y_dr, z_dr, psi_dr,                                   // position
    on_ground_dr, jetway_dr;                                    // state

// the values of beyondatc/ai/jetway
enum JetwayIntent {
    JW_NONE = 0,    // no request: a docked jetway stays docked
    JW_DOCK = 1,    // parked at its stand, a jetway may dock
    JW_UNDOCK = 2,  // leaving, undock now
};

class MpPlane_batc : public OsPlane {
    std::string flight_id_;
    int last_intent_{JW_DOCK};

   public:
    MpPlane_batc(const std::string& flight_id, const std::string& acf_type, float x, float y, float z, float psi);
    ~MpPlane_batc() override {
    }

    // -> false when the plane should be deleted
    bool update(int intent, bool on_ground);

    bool auto_mode() const override {
        return true;
    }
    bool dock_requested() override {
        return true;
    }
};

MpPlane_batc::MpPlane_batc(const std::string& flight_id, const std::string& acf_type, float x, float y, float z,
                           float psi) {
    flight_id_ = flight_id;

    on_ground_ = true;  // otherwise we were not here
    parkbrake_set_ = true;

    LogMsg("pid={:03}, constructing MpPlane {}/{}", id_, flight_id_, acf_type);

    // first an optional translation to a generic icao code
    auto it = acf_generic_type_map.find(acf_type);
    if (it != acf_generic_type_map.end())
        icao_ = it->second;
    else
        icao_ = acf_type;

    MpAdapter::LoadDoorInfo(*this);
    if (door_info_.empty()) {
        LogMsg("pid={:03}, {}: door 1 is not defined in door_info_map, deactivating slot", id_, acf_type);
        state_ = kDisabled;
        return;
    }

    // The published position is the origin of the drawn model (the CSL "ZERO" for the
    // Bluebell models). No longitudinal adjustment at this time.
    x_ = x;
    z_ = z;

    // get y for ground level
    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo)) {
        LogMsg("terrain probe failed???");
    }
    y_ = probeinfo.locationY;

    psi_ = psi;

    LogMsg("pid={:03}, icao: {}, found door 1 in door_info_map: x: {:.2}, y: {:.2}, z: {:.2}", id_, icao_,
           door_info_[0].x, door_info_[0].y, door_info_[0].z);

    state_ = kIdle;
}

bool MpPlane_batc::update(int intent, bool on_ground) {
    if (state_ == kDisabled)
        return true;  // keep the slot, so the plane is not constructed again and again

    // An aircraft that left the ground has left its stand for sure.
    if (!on_ground)
        return false;

    // Undocked and no new request: the aircraft moves off, the plane goes. Otherwise the
    // IDLE state would see "on the ground, beacon off" and dock the taxiing aircraft again.
    if (last_intent_ == JW_UNDOCK && intent == JW_NONE)
        return false;
    last_intent_ = intent;

    beacon_on_ = (intent == JW_UNDOCK);

    // Jetways are only dockable if they were rendered once.
    // As they come in view over time we just retry a docking attempt if the plane is stuck
    // in CANT_DOCK.
    if (!beacon_on_ && state_ == kCantDock && now > state_change_ts_ + 60.0f)
        state_ = kParked;

    LogMsg("MP update: pid={:03}, icao: {}, id: {}, intent: {}, beacon: {}, state: {}", id_, icao_,
           flight_id_, intent, beacon_on_, state_str_[state_]);
    return true;
}

//==============  MpAdapter_batc ========================================
bool MpAdapter_batc::probe() {
    if (jetway_dr == nullptr)
        jetway_dr = XPLMFindDataRef("beyondatc/ai/jetway");

    if (jetway_dr == nullptr)
        return false;

    XPLMPluginID batc = XPLMFindPluginBySignature(kBatcPluginSignature);
    return (batc != XPLM_NO_PLUGIN_ID && XPLMIsPluginEnabled(batc));
}

MpAdapter_batc::MpAdapter_batc() {
    LogMsg("MpAdapter_batc constructor");
    static bool init_done{false};

    if (!init_done) {
        // jetway_dr is already done in probe
        count_dr = XPLMFindDataRef("beyondatc/ai/count");
        id_dr = XPLMFindDataRef("beyondatc/ai/id");
        acf_type_dr = XPLMFindDataRef("beyondatc/ai/icao_type");
        flight_id_dr = XPLMFindDataRef("beyondatc/ai/flight_id");
        x_dr = XPLMFindDataRef("beyondatc/ai/x");
        y_dr = XPLMFindDataRef("beyondatc/ai/y");
        z_dr = XPLMFindDataRef("beyondatc/ai/z");
        psi_dr = XPLMFindDataRef("beyondatc/ai/psi");
        on_ground_dr = XPLMFindDataRef("beyondatc/ai/on_ground");
        init_done = true;
    }
}

MpAdapter_batc::~MpAdapter_batc() {
    LogMsg("MpAdapter_batc destructor");
}

#define LOAD_DR(type, name)                                                                     \
    {                                                                                           \
        [[maybe_unused]] int l = XPLMGetDatav##type(name##_dr, name##_val_.get(), 0, n_planes); \
        assert(l == n_planes);                                                                  \
    }

// static
float MpAdapter_batc::update() {
    int n_planes = XPLMGetDatai(count_dr);
    LogMsg("MpPlane_batc drefs #: {}", n_planes);

    if (n_planes > vector_size_) {
        vector_size_ = std::max(n_planes + 50, 200);
        LogMsg("allocated vector_size_ {}", vector_size_);
        id_val_ = std::make_unique_for_overwrite<int[]>(vector_size_);
        on_ground_val_ = std::make_unique_for_overwrite<int[]>(vector_size_);
        jetway_val_ = std::make_unique_for_overwrite<int[]>(vector_size_);
        x_val_ = std::make_unique_for_overwrite<float[]>(vector_size_);
        y_val_ = std::make_unique_for_overwrite<float[]>(vector_size_);
        z_val_ = std::make_unique_for_overwrite<float[]>(vector_size_);
        psi_val_ = std::make_unique_for_overwrite<float[]>(vector_size_);
        acf_type_val_ = std::make_unique_for_overwrite<char[]>(vector_size_ * kStringSlot);
        flight_id_val_ = std::make_unique_for_overwrite<char[]>(vector_size_ * kStringSlot);
    }

    // the arrays can shrink between the count and the reads: take what is there
    n_planes = std::min(n_planes, XPLMGetDatavi(id_dr, nullptr, 0, 0));

    LOAD_DR(i, id);
    LOAD_DR(i, on_ground);
    LOAD_DR(i, jetway);
    LOAD_DR(f, x);
    LOAD_DR(f, y);
    LOAD_DR(f, z);
    LOAD_DR(f, psi);

    XPLMGetDatab(acf_type_dr, acf_type_val_.get(), 0, n_planes * kStringSlot);
    XPLMGetDatab(flight_id_dr, flight_id_val_.get(), 0, n_planes * kStringSlot);

    std::unordered_map<std::string, bool> dref_planes;
    dref_planes.reserve(n_planes * 2);

    int spawn_remain = kSpawnPerRun;
    for (int i = 0; i < n_planes; i++) {
        if (!on_ground_val_[i] || std::hypot(x_val_[i] - my_plane->x(), z_val_[i] - my_plane->z()) > kMpMaxDist)
            continue;

        char* cptr = flight_id_val_.get() + i * kStringSlot;
        cptr[kStringSlot - 1] = '\0';  // play it safe
        std::string flight_id = cptr;

        cptr = acf_type_val_.get() + i * kStringSlot;
        cptr[kStringSlot - 1] = '\0';
        std::string acf_type = cptr;

        // the BeyondATC id is the stable key; the slot of an aircraft can change
        std::string key = std::to_string(id_val_[i]);
        int intent = jetway_val_[i];

        auto it = mp_planes_.find(key);
        if (it != mp_planes_.end()) {
            MpPlane_batc* mp_plane = static_cast<MpPlane_batc*>(it->second.get());
            if (mp_plane->update(intent, true))
                dref_planes[key] = true;
            // else: not in dref_planes -> deleted below
        } else {
            if (intent == JW_DOCK) {  // new creation only when BeyondATC asks for a jetway
                if (--spawn_remain < 0)
                    break;
                mp_planes_.emplace(key, new MpPlane_batc(flight_id, acf_type, x_val_[i], y_val_[i], z_val_[i],
                                                         psi_val_[i]));
                dref_planes[key] = true;
            }
        }
    }

    // loop over mp_planes_ and delete the ones that are no longer in drefs
    for (auto it = mp_planes_.begin(); it != mp_planes_.end();) {
        const std::string& key = it->first;
        OsPlane& plane = *(it->second);

        if (!dref_planes.contains(key)) {
            LogMsg("pid={:03} key: {} not longer exists, deleted", plane.id_, key);
            it = mp_planes_.erase(it);
        } else
            ++it;
    }

    if (!mp_planes_.empty())
        LogMsg("------------------ MP active planes found: {} -----------------", mp_planes_.size());
    return 2.0f;
}
