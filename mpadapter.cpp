//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2025, 2026  Holger Teutsch
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

#include <cassert>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include "os_plane.h"
#include "mpadapter.h"
#include "mpadapter_xpilot.h"
#include "mpadapter_tgxp.h"
#include "mpadapter_lt.h"
#include "mpadapter_vat.h"
#include "log_msg.h"

std::unordered_map<std::string, DoorInfo> csl_door_info_map;
std::unordered_map<std::string, std::string> acf_generic_type_map;

static bool active;

MpAdapter::~MpAdapter() {
    mp_planes_.clear();
    active = false;
}

std::unique_ptr<MpAdapter> MpAdapter_factory() {
    // ensure that we only have one active adapter
    assert(!active);

    std::unique_ptr<MpAdapter> adapter;

    if (MpAdapter_xPilot::probe())
        adapter = std::unique_ptr<MpAdapter>(new MpAdapter_xPilot());
    else if (MpAdapter_lt::probe())
        adapter = std::unique_ptr<MpAdapter>(new MpAdapter_lt());
    else if (MpAdapter_tgxp::probe())
        adapter = std::unique_ptr<MpAdapter>(new MpAdapter_tgxp());
    else if (MpAdapter_vat::probe())
        adapter = std::unique_ptr<MpAdapter>(new MpAdapter_vat());

    active = (adapter != nullptr);
    return adapter;
}

float MpAdapter::JwStateMachine() {
    float jw_loop_delay = 10.0;
    for (auto& p : mp_planes_)
        jw_loop_delay = std::min(p.second->JwStateMachine(), jw_loop_delay);
    return jw_loop_delay;
}

void MpAdapter::Reset() {
    LogMsg("MpAdapter::Reset() called, clearing MP planes");
    mp_planes_.clear();
}

// load door info for multiplayer plane
void MpAdapter::LoadDoorInfo(OsPlane& mp_plane, const std::string& icao) {
    mp_plane.door_info_.clear();

    auto door_it = csl_door_info_map.find(icao + '1');
    if (door_it != csl_door_info_map.end())
        mp_plane.door_info_.push_back(door_it->second);
    else
        return;

    // door 2 + 3 are optional
    auto it2 = csl_door_info_map.find(icao + '2');
    if (it2 != csl_door_info_map.end())
        mp_plane.door_info_.push_back(it2->second);

    auto it3 = csl_door_info_map.find(icao + '3');
    if (it3 != csl_door_info_map.end())
        mp_plane.door_info_.push_back(it3->second);
}
