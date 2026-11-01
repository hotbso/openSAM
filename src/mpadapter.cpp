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
#include <memory>
#include <algorithm>
#include <fstream>

#include "opensam.h"
#include "os_plane.h"
#include "mpadapter.h"
#include "mpadapter_xpilot.h"
#include "mpadapter_tgxp.h"
#include "mpadapter_lt.h"
#include "mpadapter_vat.h"
#include "mpadapter_batc.h"
#include "log_msg.h"

std::unordered_map<std::string, DoorInfo> MpAdapter::csl_door_info_map;
std::unordered_map<std::string, std::string> MpAdapter::acf_generic_type_map;

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
    else if (MpAdapter_batc::probe())
        adapter = std::unique_ptr<MpAdapter>(new MpAdapter_batc());
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
void MpAdapter::LoadDoorInfo(OsPlane& mp_plane) {
    mp_plane.door_info_.clear();
    std::string key = mp_plane.icao() + '1';

    auto door_it = csl_door_info_map.find(key);
    if (door_it != csl_door_info_map.end())
        mp_plane.door_info_.push_back(door_it->second);
    else
        return;

    // door 2 + 3 are optional
    key.back() = '2';
    auto it2 = csl_door_info_map.find(key);
    if (it2 != csl_door_info_map.end())
        mp_plane.door_info_.push_back(it2->second);

    key.back() = '3';
    auto it3 = csl_door_info_map.find(key);
    if (it3 != csl_door_info_map.end())
        mp_plane.door_info_.push_back(it3->second);
}

// load config files for door info and aircraft generic type mappings
bool MpAdapter::LoadMaps() {
    std::string fn = base_dir + "csl_door_position.txt";
    std::ifstream f(fn);
    if (!f.is_open()) {
        LogMsg("Error loading '{}'", fn);
        return false;
    }

    LogMsg("Building door_info_map from {}", fn);

    std::string line;
    line.reserve(100);

    while (std::getline(f, line)) {
        if (line.empty())
            continue;

        if (line.back() == '\r')
            line.pop_back();

        if (line.empty() || line[0] == '#')
            continue;

        char icao[5];
        int d;
        float x, y, z;
        if (5 == sscanf(line.c_str(), "%4s %d %f %f %f", icao, &d, &x, &y, &z)) {
            if (d < 1 || d > kMaxDoor) {
                LogMsg("invalid entry: '{}'", line);
                continue;
            }

            char c = d + '0';
            DoorInfo di{x, y, z};
            csl_door_info_map[std::string(icao) + c] = di;
        }
    }

    LogMsg("{} mappings loaded", csl_door_info_map.size());
    f.close();

    fn = base_dir + "acf_generic_type.txt";
    f.open(fn);
    if (!f.is_open()) {
        LogMsg("Error loading '{}'", fn);
        return false;
    }

    LogMsg("Building acf_generic_type_map from {}", fn);

    while (std::getline(f, line)) {
        if (line.empty())
            continue;

        if (line.back() == '\r')
            line.pop_back();

        if (line.empty() || line[0] == '#')
            continue;

        char code[5];
        char icao[5];
        if (2 == sscanf(line.c_str(), "%4s %4s", code, icao))
            acf_generic_type_map[std::string(code)] = std::string(icao);

    }

    LogMsg("{} mappings loaded", acf_generic_type_map.size());
    f.close();
    return true;
}
