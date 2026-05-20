//
//    AutoDGS / openSAM: Manage DGS
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

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <regex>
#include <ranges>
#include <vector>
#include <sstream>

#include "airport.h"
#include "plane.h"
#include "dgs_impl.h"

#include "log_msg.h"

#include "XPLMPlanes.h"

static constexpr float kF2M = 0.3048;               // 1 ft [m]

namespace dgs {

void FlexDref::MapDref() {
    if (mapped_)
        return;
    mapped_ = true;

    dr_ = XPLMFindDataRef(name_.c_str());

    if (dr_ == nullptr) {
        LogMsg("dataref '%s' not found", name_.c_str());
        throw std::runtime_error("config error, dataref not found");
    }

    type_id_ = XPLMGetDataRefTypes(dr_);
    LogMsg("Mapped dataref '%s', type_id: %d, trigger_value: %f", name_.c_str(), type_id_, cmp_value_);
}

void FlexDref::Clear() {
    name_.clear();
    dr_ = nullptr;
    mapped_ = false;
    cmp_value_ = 0.0;
    cmp_type_ = -1;
}

bool FlexDref::empty() const { return name_.empty(); }

void FlexDref::Set(const std::string& name_cmp_value) {
    mapped_ = false;

    std::istringstream iss(name_cmp_value);
    auto words = std::views::istream<std::string>(iss) | std::ranges::to<std::vector<std::string>>();

    if (words.size() >= 3) {
        name_ = words[0];
        const std::string& cmp = words[1];
        if (cmp == "eq") {
            cmp_type_ = 0;
        } else if (cmp == "ge") {
            cmp_type_ = 1;
        } else if (cmp == "gt") {
            cmp_type_ = 2;
        } else if (cmp == "le") {
            cmp_type_ = 3;
        } else if (cmp == "lt") {
            cmp_type_ = 4;
        } else {
            LogMsg("invalid comparison type '%s' for '%s'", cmp.c_str(), name_.c_str());
            throw std::runtime_error("config error, invalid comparison type");
        }

        try {
            cmp_value_ = std::stof(words[2]);
        } catch (const std::exception& e) {
            LogMsg("invalid comparison value in '%s': %s", name_cmp_value.c_str(), e.what());
            throw std::runtime_error("config error, invalid comparison value");
        }
    } else {
        // just return the value, e.g. for "pax_no_dref	AirbusFBW/NoPax"
        cmp_type_ = -1; // not a trigger dref
        name_ = name_cmp_value;
        cmp_value_ = -1.0f;
    }
}

int FlexDref::GetTriggered() {
    float value = GetValue();
    switch (cmp_type_) {
        case 0: return value == cmp_value_;
        case 1: return value >= cmp_value_;
        case 2: return value > cmp_value_;
        case 3: return value <= cmp_value_;
        case 4: return value < cmp_value_;
        default: return 0; // not a trigger dref
    }
}

float FlexDref::GetValue() {
    MapDref();
    float value;
    if (type_id_ == xplmType_Int) {
        value = XPLMGetDatai(dr_);
    } else if (type_id_ == xplmType_Float) {
        value = XPLMGetDataf(dr_);
    } else {
        LogMsg("unsupported dataref type for '%s'", name_.c_str());
        throw std::runtime_error("config error, unsupported dataref type");
    }

    return value;
}

void FlexCmd::MapCmd() {
    if (mapped_)
        return;
    mapped_ = true;

    if (is_cmd_) {
        cmd_ = XPLMFindCommand(name_.c_str());
        if (cmd_ == nullptr) {
            LogMsg("command '%s' not found", name_.c_str());
            throw std::runtime_error("config error, command not found");
        }
    } else {
        dr_ = XPLMFindDataRef(name_.c_str());
        if (dr_ == nullptr) {
            LogMsg("dataref '%s' not found", name_.c_str());
            throw std::runtime_error("config error, dataref not found");
        }
    }

    LogMsg("Mapped %s '%s', dr_value: %f", is_cmd_ ? "command" : "dataref", name_.c_str(), dr_value_);
}

void FlexCmd::Clear() {
    name_.clear();
    cmd_ = nullptr;
    dr_ = nullptr;
    dr_value_ = 1.0f;
    mapped_ = false;
    is_cmd_ = false;
}

bool FlexCmd::empty() const { return name_.empty(); }

void FlexCmd::Set(const std::string& name) {
    mapped_ = false;
    if (name.starts_with("cmd ")) {
        is_cmd_ = true;
        name_ = name.substr(4);
    } else if (name.starts_with("dref ")) {
        is_cmd_ = false;
        std::string dref_part = name.substr(5);
        auto sep = dref_part.find_first_of(" \t");
        if (sep != std::string::npos) {
            name_ = dref_part.substr(0, sep);
            try {
                dr_value_ = std::stof(dref_part.substr(sep + 1));
            } catch (const std::exception& e) {
                LogMsg("invalid dataref value in '%s': %s", name.c_str(), e.what());
                throw std::runtime_error("config error, invalid dataref value");
            }
        } else {
            name_ = dref_part;
            dr_value_ = 1.0f; // default
        }
    } else {
        LogMsg("invalid FlexCmd name '%s', must start with 'cmd ' or 'dref '", name.c_str());
        throw std::runtime_error("config error, invalid FlexCmd name");
    }
}

void FlexCmd::Execute() {
    MapCmd();
    if (is_cmd_) {
        XPLMCommandOnce(cmd_);
    } else {
        XPLMDataTypeID type_id = XPLMGetDataRefTypes(dr_);
        if (type_id == xplmType_Int) {
            XPLMSetDatai(dr_, (int)dr_value_);
        } else if (type_id == xplmType_Float) {
            XPLMSetDataf(dr_, dr_value_);
        } else {
            LogMsg("unsupported dataref type for '%s'", name_.c_str());
            throw std::runtime_error("config error, unsupported dataref type");
        }
    }
}

bool Plane::EnginesOn() {
    int er[8];
    int n = XPLMGetDatavi(eng_running_dr, er, 0, 8);
    for (int i = 0; i < n; i++)
        if (er[i])
            return true;

    return false;
}

void Plane::ResetBeacon() {
    beacon_state_ = beacon_last_pos_ = XPLMGetDatai(beacon_dr);
    beacon_on_ts_ = beacon_off_ts_ = -10.0;
}

bool Plane::BeaconOn(void) {
    if (use_engines_on_)
        return EnginesOn();

    // when checking the beacon guard against power transients when switching
    // to the APU generator (e.g. for the ToLiss fleet).
    // Report only state transitions if the new (off) state persisted for 3 seconds

    int beacon = XPLMGetDatai(beacon_dr);
    if (beacon) {
        if (!beacon_last_pos_) {
            beacon_on_ts_ = now;
            beacon_last_pos_ = 1;
        } else if (now > beacon_on_ts_ + 0.5)
            beacon_state_ = 1;
    } else {
        if (beacon_last_pos_) {
            beacon_off_ts_ = now;
            beacon_last_pos_ = 0;
        } else if (now > beacon_off_ts_ + 3.0)
            beacon_state_ = 0;
    }

    return beacon_state_;
}

bool Plane::SetChocks() {
    if (!set_chocks_fcmdr_.empty()) {
        set_chocks_fcmdr_.Execute();
        return true;
    }

    return false;
}

int Plane::PaxNo() {
    if (pax_no_fdr_.empty())
        return -1;
    return pax_no_fdr_.GetValue() + 0.5f;  // round upwards
}

void Plane::GetEqStatus(dgs::EqStatus& eq_status) {
    // chocks, gpu, pca, pbb status
    eq_status.chocks = eq_status.gpu = eq_status.pca = eq_status.pbb = dgs::kEqUnknown;

    static_assert(dgs::kEqUnknown == 0 && dgs::kEqOff == 1 && dgs::kEqOn == 2, "unexpected EqStatus values");

    if (!chk_fdr_.empty())
        eq_status.chocks = chk_fdr_.GetTriggered() + 1;  // +1 to convert from 0/1 to kEqOff/kEqOn

    if (!gpu_fdr_.empty())
        eq_status.gpu = gpu_fdr_.GetTriggered() + 1;

    if (!pca_fdr_.empty())
        eq_status.pca = pca_fdr_.GetTriggered() + 1;

    eq_status.pbb = PbbEqStatus();
}

std::shared_ptr<Plane> plane;

// Helper to trim whitespace
static void trim(std::string& s) {
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
};

// Helper to extract config variables for the current plane from planes.cfg, e.g. chocks dref, door pos, etc.
void Plane::GetPlaneConfig(const std::string& filepath, const std::string& target_icao,
                           const std::string& target_studio) {
    cfg_.clear();
    std::ifstream file(filepath);
    if (!file.is_open()) {
        LogMsg("Can't open plane config file '%s'", filepath.c_str());
        return;
    }

    std::string line;
    line.reserve(256);

    // Helper to check if current block matches criteria
    auto block_matches = [&]() -> bool {
        if (cfg_.empty())
            return false;

        bool icao_matches = false;
        bool studio_matches = true;

        // Evaluate RegEx for ICAO match
        if (cfg_.count("icao")) {
            try {
                // e.g. "A[0-9N]+"
                std::regex icao_regex(cfg_["icao"]);
                icao_matches = std::regex_match(target_icao, icao_regex);
            } catch (const std::regex_error& e) {
                LogMsg("Invalid regex in planes.cfg for icao: '%s', error: %s", cfg_["icao"].c_str(), e.what());
                // Skip on malformed regex
                icao_matches = false;
            }
        }

        // Evaluate Studio match (if defined in the block)
        if (cfg_.count("studio")) {
            if (cfg_["studio"] != target_studio) {
                studio_matches = false;
            }
        }

        return icao_matches && studio_matches;
    };

    while (std::getline(file, line)) {
        trim(line);
        // Blocks are separated by empty lines or comments
        if (line.empty() || line[0] == '#') {
            if (block_matches()) {
                return;
            }
            cfg_.clear();
            continue;
        }

        std::size_t separator_pos = line.find_first_of(" \t");
        if (separator_pos != std::string::npos) {
            std::string key = line.substr(0, separator_pos);
            std::string value = line.substr(separator_pos + 1);
            trim(value);
            cfg_[key] = value;
        } else {
            // Cases where an option might not have a value provided
            cfg_[line] = "";
        }
    }

    // Checking last block if file does not end with an empty line
    if (block_matches()) {
        return;
    }

    // Return empty map if no block matched
    cfg_.clear();
    return;
}

void Plane::PlaneLoadedCb() {
    pax_no_fdr_.Clear();
    chk_fdr_.Clear();
    gpu_fdr_.Clear();
    pca_fdr_.Clear();
    set_chocks_fcmdr_.Clear();

    char buffer[41]{};
    XPLMGetDatab(acf_icao_dr, buffer, 0, 40);

    for (int i = 0; i < 4; i++)
        buffer[i] = (isupper(buffer[i]) || isdigit(buffer[i])) ? buffer[i] : ' ';
    buffer[4] = '\0';
    acf_icao_ = buffer;

    float plane_cg_z = kF2M * XPLMGetDataf(acf_cg_z_dr);
    float plane_cg_y = kF2M * XPLMGetDataf(acf_cg_y_dr);

    float gear_z[2];
    if (2 == XPLMGetDatavf(acf_gear_z_dr, gear_z, 0, 2)) {  // nose + main wheel
        nw_z_ = -gear_z[0];
        mw_z_ = -gear_z[1];
    } else
        nw_z_ = mw_z_ = plane_cg_z;  // fall back to CG

    is_helicopter_ = XPLMGetDatai(is_helicopter_dr);

    pe_y_0_valid = false;
    pe_y_0 = 0.0;

    std::string studio;

    // unfortunately the *default* pilot eye y coordinate is not published in
    // a dataref, only the dynamic values.
    // Therefore we pull it from the acf file.

    char acf_path[512], acf_file[256];
    XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
    LogMsg("acf_path: '%s'", acf_path);

    int acf_has_door2 = 0;
    float acf_door2_pos[3]{};

    std::ifstream acf(acf_path);
    if (acf.is_open()) {
        std::string line;
        line.reserve(256);
        while (std::getline(acf, line)) {
            // we go the simple brute force way
            if (! is_helicopter_ && line.starts_with("P acf/_pe_xyz/1 ")) {
                if (1 == sscanf(line.c_str() + 16, "%f", &pe_y_0)) {
                    pe_y_0 -= XPLMGetDataf(acf_cg_y_dr);
                    pe_y_0 *= kF2M;
                    pe_y_0_valid = true;
                }
                continue;
            }

            if (line.starts_with("P acf/_studio ")) {
                studio = line.substr(14);
                trim(studio);
                continue;
            }

            if (line.starts_with("P acf/_has_board_2 ")) {
                sscanf(line.c_str() + 19, "%d", &acf_has_door2);
                continue;
            }

            if (line.starts_with("P acf/_board_2/0 ")) {
                if (1 == sscanf(line.c_str() + 17, "%f", &acf_door2_pos[0])) {
                    acf_door2_pos[0] *= kF2M;

                }
                continue;
            }

            if (line.starts_with("P acf/_board_2/1 ")) {
                float y;
                if (1 == sscanf(line.c_str() + 17, "%f", &y)) {
                    acf_door2_pos[1] = y * kF2M - plane_cg_y;
                }
                continue;
            }
            if (line.starts_with("P acf/_board_2/2 ")) {
                float z;
                if (1 == sscanf(line.c_str() + 17, "%f", &z)) {
                    acf_door2_pos[2] = z * kF2M - plane_cg_z;
                }
                continue;
            }
        }
    }

    GetPlaneConfig(base_dir + "planes.cfg", acf_icao_, studio);
    LogMsg("Plane config loaded for %s, studio: %s", acf_icao_.c_str(), studio.c_str());
    for (const auto& [k, v] : cfg_)
        LogMsg("  %s: %s", k.c_str(), v.c_str());

    // check flags from planes.cfg
    use_engines_on_ = cfg_.count("use_engines_on") > 0;
    dont_connect_jetway_ = cfg_.count("dont_connect_jetway") > 0;

    LogMsg("use_engines_on: %d, dont_connect_jetway: %d", use_engines_on_, dont_connect_jetway_);

    if (!cfg_.contains("door_2") && acf_has_door2) {
        // if not explicitly configured in planes.cfg, we assume that the second door is at the position from the acf file
        // stash that away for other interested parties
        cfg_["door_2"] = std::to_string(acf_door2_pos[0]) + " " + std::to_string(acf_door2_pos[1]) + " " + std::to_string(acf_door2_pos[2]);
        LogMsg("Inferred door_2 position from acf file: %s", cfg_["door_2"].c_str());
    }

    if (auto it = cfg_.find("pax_no_dref"); it != cfg_.end())
        pax_no_fdr_.Set(it->second);

    if (auto it = cfg_.find("chk_dref"); it != cfg_.end())
        chk_fdr_.Set(it->second);

    if (auto it = cfg_.find("gpu_dref"); it != cfg_.end())
        gpu_fdr_.Set(it->second);

    if (auto it = cfg_.find("pca_dref"); it != cfg_.end())
        pca_fdr_.Set(it->second);

    if (auto it = cfg_.find("chk_set"); it != cfg_.end())
        set_chocks_fcmdr_.Set(it->second);

    // the VDGS object does not like letters in the last position
    if (acf_icao_ == "A20N")
        acf_icao_ = "A320";
    else if (acf_icao_ == "A21N")
        acf_icao_ = "A321";

    LogMsg(
        "plane loaded: %s, plane_cg_z: %1.2f, nw_z: %1.2f, mw_z: %1.2f, "
        "pe_y_0_valid: %d, pe_y_0: %0.2f, is_helicopter_: %d",
        acf_icao_.c_str(), plane_cg_z, nw_z_, mw_z_, pe_y_0_valid, pe_y_0, is_helicopter_);
}

} // namespace dgs
