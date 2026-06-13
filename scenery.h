//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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

#include <string>
#include <vector>
#include <memory>

#include "os_anim.h"

static constexpr float kD2R = std::numbers::pi/180.0;

struct SceneryPacks {
    std::string openSAM_Library_path;
    std::string SAM_Library_path;

    std::vector<std::string> sc_paths;

    SceneryPacks(const std::string& xp_dir);
};

class Scenery {
  public:
    static std::vector<Scenery> sceneries;

    std::string name_;
    std::string arpt_icao_;

    std::vector<SamObj> sam_objs_;
    std::vector<SamAnim> sam_anims_;

    // 'iterators' into the global sam_jw_list for the jetways learned from the scenery's sam.xml
    // only used for testing and debugging, not for actual lookup, which is done by quadtree
    int jw_idx_start_ = 0;
    int jw_idx_end_ = 0;

    Scenery() {
        sam_objs_.reserve(50);
        sam_anims_.reserve(50);
    }

    // Not copyable or assignable
    Scenery(const Scenery&) = delete;
    Scenery& operator=(const Scenery&) = delete;
    Scenery(Scenery&&) = default;

    static Scenery *FindScenery(float lat, float lon);
    // a poor man's factory for creating sceneries, return max # of stands in sam sceneries
    static void CollectSceneries(const SceneryPacks& scp, int& max_sam_stands);
};
