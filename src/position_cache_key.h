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

#pragma once

#include <functional>
#include <cstdint>

// for quick lookup of objects by position (x/y/z)
struct PositionCacheKey {
    // x, z are obj_x/z coordinates of an object, used for quick lookup of object by position
    float x, z;

    bool operator==(const PositionCacheKey &other) const {
        return (x == other.x) && (z == other.z);
    }
};

struct PositionCacheKeyHasher {
    std::size_t operator()(const PositionCacheKey& key) const {
        // x, z should be enough to identify an object
        int64_t x_int = (int64_t)(key.x * 100.0f);  // scale to preserve 2 decimal places
        int64_t z_int = (int64_t)(key.z * 100.0f);
        // 0x9e3779b9 is the Golden Ratio constant used to prevent bit clustering
        std::size_t seed = std::hash<int64_t>{}(x_int);
        seed ^= std::hash<int64_t>{}(z_int) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};
