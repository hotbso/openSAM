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

#include <vector>
#include <array>
#include <unordered_map>
#include <string>
#include <memory>
#include <algorithm>

// Inspired by: https://pvigier.github.io/2019/08/04/quadtree-collision-detection.html

// A quadtree for 2d points identified by lon,lat working in space (-180, 180] x [-89, 89].
// It addresses the peculiarity of lon arithmetics around the antimeridian and lat clamping.
//
// This quadtree is designed to be used with items that have a bounding box (e.g. airports or jetways, which have a
// lon,lat and some "box" around its position to catch the item), and it will insert the item into all quadrants that
// intersect with its bounding box. This means that an item may be stored in multiple nodes if it intersects with
// multiple quadrants. The quadtree will subdivide nodes as necessary to ensure that no node contains more than
// kMaxItems items, or if an item cannot fit into the node's bounds.
//
// Data is static and inserted all at once, so we don't need to worry about deleting, balancing the tree or merging
// nodes. The main use case is to build the tree once and then perform many queries on it, so we optimize for query
// performance at the cost of potentially having more nodes and items stored in multiple nodes.
//
// It manages non-owning pointers of a class Item that must have methods (Float being a template parameter):
//  Float lon()
//  Float lat()
//  Box<Float> bounds()
//  std::string repr() (for debugging)
//

namespace quadtree {

// borrowed from flat_earth_math.h, but we don't want to depend on it for this standalone quadtree implementation

// return relative angle in (-180, 180]
static inline double RA(double angle) {
    // optimize for small angles, which are more common in our use case
    while (angle > 180.0)
        angle -= 360.0;

    while (angle <= -180.0)
        angle += 360.0;

    return angle;
}

static inline float RA(float angle) {
    // optimize for small angles, which are more common in our use case
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle <= -180.0f)
        angle += 360.0f;

    return angle;
}

template <typename Float>
class Box {
    public:
    // it is assumed that the lon values are normalized
    Float min_lon_, min_lat_, max_lon_, max_lat_;

    bool Contains(Float lon, Float lat) const noexcept;
    bool Intersects(const Box& other) const noexcept;

    // this is inside other
    bool InsideOf(const Box& other) const noexcept {
        return other.Contains(min_lon_, min_lat_) && other.Contains(max_lon_, max_lat_);
    }

    Box() {};

    Box(Float min_lon, Float min_lat, Float max_lon, Float max_lat) noexcept
        : min_lon_(min_lon), min_lat_(min_lat), max_lon_(max_lon), max_lat_(max_lat) {}

    // Find items in a +-dist_meters box around ll point (= distance in infinity-norm)
    Box(Float center_lon, Float center_lat, Float dist_meters) noexcept;

    // return the 4 quadrants of this box, in order sw, se, nw, ne
    std::array<Box<Float>, 4> Subdivide() const noexcept;
};

template <typename Float, typename Item, int kMaxItem>
class LLQuadTreeNode;

template <typename Float, typename Item, int kMaxItem>
class LLQuadTree {
    std::unique_ptr<LLQuadTreeNode<Float, Item, kMaxItem>> root_;

   protected:
    friend class LLQuadTreeNode<Float, Item, kMaxItem>;
    int next_node_id_ = 0;

   public:
    LLQuadTree() = default;
    LLQuadTree(const LLQuadTree&) = delete;
    LLQuadTree& operator=(const LLQuadTree&) = delete;
    ~LLQuadTree() = default;

    bool empty() const { return root_ == nullptr; }
    size_t size() const { return root_ ? root_->n_below_ : 0; }
    void Insert(Item* item);
    int Find(Float lon, Float lat, std::array<Item*, kMaxItem>& items, int* depth = nullptr) const;

    // find all items in a box, e.g. jetways near to a stand
    std::unordered_map<Item*, bool> FindInBox(const Box<Float>& box) const;

    void Dump();    // for debugging
};

};  // namespace quadtree
