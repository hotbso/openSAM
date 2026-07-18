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

#include <cassert>
#include <cmath>
#include <functional>

#include "quadtree.h"
#include "log_msg.h"

namespace quadtree {
template <typename Float>
static constexpr Float kLat2M = 111120;  // 1° lat in m

template <typename Float>
static constexpr Float kD2R = std::numbers::pi_v<Float> / static_cast<Float>(180);

template <typename Float>
Box<Float>::Box(Float center_lon, Float center_lat, Float dist_meters) noexcept {
    center_lon = RA(center_lon);
    center_lat = std::clamp(center_lat, static_cast<Float>(-89), static_cast<Float>(89));
    Float height_deg = dist_meters / kLat2M<Float>;  // convert height from meters to degrees
    Float width_deg = height_deg / std::cos(center_lat * kD2R<Float>);
    min_lon_ = center_lon - width_deg;
    max_lon_ = center_lon + width_deg;
    min_lat_ = center_lat - height_deg;
    max_lat_ = center_lat + height_deg;
}

template <typename Float>
bool Box<Float>::Contains(Float lon, Float lat) const noexcept {
    // just clamp lat
    lat = std::clamp(lat, static_cast<Float>(-89), static_cast<Float>(89));

    if (!(min_lat_ <= lat && lat <= max_lat_))
        return false;

    lon = RA(lon);  // normalize lon
    if (min_lon_ < max_lon_) {
        return (min_lon_ < lon && lon <= max_lon_);
    } else {
        // Handle the case where the bounds cross the antimeridian
        return (lon > min_lon_ || lon <= max_lon_);
    }
}

template <typename Float>
bool Box<Float>::Intersects(const Box& other) const noexcept {
    if (max_lat_ < other.min_lat_ || min_lat_ > other.max_lat_)
        return false;

    if (min_lon_ < max_lon_) {
        if (other.min_lon_ < other.max_lon_) {
            return !(max_lon_ < other.min_lon_ || min_lon_ > other.max_lon_);
        } else {
            return !(max_lon_ < other.min_lon_ && min_lon_ > other.max_lon_);
        }
    } else {
        if (other.min_lon_ < other.max_lon_) {
            return !(max_lon_ < other.min_lon_ && min_lon_ > other.max_lon_);
        } else {
            return true;  // Both bounds cross the antimeridian, so they must intersect
        }
    }
}

// return the 4 quadrants of this box, in order sw, se, nw, ne
template <typename Float>
std::array<Box<Float>, 4> Box<Float>::Subdivide() const noexcept {
    Float mid_lon = (min_lon_ + max_lon_) / 2;
    Float mid_lat = (min_lat_ + max_lat_) / 2;

    return {{{min_lon_, min_lat_, mid_lon, mid_lat},
             {mid_lon, min_lat_, max_lon_, mid_lat},
             {min_lon_, mid_lat, mid_lon, max_lat_},
             {mid_lon, mid_lat, max_lon_, max_lat_}}};
}

template <typename Float, typename Item, int kMaxItem>
class LLQuadTreeNode {
    LLQuadTree<Float, Item, kMaxItem>& tree_;
    const int id_;

   protected:
    friend class LLQuadTree<Float, Item, kMaxItem>;
    Box<Float> bounds_;
    std::vector<Item*> items_;
    std::array<std::unique_ptr<LLQuadTreeNode<Float, Item, kMaxItem>>, 4> children_{};  // sw, se, nw, ne
    static constexpr std::array<const char*, 4> child_labels_{"sw", "se", "nw", "ne"};
    int n_below_ = 0;  // number of items in this node and all its children

    bool Verify(const char* label) const;

   public:
    LLQuadTreeNode(LLQuadTree<Float, Item, kMaxItem>& tree, int id, Box<Float> bounds, Item* item);
    void Insert(Item* item);
    void Dump(const char* label, int indent, bool no_recurse = false) const;
};

template <typename Float, typename Item, int kMaxItem>
LLQuadTreeNode<Float, Item, kMaxItem>::LLQuadTreeNode(LLQuadTree<Float, Item, kMaxItem>& tree, int id,
                                                      Box<Float> bounds, Item* item)
    : tree_(tree), id_(id), bounds_(bounds) {
    assert(item != nullptr);
    [[maybe_unused]] auto item_bounds = item->bounds();
    assert(bounds_.Intersects(item_bounds));  // The item must intersect with the node's bounds
    items_.reserve(kMaxItem);
    items_.push_back(item);
    n_below_ = 1;
    assert(Verify("post constructor"));
}

template <typename Float, typename Item, int kMaxItem>
void LLQuadTreeNode<Float, Item, kMaxItem>::Insert(Item* new_item) {
    assert(Verify("pre insert"));
    assert(bounds_.Intersects(new_item->bounds()));  // The item must intersect with the node's bounds

    n_below_++;  // increment the count of items in this node and all its children
    std::array<Box<Float>, 4> quadrants = bounds_.Subdivide();

    // insert item into all quadrants it intersects with, creating child nodes if necessary
    auto InsertIntoQuadrants = [&](Item* item) {
        auto item_bounds = item->bounds();

        for (int i = 0; i < 4; i++) {
            if (quadrants[i].Intersects(item_bounds)) {
                if (!children_[i]) {
                    children_[i] = std::make_unique<LLQuadTreeNode<Float, Item, kMaxItem>>(
                        tree_, tree_.next_node_id_++, quadrants[i], item);
                } else
                    children_[i]->Insert(item);
            }
        }
    };

    // simple case: leaf and not full
    if (!items_.empty() && items_.size() < kMaxItem) {
        items_.push_back(new_item);
        assert(Verify("post insert contained"));
        return;
    }

    // must subdivide this node and push down the existing items if we are at capacity
    if (items_.size() == kMaxItem) {
        //LogMsg("Node %d at capacity with %d items, pushing down to quadrants", id_, (int)items_.size());
        // a full leaf node, push down the existing items
        assert(Verify("pre insert push down"));
        for (Item* it : items_)
            InsertIntoQuadrants(it);

        items_.clear();  // This node is no longer a leaf
        items_.shrink_to_fit();
        assert(Verify("post insert push down"));
    }

    // Insert the new item into the appropriate quadrant
    InsertIntoQuadrants(new_item);
    assert(Verify("post insert"));
}

template <typename Float, typename Item, int kMaxItem>
void LLQuadTreeNode<Float, Item, kMaxItem>::Dump(const char* label, int indent, bool no_recurse) const {
    Float dlat = bounds_.max_lat_ - bounds_.min_lat_;
    Float dlon = bounds_.max_lon_ - bounds_.min_lon_;
    Float dlat_m = dlat * kLat2M<Float>;
    Float dlon_m = dlon * kLat2M<Float> * std::cos((bounds_.min_lat_ + bounds_.max_lat_) / 2 * kD2R<Float>);
    LogMsg("%*sNode %d %s: lon (%.3f, %.3f] - lat (%.3f, %.3f], n_below %d, size (%.3f m, %.3f m)", indent, "", id_, label, bounds_.min_lon_,
           bounds_.max_lon_, bounds_.min_lat_, bounds_.max_lat_, n_below_, dlon_m, dlat_m);
    for (Item* item : items_) {
        Float item_lon = item->lon(), item_lat = item->lat();
        LogMsg("%*sItem at (%.7f, %.7f), '%s'", indent + 2, "", item_lon, item_lat, item->repr().c_str());
    }

    if (!no_recurse) {
        for (int i = 0; i < 4; i++) {
            if (children_[i])
                children_[i]->Dump(child_labels_[i], indent + 2);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (children_[i])
                LogMsg("%*sChild %s: node %d", indent + 2, "", child_labels_[i], children_[i]->id_);
        }
    }
}

template <typename Float, typename Item, int kMaxItem>
bool LLQuadTreeNode<Float, Item, kMaxItem>::Verify(const char* label) const {
    // Verify that all items in this node intersect with the node's bounds
    for (Item* item : items_) {
        if (!bounds_.Intersects(item->bounds())) {
            LogMsg("Item bounds do not intersect with node bounds for node %d", id_);
            LogMsg("Node bounds: lon (%.3f, %.3f] - lat (%.3f, %.3f]", bounds_.min_lon_, bounds_.max_lon_,
                   bounds_.min_lat_, bounds_.max_lat_);
            LogMsg("Item bounds: lon (%.3f, %.3f] - lat (%.3f, %.3f]", item->bounds().min_lon_, item->bounds().max_lon_,
                   item->bounds().min_lat_, item->bounds().max_lat_);
            goto fail;
        }
    }

    if (!items_.empty()) {
        if (children_[0] || children_[1] || children_[2] || children_[3]) {  // A leaf has no childs and vice versa
            LogMsg("Node %d has items but also has children", id_);
            LogMsg("Children: sw %d, se %d, nw %d, ne %d", children_[0] ? children_[0]->id_ : -1,
                   children_[1] ? children_[1]->id_ : -1, children_[2] ? children_[2]->id_ : -1,
                   children_[3] ? children_[3]->id_ : -1);
            goto fail;
        }
        return true;
    }

    return true;

fail:
    LogMsg("Verification failed for node %d '%s'", id_, label);
    tree_.Dump();
    return false;
}

template <typename Float, typename Item, int kMaxItem>
void LLQuadTree<Float, Item, kMaxItem>::Insert(Item* item) {
    if (root_ == nullptr) {
        root_ = std::make_unique<LLQuadTreeNode<Float, Item, kMaxItem>>(*this, next_node_id_++,
                                                                         Box<Float>{-180, -90, 180, 90}, item);
    } else
        root_->Insert(item);
}

template <typename Float, typename Item, int kMaxItem>
int LLQuadTree<Float, Item, kMaxItem>::Find(Float lon, Float lat, std::array<Item*, kMaxItem>& items,
                                                     int* depth) const {
    if (depth)
        *depth = 0;

    if (root_ == nullptr)
        return 0;

    int ret_items = 0;
    const LLQuadTreeNode<Float, Item, kMaxItem>* node = root_.get();
    while (node != nullptr) {
        // node.Dump("searching", 0, true);
        if (!node->bounds_.Contains(lon, lat))
            return 0;  // Out of bounds

        if (!node->items_.empty()) {
            assert(node->items_.size() <= kMaxItem);
            for (Item* item : node->items_) {
                Box<Float> item_bounds = item->bounds();
                if (!item_bounds.Contains(lon, lat))
                    continue;  // Item bounds do not contain the point
                items[ret_items++] = item;
            }
            return ret_items;  // Found items in a leaf node
        }

        Float mid_lon = (node->bounds_.min_lon_ + node->bounds_.max_lon_) / 2;
        Float mid_lat = (node->bounds_.min_lat_ + node->bounds_.max_lat_) / 2;

        if (lon < mid_lon && lat < mid_lat) {
            node = node->children_[0].get();  // sw
        } else if (lon >= mid_lon && lat < mid_lat) {
            node = node->children_[1].get();  // se
        } else if (lon < mid_lon && lat >= mid_lat) {
            node = node->children_[2].get();  // nw
        } else {
            node = node->children_[3].get();  // ne
        }

        if (depth)
            (*depth)++;
    }

    assert(ret_items == 0);
    return 0;
}

// find all items in a box, e.g. jetways near to a stand
template <typename Float, typename Item, int kMaxItem>
std::unordered_map<Item*, bool> LLQuadTree<Float, Item, kMaxItem>::FindInBox(const Box<Float>& box) const {
    std::unordered_map<Item*, bool>  found_items;
    found_items.reserve(50);

    using Node = LLQuadTreeNode<Float, Item, kMaxItem>;

    std::function<void(const Node*)> collect_items = [&collect_items, &found_items, &box](const Node* node) {
        if (!node->bounds_.Intersects(box))
            return;

        for (Item* item : node->items_)
            if (box.Contains(item->lon(), item->lat()))
                found_items[item] = true;  // mark item as found, that's auto dup removal

        // recurse
        for (const auto& child : node->children_) {
            if (child)
                collect_items(child.get());
        }
    };

    if (root_)
        collect_items(root_.get());

    return found_items;
}

template <typename Float, typename Item, int kMaxItem>
void LLQuadTree<Float, Item, kMaxItem>::Dump() {
    if (root_ == nullptr) {
        LogMsg("Empty QuadTree");
    } else {
        root_->Dump("root", 0);
        LogMsg("Total nodes: %u", next_node_id_);
    }
}

};  // namespace quadtree
