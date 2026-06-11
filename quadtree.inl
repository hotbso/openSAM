//
//    AutoDGS / openSAM: ManageVDGS
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

#pragma once

#include <cassert>
#include "quadtree.h"
#include "log_msg.h"

namespace quadtree {

template <typename Float>
bool Box<Float>::Contains(Float lon, Float lat) const noexcept {
    // just clamp lat
    lat = std::clamp(lat, static_cast<Float>(-89), static_cast<Float>(89));

    if (!(min_lat <= lat && lat <= max_lat))
        return false;

    lon = RA(lon);  // normalize lon
    if (min_lon < max_lon) {
        return (min_lon < lon && lon <= max_lon);
    } else {
        // Handle the case where the bounds cross the antimeridian
        return (lon > min_lon || lon <= max_lon);
    }
}

template <typename Float>
bool Box<Float>::Intersects(const Box& other) const noexcept {
    if (max_lat < other.min_lat || min_lat > other.max_lat)
        return false;

    if (min_lon < max_lon) {
        if (other.min_lon < other.max_lon) {
            return !(max_lon < other.min_lon || min_lon > other.max_lon);
        } else {
            return !(max_lon < other.min_lon && min_lon > other.max_lon);
        }
    } else {
        if (other.min_lon < other.max_lon) {
            return !(max_lon < other.min_lon && min_lon > other.max_lon);
        } else {
            return true;  // Both bounds cross the antimeridian, so they must intersect
        }
    }
}

// return the 4 quadrants of this box, in order sw, se, nw, ne
template <typename Float>
std::array<Box<Float>, 4> Box<Float>::Subdivide() const noexcept {
    Float mid_lon = (min_lon + max_lon) / 2;
    Float mid_lat = (min_lat + max_lat) / 2;

    return {{{min_lon, min_lat, mid_lon, mid_lat},
             {mid_lon, min_lat, max_lon, mid_lat},
             {min_lon, mid_lat, mid_lon, max_lat},
             {mid_lon, mid_lat, max_lon, max_lat}}};
}


template <typename Float, typename Item, auto kMaxItem>
class LLQuadTreeNode {
    LLQuadTree<Float, Item, kMaxItem>& tree_;
    const unsigned int id_;

   protected:
    friend class LLQuadTree<Float, Item, kMaxItem>;
    Box<Float> bounds_;
    std::vector<Item*> items_;
    std::array<std::unique_ptr<LLQuadTreeNode<Float, Item, kMaxItem>>, 4> children_{};  // sw, se, nw, ne
    static constexpr std::array<const char*, 4> child_labels_{"sw", "se", "nw", "ne"};
    unsigned int n_below_ = 0;  // number of items in this node and all its children

    bool Verify(const char* label) const;

   public:
    LLQuadTreeNode(LLQuadTree<Float, Item, kMaxItem>& tree, unsigned int id, Box<Float> bounds, Item* item);
    void Insert(Item* item);
    void Dump(const char* label, int indent, bool no_recurse = false) const;
};

template <typename Float, typename Item, auto kMaxItem>
LLQuadTreeNode<Float, Item, kMaxItem>::LLQuadTreeNode(LLQuadTree<Float, Item, kMaxItem>& tree, unsigned int id,
                                                      Box<Float> bounds, Item* item)
    : tree_(tree), id_(id), bounds_(bounds) {
    assert(item != nullptr);
    auto item_bounds = item->bounds();
    assert(bounds_.Intersects(item_bounds));  // The item must intersect with the node's bounds
    items_.reserve(kMaxItem);
    items_.push_back(item);
    n_below_ = 1;
    assert(Verify("post constructor"));
}

template <typename Float, typename Item, auto kMaxItem>
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

    // simple case: the item can fit into this node without needing to subdivide
    bool item_is_fully_contained = new_item->bounds().InsideOf(bounds_);
    if (!items_.empty() && items_.size() < kMaxItem && item_is_fully_contained) {
        items_.push_back(new_item);
        assert(Verify("post insert contained"));
        return;
    }

    // must subdivide this node and push down the existing items if we are at capacity or if the new item cannot fit
    // into this node
    if (items_.size() == kMaxItem || (!items_.empty() && !item_is_fully_contained)) {
        // a full leaf node, push down the existing items
        assert(Verify("pre insert push down"));
        for (Item* it : items_)
            InsertIntoQuadrants(it);

        items_.clear();  // This node is no longer a leaf
        items_.shrink_to_fit();
        assert(Verify("post insert push down"));
    }

    // printf("id_ %d, item_: %p, sw_: %d, se_: %d, nw_: %d, ne_: %d\n", id_, item_, sw_, se_, nw_, ne_);

    // Insert the new item into the appropriate quadrant
    InsertIntoQuadrants(new_item);
    assert(Verify("post insert"));
}

template <typename Float, typename Item, auto kMaxItem>
void LLQuadTreeNode<Float, Item, kMaxItem>::Dump(const char* label, int indent, bool no_recurse) const {
    LogMsg("%*sNode %d %s: lon (%.2f, %.2f] - lat (%.2f, %.2f], n_below %d", indent, "", id_, label, bounds_.min_lon,
           bounds_.max_lon, bounds_.min_lat, bounds_.max_lat, n_below_);
    for (Item* item : items_) {
        Float item_lon = item->lon(), item_lat = item->lat();
        LogMsg("%*sItem at (%.2f, %.2f), '%s'", indent + 2, "", item_lon, item_lat, item->repr().c_str());
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

template <typename Float, typename Item, auto kMaxItem>
bool LLQuadTreeNode<Float, Item, kMaxItem>::Verify(const char* label) const {
    // Verify that all items in this node intersect with the node's bounds
    for (Item* item : items_) {
        if (!bounds_.Intersects(item->bounds())) {
            LogMsg("Item bounds do not intersect with node bounds for node %d", id_);
            LogMsg("Node bounds: lon (%.2f, %.2f] - lat (%.2f, %.2f]", bounds_.min_lon, bounds_.max_lon, bounds_.min_lat,
                   bounds_.max_lat);
            LogMsg("Item bounds: lon (%.2f, %.2f] - lat (%.2f, %.2f]", item->bounds().min_lon, item->bounds().max_lon, item->bounds().min_lat, item->bounds().max_lat);
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

template <typename Float, typename Item, auto kMaxItem>
void LLQuadTree<Float, Item, kMaxItem>::Insert(Item* item) {
    if (root_ == nullptr) {
        root_ = std::make_unique<LLQuadTreeNode<Float, Item, kMaxItem>>(*this, next_node_id_++,
                                                                         Box<Float>{-180, -90, 180, 90}, item);
    } else
        root_->Insert(item);
}

template <typename Float, typename Item, auto kMaxItem>
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

        Float mid_lon = (node->bounds_.min_lon + node->bounds_.max_lon) / 2;
        Float mid_lat = (node->bounds_.min_lat + node->bounds_.max_lat) / 2;

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

template <typename Float, typename Item, auto kMaxItem>
void LLQuadTree<Float, Item, kMaxItem>::Dump() {
    if (root_ == nullptr) {
        LogMsg("Empty QuadTree");
    } else {
        root_->Dump("root", 0);
        LogMsg("Total nodes: %u", next_node_id_);
    }
}

};  // namespace quadtree
