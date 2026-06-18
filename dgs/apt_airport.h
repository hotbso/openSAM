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

#ifndef _APT_AIRPORT_H_
#define _APT_AIRPORT_H_

#include <string>
#include <vector>
#include <unordered_map>

#include "flat_earth_math.h"
#include "quadtree.h"

extern void scenery_test();  // for testing purposes, to access private members

namespace dgs {

// The airport database as loaded at plugin start and then stays unmodified.
// Pointers to AptStand and AptAirport therefore never become dangling.
// Hence we use raw pointers here.
struct AptStand {
	std::string name;
	double lon, lat;
	float hdgt;
    bool has_xp12_jw{false};
};

// code 100 data
struct AptRunway {
    std::string name;
    flat_earth_math::LLPos end1, end2;
    flat_earth_math::Vec2 cl;   // center line unit vector, end1 -> end2
    double len;
    float width;
};

class AptAirport {
   protected:
    friend void ::scenery_test();  // for testing purposes, to access private members

    static constexpr unsigned int kMaxAirportsPerNode = 10;
    static std::unordered_map<std::string, AptAirport*> apt_airports_;
    static quadtree::LLQuadTree<double, AptAirport, kMaxAirportsPerNode> apt_quadtree_;  // for fast lookup by position

    bool has_twr_{false};
    bool ignore_{false};      // e.g. no_autodgs marker present

   public:
    bool is_opensam_{false};    // whether this is an openSAM airport
    bool has_xp12_jws_{false};  // whether this airport has XP12 jetways, from apt.dat
    std::vector<AptStand> stands_;
    std::vector<AptRunway> rwys_;

    flat_earth_math::LLPos bbox_min_, bbox_max_;  // bounding box of this airport
    std::string icao_;

    AptAirport(const AptAirport&) = delete;
    AptAirport& operator=(const AptAirport&) = delete;

    // parse apt.dat, enter data into apt_airports_, return pointer to (last) parsed airport or nullptr on error
    // if ignore is true, a dummy airport with that id will be added and marked as ignored
    // this will also shadow a global airport with the same id, so it won't be found by LocateAirport

    static AptAirport* ParseAptDat(const std::string& fn, bool ignore, bool is_opensam, bool filter_autodgs,
                                   int& total_stands);

    static int NumAirports() { return apt_airports_.size(); }
    static void LoadingFinished();  // call after all airports have been loaded to build the quadtree

    // lookup by name
    static const AptAirport* LookupAirport(const std::string& airport_id);
    // lookup by position
    static const AptAirport* LocateAirport(const flat_earth_math::LLPos& pos);

    AptAirport(const std::string& name) : icao_(name) {}
    void dump() const;
    void ComputeBBox();

    // for quadtree
    double lon() const { return (bbox_min_.lon + bbox_max_.lon) / 2; }  // center point
    double lat() const { return (bbox_min_.lat + bbox_max_.lat) / 2; }
    quadtree::Box<double> bounds() const { return {bbox_min_.lon, bbox_min_.lat, bbox_max_.lon, bbox_max_.lat}; }
    std::string repr() const { return icao_; }
};

} // namespace dgs

#endif
