//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
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

#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <cassert>

#include "scenery.h"
#include "samjw.h"
#include "os_anim.h"
#include "dgs/airport.h"

#include "log_msg.h"

#include "quadtree.h"
#include "quadtree.inl"

#include "flat_earth_math.h"
namespace fem = flat_earth_math;

const char *log_msg_prefix = "scenery_test: ";

std::string xp_dir{"E:/X-Plane-12-test"};
std::vector<SamDrf> SamDrf::sam_drfs;
unsigned long long stat_sc_last;
std::vector<SamJw*> sam_jw_list;

quadtree::LLQuadTree<double, SamJw, kMaxJwPerNode> jw_quadtree;
std::vector<SamLibJw*> lib_jw;

void scenery_test() {

    std::cout << "scenery_test starting\n";

    try {
        SceneryPacks scp(xp_dir);
        int max_sam_stands;
        Scenery::CollectSceneries(scp, max_sam_stands);
        LogMsg("%d sceneries with sam jetways found, max stands: %d", (int)Scenery::sceneries.size(), max_sam_stands);
        int n_stands;
        if (!dgs::AptAirport::ParseAptDat(xp_dir + "/Global Scenery/Global Airports/Earth nav data/apt.dat", false, false, true, n_stands)) {
             LogMsg("WARNING: global apt.dat could not be parsed, no DGS support!");
            return;
        } else {
            LogMsg("%d stands with DGS found in global apt.dat", n_stands);
        }
    } catch (const std::exception& ex) {
        LogMsg("fatal error: '%s', bye!", ex.what());
        return;   // bye
    }

    dgs::AptAirport::LoadingFinished();

    printf("\n%d sceneries collected\n", (int)Scenery::sceneries.size());

    printf("%d datarefs collected\n", (int)SamDrf::sam_drfs.size());

    for (int i = 0; i < (int)SamDrf::sam_drfs.size(); i++) {
        SamDrf& drf = SamDrf::sam_drfs[i];
        printf("%d: '%s', auto_play: %d, randomize_phase: %d, augment_wind_speed: %d\n",
               i, drf.name.c_str(), drf.autoplay, drf.randomize_phase, drf.augment_wind_speed);

        for (int j = 0; j < drf.n_tv; j++)
            printf("   t: %6.2f, v: %6.2f\n", drf.t[j], drf.v[j]);

        puts("");
    }

    for (const auto& sc : Scenery::sceneries) {
        puts("\nObjects");
        for (auto& obj : sc.sam_objs_)
            printf("'%s' %5.6f %5.6f %5.6f %5.6f\n", obj.id.c_str(), obj.latitude, obj.longitude,
                   obj.elevation, obj.heading);

        puts("\nAnimations");
        for (auto& anim : sc.sam_anims_)
            printf("'%s' '%s', obj: '%s', drf: '%s'\n", anim.label.c_str(), anim.title.c_str(),
                   sc.sam_objs_[anim.obj_idx].id.c_str(), SamDrf::sam_drfs[anim.drf_idx].name.c_str());

        puts("\nJetways");
        for (auto i = sc.jw_idx_start_; i < sc.jw_idx_end_; i++) {
            const SamJw* jw = sam_jw_list[i];
            printf("%s %5.6f %5.6f door: %d\n", jw->name.c_str(), jw->latitude, jw->longitude, jw->door);
        }
        puts("\n");
    }

    puts("Library jetways");
    for (unsigned int i = 1; i < lib_jw.size(); i++) {
        const SamLibJw *ljw = lib_jw[i];
        printf("'%s'; '%s', height: %0.2f, cabinPos: %0.2f\n", ljw->id.c_str(), ljw->name.c_str(), ljw->height, ljw->cabinPos);
    }

    printf("\napt.dat collected: %d\n\n", dgs::AptAirport::NumAirports());

    const auto eddf = dgs::AptAirport::LookupAirport("EDDF");
    if (eddf) {
        eddf->dump();
    }

    const auto ekbi = dgs::AptAirport::LookupAirport("EKBI");
    if (ekbi) {
        ekbi->dump();
    }

    double min_lat = 90.0, max_lat = -90.0, min_lon = 360.0, max_lon = -360.0;
    for (const auto& [s, a] : dgs::AptAirport::apt_airports_) {
        min_lat = std::min(min_lat, a->bbox_min_.lat);
        max_lat = std::max(max_lat, a->bbox_max_.lat);
        min_lon = std::min(min_lon, a->bbox_min_.lon);
        max_lon = std::max(max_lon, a->bbox_max_.lon);
    }

    printf("\nGlobal Bounding box: %0.3f,%0.3f -> %0.3f,%0.3f\n", min_lat, min_lon, max_lat, max_lon);

    // dgs::AptAirport::apt_quadtree_.Dump();

    const dgs::AptAirport* arpt = dgs::AptAirport::LocateAirport(fem::LLPos(50.032548, 8.514717));  // EDDF, slightly west inside bbox
    if (arpt)
        printf("\nEDDF found by position lookup: %s\n", arpt->icao_.c_str());
    else
        printf("\nEDDF not found by position lookup\n");

    jw_quadtree.Dump();

    printf("\n\nSearching in +-50m box around EDDM stand 251A (11.797650, 48.354206)\n");
    std::vector<SamJw*>found_items;
    quadtree::Box<double> search_box(11.797650, 48.354206, 50);  // 50 m box around the stand
    std::unordered_map<SamJw*, bool> found_map = jw_quadtree.FindInBox(search_box);
    if (found_map.empty())
        printf("\nNo jetways found by FindInBox\n");
    else {
        for (auto& [jw, _] : found_map)
            printf("Found jetway by FindInBox: '%s' at %0.6f, %0.6f\n", jw->name.c_str(), jw->latitude,
                   jw->longitude);
    }
}

int
main(int argc, char **argv) {
    scenery_test();
    return 0;
}
