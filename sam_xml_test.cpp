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

#include "openSAM.h"
#include "os_dgs.h"
#include "samjw.h"
#include "os_anim.h"

const char *log_msg_prefix = "sam_xml_test: ";

std::string xp_dir{"E:/X-Plane-12-test"};

int
main(int argc, char **argv) {

    std::cout << "sam_xml_test starting\n";

    try {
        SceneryPacks scp(xp_dir);
        CollectSamXml(scp);
        LogMsg("%d sceneries with sam jetways found", (int)sceneries.size());
    } catch (const OsEx& ex) {
        LogMsg("fatal error: '%s', bye!", ex.what());
        return 0;   // bye
    }

    printf("\n%d sceneries collected\n", (int)sceneries.size());

    printf("%d datarefs collected\n", (int)sam_drfs.size());

    for (auto drf : sam_drfs) {
        printf("%s, auto_play: %d, randomize_phase: %d, augment_wind_speed: %d\n",
               drf->name.c_str(), drf->autoplay, drf->randomize_phase, drf->augment_wind_speed);

        for (int j = 0; j < drf->n_tv; j++)
            printf("   t: %6.2f, v: %6.2f\n", drf->t[j], drf->v[j]);

        puts("");
    }

    for (auto sc : sceneries) {
        printf("%s: %d jetways, %d stands collected, bbox: %0.3f,%0.3f -> %0.3f, %0.3f\n",
               sc->name.c_str(), (int)sc->sam_jws.size(), (int)sc->stands.size(),
               sc->bb_lat_min, sc->bb_lon_min, sc->bb_lat_max, sc->bb_lon_max);

        puts("\nObjects");
        for (auto obj : sc->sam_objs)
            printf("'%s' %5.6f %5.6f %5.6f %5.6f\n", obj->id.c_str(), obj->latitude, obj->longitude,
                   obj->elevation, obj->heading);

        puts("\nAnimations");
        for (auto anim : sc->sam_anims)
            printf("'%s' '%s', obj: '%s', drf: '%s'\n", anim->label.c_str(), anim->title.c_str(),
                   sc->sam_objs[anim->obj_idx]->id.c_str(), sam_drfs[anim->drf_idx]->name.c_str());

        puts("\nJetways");
        for (auto jw : sc->sam_jws) {
            printf("%s %5.6f %5.6f door: %d\n", jw->name.c_str(), jw->latitude, jw->longitude, jw->door);
        }
        puts("\n");
    }

    puts("Library jetways");
    for (unsigned int i = 1; i < lib_jw.size(); i++) {
        SamLibJw *ljw = lib_jw[i];
        printf("'%s'; '%s', height: %0.2f, cabinPos: %0.2f\n", ljw->id.c_str(), ljw->name.c_str(), ljw->height, ljw->cabinPos);
    }

    puts("\nRamps");
    for (auto sc : sceneries) {
        printf("%s\n", sc->name.c_str());
        for (auto stand : sc->stands) {
            printf("%-40s %5.6f, %5.6f %6.2f\n", stand->id.c_str(),
                    stand->lat, stand->lon, stand->hdgt);
        }
        puts("\n");
    }

	return (0);
}
