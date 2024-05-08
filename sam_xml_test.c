/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

*/

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "openSAM.h"
#include "os_dgs.h"
#include "os_jw.h"

int
main(int argc, char **argv) {

    if (!collect_sam_xml("E:/X-Plane-12-test")) {
        log_msg("Error reading sam.xml files");
        exit(2);
    }

    printf("%d sceneries collected\n", n_sceneries);

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        printf("%s\n%d jetways collected, bbox: %0.3f,%0.3f -> %0.3f, %0.3f\n", sc->name, sc->n_sam_jws,
               sc->bb_lat_min, sc->bb_lon_min, sc->bb_lat_max, sc->bb_lon_max);

        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            log_msg("%s %5.6f %5.6f door: %d", jw->name, jw->latitude, jw->longitude, jw->door);
        }
        puts("\n");
    }

    puts("Library jetways");
    for (int i = 0; i <= MAX_SAM3_LIB_JW; i++) {
        sam_jw_t *jw = &sam3_lib_jw[i];
        if (jw->id == 0)
            continue;
        log_msg("%d; %s height: %0.2f, cabinPos: %0.2f", jw->id, jw->name, jw->height, jw->cabinPos);
    }

    puts("Ramps");
    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        printf("%s\n", sc->name);
        for (stand_t *stand = sc->stands; stand < sc->stands + sc->n_stands; stand++) {
            log_msg("%-40s %5.6f, %5.6f %6.2f", stand->id,
                    stand->lat, stand->lon, stand->hdgt);
        }
        puts("\n");
    }

	return (1);
}
