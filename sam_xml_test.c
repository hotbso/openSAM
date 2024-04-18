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

int
main(int argc, char **argv) {

    if (!collect_sam_xml("null")) {
        log_msg("Error reading sam.xml files");
        exit(2);
    }

    printf("%d jetways collected\n", n_sam_jws);
    for (sam_jw_t *jw = sam_jws; jw < sam_jws + n_sam_jws; jw++) {
        log_msg("%s %5.6f %5.6f", jw->name, jw->latitude, jw->longitude);
    }
	return (1);
}
