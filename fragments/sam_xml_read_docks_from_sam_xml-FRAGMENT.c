=============================
This is just a code fragment
kept in case is should be usefull later


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
struct _sam_dgs  {

    // local dgs x,z computed from the xml's lat/lon
    double dgs_x, dgs_y, dgs_z;
    double stand_x, stand_y, stand_z;

    unsigned int ref_gen;   // only valid if this matches the generation of the ref frame

    // this is are from sam.xml
    char id[40];

    float latitude, longitude, elevation, heading, dockLatitude, dockLongitude, dockHeading;

    //float bb_lat_min, bb_lat_max, bb_lon_min, bb_lon_max;   /* bounding box for FAR_SKIP */
};

-------------------------------------------------------------------------------------------
#undef GET_FLOAT_PROP
#undef GET_STR_PROP

#define GET_FLOAT_PROP(p) \
    sam_dgs->p = extract_float(line, #p);

#define GET_STR_PROP(p) \
    extract_str(line, #p, sam_dgs->p, sizeof(sam_dgs->p));

static void
get_dgs_props(const char *line, sam_dgs_t *sam_dgs)
{
    memset(sam_dgs, 0, sizeof(*sam_dgs));

    GET_STR_PROP(id)
    GET_FLOAT_PROP(latitude)
    GET_FLOAT_PROP(longitude)
    GET_FLOAT_PROP(heading)
    GET_FLOAT_PROP(elevation)
    GET_FLOAT_PROP(dockLatitude)
    GET_FLOAT_PROP(dockLongitude)
    GET_FLOAT_PROP(dockHeading)
}

static int
read_sam_xml(FILE *f, scenery_t *sc)
{
 ....

        } else if (strstr(line, "<dock ")) {
            //log_msg("%s", line);
            if (sc->n_sam_dgs == max_sam_dgs) {
                max_sam_dgs += 100;
                sc->sam_dgs = realloc(sc->sam_dgs, max_sam_dgs * sizeof(sam_dgs_t));
                if (sc->sam_dgs == NULL) {
                    log_msg("Can't allocate memory");
                    return 0;
                }
            }

            get_dgs_props(line, &sc->sam_dgs[sc->n_sam_dgs]);
            sc->n_sam_dgs++;

        }
    }

    sc->sam_jws = realloc(sc->sam_jws, sc->n_sam_jws * sizeof(sam_jw_t));   // shrink to actual
    sc->sam_dgs = realloc(sc->sam_dgs, sc->n_sam_dgs * sizeof(sam_dgs_t));
    return 1;
