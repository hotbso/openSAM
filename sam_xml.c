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

scenery_t *sceneries;
int n_sceneries;
static int max_sceneries;

static float
extract_float(const char *line, const char *prop) {
    const char *cptr;

    if (NULL == (cptr = strstr(line, prop)))
        return 0.0f;

    if (NULL == (cptr = strchr(cptr, '"')))
        return 0.0f;
    cptr++;

    float res = atof(cptr);
    //log_msg("%15s '%s' %8.3f", prop, cptr, res);
    return res;
}

static void
extract_str(const char *line, const char *prop, char *value, int value_len) {
    const char *cptr, *cptr1;

    value[0] = '\0';
    if (NULL == (cptr = strstr(line, prop)))
        return;

    if (NULL == (cptr = strchr(cptr, '"')))
        return;
    cptr++;

    if (NULL == (cptr1 = strchr(cptr, '"')))
        return;

    int len = cptr1 - cptr;
    if (len > value_len - 1)
        len = value_len - 1;
    strncpy(value, cptr, len);
    //printf("%15s %s\n", prop, value);
}

#define GET_FLOAT_PROP(p) \
    sam_jw->p = extract_float(line, #p);

#define GET_STR_PROP(p) \
    extract_str(line, #p, sam_jw->p, sizeof(sam_jw->p));

static void
get_sam_props(const char *line, sam_jw_t *sam_jw)
{
    memset(sam_jw, 0, sizeof(*sam_jw));

    GET_STR_PROP(name)
    GET_FLOAT_PROP(latitude)
    GET_FLOAT_PROP(longitude)
    GET_FLOAT_PROP(heading)
    GET_FLOAT_PROP(height)
    GET_FLOAT_PROP(wheelPos)
    GET_FLOAT_PROP(cabinPos)
    GET_FLOAT_PROP(cabinLength)
    GET_FLOAT_PROP(wheelDiameter)
    GET_FLOAT_PROP(wheelDistance)
    GET_STR_PROP(sound)
    GET_FLOAT_PROP(minRot1)
    GET_FLOAT_PROP(maxRot1)
    GET_FLOAT_PROP(minRot2)
    GET_FLOAT_PROP(maxRot2)
    GET_FLOAT_PROP(minRot3)
    GET_FLOAT_PROP(maxRot3)
    GET_FLOAT_PROP(minExtent)
    GET_FLOAT_PROP(maxExtent)
    GET_FLOAT_PROP(minWheels)
    GET_FLOAT_PROP(maxWheels)
    GET_FLOAT_PROP(initialRot1)
    GET_FLOAT_PROP(initialRot2)
    GET_FLOAT_PROP(initialRot3)
    GET_FLOAT_PROP(initialExtent)

    char buffer[10];
    extract_str(line, "forDoorLocation", buffer, sizeof(buffer));
    if (0 == strcmp(buffer, "LF2")) {
        sam_jw->door = 1;
    }
}

static int
read_sam_xml(FILE *f, scenery_t *sc)
{
    char line[2000];    // can be quite long

    memset(sc, 0, sizeof(scenery_t));
    int max_sam_jws = 0;

    while (fgets(line, sizeof(line) - 1, f)) {
        char *cptr = strstr(line, "<jetway ");
        if (NULL == cptr)
            continue;

        if ((cptr = strchr(line, '\r')))
            *cptr = '\0';

        //log_msg("%s", line);
        if (sc->n_sam_jws == max_sam_jws) {
            max_sam_jws += 100;
            sc->sam_jws = realloc(sc->sam_jws, max_sam_jws * sizeof(sam_jw_t));
            if (sc->sam_jws == NULL) {
                log_msg("Can't allocate memory");
                return 0;
            }
        }

        get_sam_props(line, &sc->sam_jws[sc->n_sam_jws]);
        sc->n_sam_jws++;
    }

    sc->sam_jws = realloc(sc->sam_jws, sc->n_sam_jws * sizeof(sam_jw_t));   /* shrink to actual */
    // TODO: compute the NE, SW values
    return 1;
}

int
collect_sam_xml(const char *xp_dir)
{
    char line[1000];
    line[sizeof(line) - 1] = '\0';
    strncpy(line, xp_dir, sizeof(line) - 200);
    strcat(line, "/Custom Scenery/scenery_packs.ini");

    FILE *scp = fopen(line, "r");
    if (NULL == scp) {
        log_msg("Can't open '%s'", line);
        return 0;
    }

    while (fgets(line, sizeof(line) - 100, scp)) {
        char *cptr = strchr(line, '\r');
        if (cptr)
            *cptr = '\0';

        cptr = strchr(line, '\n');
        if (cptr)
            *cptr = '\0';

        cptr = strstr(line, "SCENERY_PACK ");
        if (NULL == cptr)
            continue;
        char *scenery_path = cptr + 13;
        int is_absolute = (scenery_path[0] == '/' || strchr(scenery_path, ':'));

        char fn[2000];
        fn[0] = '\0';

        if (is_absolute) {
            strncpy(fn, scenery_path, sizeof(fn) - 100);
        } else {
            strncpy(fn, xp_dir, sizeof(fn) - 100);
            strcat(fn, "/");
            strncat(fn, scenery_path, sizeof(fn) - 100);
        }

        int path_len = strlen(fn);

        strcat(fn, "sam.xml");

        //log_msg("Trying '%s'", fn);
        FILE *f = fopen(fn, "r");
        if (f) {
            log_msg("Processing '%s'", fn);

            if (n_sceneries == max_sceneries) {
                max_sceneries += 100;
                sceneries = realloc(sceneries, max_sceneries * sizeof(scenery_t));
                if (sceneries == NULL) {
                    log_msg("Can't allocate memory");
                    fclose(scp); fclose(f);
                    return 0;
                }
            }

            scenery_t *sc = &sceneries[n_sceneries];
            int rc = read_sam_xml(f, sc);
            fclose(f);
            if (!rc)
                return 0;

            if (path_len > 0)
                fn[path_len - 1] = '\0';    /* strip /sam */
            cptr = strrchr(fn, '/');
            char * cptr1 = strrchr(fn, '\\');
            if (cptr < cptr1)
                sc->name = strdup(cptr1 + 1);
            else if (cptr1 < cptr)
                sc->name = strdup(cptr + 1);

            if (sc->name == NULL)
                sc->name = "unknown";

            n_sceneries++;
        }

    }

    fclose(scp);
    sceneries = realloc(sceneries, n_sceneries * sizeof(scenery_t));

    static const float far_skip_dlat = FAR_SKIP / LAT_2_M;

    /* compute the bounding boxes */
    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {
        sc->bb_lat_min = sc->bb_lon_min = 1000.0f;
        sc->bb_lat_max = sc->bb_lon_max = -1000.0f;

        for (sam_jw_t *jw = sc->sam_jws; jw < sc->sam_jws + sc->n_sam_jws; jw++) {
            jw->bb_lat_min = jw->latitude - far_skip_dlat;
            jw->bb_lat_max = jw->latitude + far_skip_dlat;

            float far_skip_dlon = far_skip_dlat / cosf(jw->latitude * D2R);
            jw->bb_lon_min = RA(jw->longitude - far_skip_dlon);
            jw->bb_lon_max = RA(jw->longitude + far_skip_dlon);

            sc->bb_lat_min = MIN(sc->bb_lat_min, jw->bb_lat_min);
            sc->bb_lat_max = MAX(sc->bb_lat_max, jw->bb_lat_max);

            sc->bb_lon_min = MIN(sc->bb_lon_min, jw->bb_lon_min);
            sc->bb_lon_max = MAX(sc->bb_lon_max, jw->bb_lon_max);
        }
    }

    return 1;
}

