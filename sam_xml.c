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
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <expat.h>

#ifndef O_BINARY
#define O_BINARY 0
#endif

#include "openSAM.h"
#include "os_jw.h"
#include "os_dgs.h"

// context for element handlers
typedef struct _expat_ctx {
    XML_Parser parser;
    bool in_jetways;
    bool in_sets;
    scenery_t *sc;
    int max_sam_jws;
} expat_ctx_t;

scenery_t *sceneries;
int n_sceneries;
static int max_sceneries;

sam_jw_t sam3_lib_jw[MAX_SAM3_LIB_JW + 1];

static const int BUFSIZE = 4096;

static int
extract_int(const XML_Char **attr, const char *prop) {
    for (int i = 0; attr[i]; i += 2)
        if (0 == strcmp(attr[i], prop))
            return atoi(attr[i+1]);
    return 0;
}

static float
extract_float(const XML_Char **attr, const char *prop) {
    for (int i = 0; attr[i]; i += 2)
        if (0 == strcmp(attr[i], prop))
            return atof(attr[i+1]);
    return 0;
}

static void
extract_str(const XML_Char **attr, const char *prop, char *value, int value_len) {
    value[0] = '\0';
    for (int i = 0; attr[i]; i += 2)
        if (0 == strcmp(attr[i], prop)) {
            int len = strlen(attr[i+1]);
            if (len > value_len - 1)
                len = value_len - 1;
            value[len] = '\0';
            strncpy(value, attr[i+1], len);
            return;
        }
}

#define GET_INT_PROP(p) \
    sam_jw->p = extract_int(attr, #p);

#define GET_FLOAT_PROP(p) \
    sam_jw->p = extract_float(attr, #p);

#define GET_STR_PROP(p) \
    extract_str(attr, #p, sam_jw->p, sizeof(sam_jw->p));

static void
get_jw_props(const XML_Char **attr, sam_jw_t *sam_jw)
{
    memset(sam_jw, 0, sizeof(*sam_jw));

    GET_INT_PROP(id)
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
    extract_str(attr, "forDoorLocation", buffer, sizeof(buffer));
    if (0 == strcmp(buffer, "LF2"))
        sam_jw->door = 1;

    if (0 == strcmp(buffer, "LU1"))
        sam_jw->door = 2;
}


static void XMLCALL
start_element(void *user_data, const XML_Char *name, const XML_Char **attr) {
    expat_ctx_t *ctx = user_data;

    if (0 == strcmp(name, "jetways")) {
        ctx->in_jetways = true;
        return;
    }

    if (0 == strcmp(name, "sets")) {
        ctx->in_sets = true;
        return;
    }

    if (ctx->in_jetways && (0 == strcmp(name, "jetway"))) {
        scenery_t *sc = ctx->sc;

        if (sc->n_sam_jws == ctx->max_sam_jws) {
            ctx->max_sam_jws += 100;
            sc->sam_jws = realloc(sc->sam_jws, ctx->max_sam_jws * sizeof(sam_jw_t));
            if (sc->sam_jws == NULL) {
                log_msg("Can't allocate memory");
                XML_StopParser(ctx->parser, XML_FALSE);
                return;
            }
        }

        get_jw_props(attr, &sc->sam_jws[sc->n_sam_jws]);
        sc->n_sam_jws++;
        return;
    }

    if (ctx->in_sets && (0 == strcmp(name, "set"))) {
        sam_jw_t sam_jw;

        get_jw_props(attr, &sam_jw);
        if (!BETWEEN(sam_jw.id, 1, MAX_SAM3_LIB_JW)) {
            log_msg("invalid library jw '%s', %d", sam_jw.name, sam_jw.id);
            return;
        }

        sam3_lib_jw[sam_jw.id] = sam_jw;
        return;
    }
}

static void XMLCALL
end_element(void *user_data, const XML_Char *name) {
    expat_ctx_t *ctx = user_data;

    if (0 == strcmp(name, "jetways")) {
        ctx->in_jetways = false;
        return;
    }

    if (0 == strcmp(name, "sets")) {
        ctx->in_sets = false;
        return;
    }
}

static int
read_sam_xml(int fd, scenery_t *sc)
{
    int rc = 0;
    XML_Parser parser = XML_ParserCreate(NULL);
    if (NULL == parser)
        goto out;

    expat_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.parser = parser;
    ctx.sc = sc;

    XML_SetUserData(parser, &ctx);
    XML_SetElementHandler(parser, start_element, end_element);

    for (;;) {
        void *buf = XML_GetBuffer(parser, BUFSIZE);
        int len = read(fd, buf, BUFSIZE);
        if (len < 0) {
            log_msg("error reading sam.xml");
            goto out;
        }

        if (XML_ParseBuffer(parser, len, len == 0) == XML_STATUS_ERROR) {
            log_msg("Parse error at line %u: %s",
                    XML_GetCurrentLineNumber(parser),
                    XML_ErrorString(XML_GetErrorCode(parser)));
            goto out;
        }

        if (len == 0)
            break;
    }

    if (sc)
        sc->sam_jws = realloc(sc->sam_jws, sc->n_sam_jws * sizeof(sam_jw_t));   // shrink to actual
    rc = 1;

  out:
    if (parser)
        XML_ParserFree(parser);
    parser = NULL;
    return rc;
}

static int
read_apt_dat(FILE *f, scenery_t *sc)
{
    char line[2000];    // can be quite long

    int max_stands = 0;

    while (fgets(line, sizeof(line) - 1, f)) {
        char *cptr = strchr(line, '\r');
        if (cptr)
            *cptr = '\0';

        cptr = strchr(line, '\n');
        if (cptr)
            *cptr = '\0';

        if (line == strstr(line, "1300 ")) {
            //log_msg("%s", line);
            if (sc->n_stands == max_stands) {
                max_stands += 100;
                sc->stands = realloc(sc->stands, max_stands * sizeof(stand_t));
                if (sc->stands == NULL) {
                    log_msg("Can't allocate memory");
                    return 0;
                }
            }
            stand_t *stand = &sc->stands[sc->n_stands];
            memset(stand, 0, sizeof(stand_t));
            int len;
            int n = sscanf(line + 5, "%f %f %f %*s %*s %n",
                           &stand->lat, &stand->lon, &stand->hdgt, &len);
            if (3 == n) {
                strncpy(stand->id, line + 5 + len, sizeof(stand->id) - 1);
                //log_msg("%d %d, %f %f %f '%s'", n, len, stand->lat, stand->lon, stand->hdgt, stand->id);

                stand->hdgt = RA(stand->hdgt);
                stand->sin_hdgt = sinf(D2R * stand->hdgt);
                stand->cos_hdgt = cosf(D2R * stand->hdgt);
                sc->n_stands++;
            }
        }

    }

    sc->stands = realloc(sc->stands, sc->n_stands * sizeof(stand_t));
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
        int fd = open(fn, O_RDONLY|O_BINARY);
        if (fd > 0) {
            log_msg("Processing '%s'", fn);

            if (n_sceneries == max_sceneries) {
                max_sceneries += 100;
                sceneries = realloc(sceneries, max_sceneries * sizeof(scenery_t));
                if (sceneries == NULL) {
                    log_msg("Can't allocate memory");
                    fclose(scp); close(fd);
                    return 0;
                }
            }

            scenery_t *sc = &sceneries[n_sceneries];
            memset(sc, 0, sizeof(scenery_t));

            int rc = read_sam_xml(fd, sc);
            close(fd);
            if (!rc) {
                fclose(scp);
                return 0;
            }

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

            // read stands from apt.dat
            fn[0] = '\0';
            if (is_absolute) {
                strncpy(fn, scenery_path, sizeof(fn) - 100);
            } else {
                strncpy(fn, xp_dir, sizeof(fn) - 100);
                strcat(fn, "/");
                strncat(fn, scenery_path, sizeof(fn) - 100);
            }

            strcat(fn, "Earth nav data/apt.dat");
            //log_msg("Trying '%s'", fn);
            FILE *f = fopen(fn, "r");
            if (f) {
                log_msg("Processing '%s'", fn);
                rc = read_apt_dat(f, sc);
                fclose(f);
                if (!rc) {
                    fclose(scp);
                    return 0;
                }
            }

            // don't save empty sceneries
            if (sc->n_sam_jws > 0 || sc->n_stands > 0)
                n_sceneries++;
        }

        fn[0] = '\0';
        if (is_absolute) {
            strncpy(fn, scenery_path, sizeof(fn) - 100);
        } else {
            strncpy(fn, xp_dir, sizeof(fn) - 100);
            strcat(fn, "/");
            strncat(fn, scenery_path, sizeof(fn) - 100);
        }

        strcat(fn, "libraryjetways.xml");
        //log_msg("Trying '%s'", fn);
        fd = open(fn, O_RDONLY|O_BINARY);
        if (fd > 0) {
            log_msg("Processing '%s'", fn);
            int rc = read_sam_xml(fd, NULL);
            close(fd);
            if (!rc)
                return 0;
            continue;
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

        for (stand_t *stand = sc->stands; stand < sc->stands + sc->n_stands; stand++) {
            float far_skip_dlon = far_skip_dlat / cosf(stand->lat * D2R);

            sc->bb_lat_min = MIN(sc->bb_lat_min, stand->lat - far_skip_dlat);
            sc->bb_lat_max = MAX(sc->bb_lat_max, stand->lat + far_skip_dlat);

            sc->bb_lon_min = MIN(sc->bb_lon_min, stand->lon - far_skip_dlon);
            sc->bb_lon_max = MAX(sc->bb_lon_max, stand->lon + far_skip_dlon);
        }
    }

    return 1;
}

