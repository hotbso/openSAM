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
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <expat.h>

#ifndef O_BINARY
#define O_BINARY 0
#endif

#include "openSAM.h"
#include "os_jw.h"
#include "os_dgs.h"
#include "os_anim.h"

// context for element handlers
typedef struct _expat_ctx {
    XML_Parser parser;
    bool in_jetways;
    bool in_sets;
    bool in_datarefs, in_dataref;
    bool in_objects;
    bool in_gui;

    scenery_t *sc;

    int max_sam_jws;
    int max_sam_objs;
    int max_sam_anims;
    int max_sam_drfs;

    sam_drf_t *cur_dataref;
} expat_ctx_t;

scenery_t *sceneries;
int n_sceneries;
static int max_sceneries;

sam_jw_t sam3_lib_jw[MAX_SAM3_LIB_JW + 1];

sam_drf_t *sam_drfs;
int n_sam_drfs;

static const int BUFSIZE = 4096;

static const char *
lookup_attr(const XML_Char **attr, const char *name) {
    for (int i = 0; attr[i]; i += 2)
        if (0 == strcmp(attr[i], name))
            return attr[i+1];

    return NULL;
}

#define GET_INT_ATTR(ptr, n) \
{ \
    const char *val = lookup_attr(attr, #n); \
    if (val) \
        ptr->n = atoi(val); \
}

#define GET_FLOAT_ATTR(ptr, n) \
{ \
    const char *val = lookup_attr(attr, #n); \
    if (val) \
        ptr->n = atof(val); \
}

#define GET_STR_ATTR(ptr, n) \
{ \
    const char *val = lookup_attr(attr, #n); \
    if (val) \
        strncpy(ptr->n, val, sizeof(ptr->n) - 1); \
}

#define GET_BOOL_ATTR(ptr, n) \
{ \
    const char *val = lookup_attr(attr, #n); \
    ptr->n = (val && (0 == strcmp(val, "true"))); \
}

static void
get_jw_attrs(const XML_Char **attr, sam_jw_t *sam_jw)
{
    *sam_jw = (sam_jw_t){0};

    GET_INT_ATTR(sam_jw, id)
    GET_STR_ATTR(sam_jw, name)
    GET_FLOAT_ATTR(sam_jw, latitude)
    GET_FLOAT_ATTR(sam_jw, longitude)
    GET_FLOAT_ATTR(sam_jw, heading)
    GET_FLOAT_ATTR(sam_jw, height)
    GET_FLOAT_ATTR(sam_jw, wheelPos)
    GET_FLOAT_ATTR(sam_jw, cabinPos)
    GET_FLOAT_ATTR(sam_jw, cabinLength)
    GET_FLOAT_ATTR(sam_jw, wheelDiameter)
    GET_FLOAT_ATTR(sam_jw, wheelDistance)
    GET_STR_ATTR(sam_jw, sound)
    GET_FLOAT_ATTR(sam_jw, minRot1)
    GET_FLOAT_ATTR(sam_jw, maxRot1)
    GET_FLOAT_ATTR(sam_jw, minRot2)
    GET_FLOAT_ATTR(sam_jw, maxRot2)
    GET_FLOAT_ATTR(sam_jw, minRot3)
    GET_FLOAT_ATTR(sam_jw, maxRot3)
    GET_FLOAT_ATTR(sam_jw, minExtent)
    GET_FLOAT_ATTR(sam_jw, maxExtent)
    GET_FLOAT_ATTR(sam_jw, minWheels)
    GET_FLOAT_ATTR(sam_jw, maxWheels)
    GET_FLOAT_ATTR(sam_jw, initialRot1)
    GET_FLOAT_ATTR(sam_jw, initialRot2)
    GET_FLOAT_ATTR(sam_jw, initialRot3)
    GET_FLOAT_ATTR(sam_jw, initialExtent)

    const char *val = lookup_attr(attr, "forDoorLocation");
    if (val) {
        if (0 == strcmp(val, "LF2"))
            sam_jw->door = 1;

        else if (0 == strcmp(val, "LU1"))
            sam_jw->door = 2;
    }
}

static int
lookup_drf(const char *name)
{
    for (int i = 0; i < n_sam_drfs; i++)
        if (0 == strcmp(sam_drfs[i].name, name))
            return i;

    return -1;
}

static int
lookup_obj(const scenery_t *sc, const char *id)
{
    for (int i = 0; i < sc->n_sam_objs; i++)
        if (0 == strcmp(sc->sam_objs[i].id, id))
            return i;

    return -1;
}

// expat's callbacks
static void XMLCALL
start_element(void *user_data, const XML_Char *name, const XML_Char **attr) {
    expat_ctx_t *ctx = user_data;

    if (0 == strcmp(name, "scenery")) {
        GET_STR_ATTR(ctx->sc, name);
        return;
    }

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
            if (sc->sam_jws == NULL)
                goto oom;
        }

        get_jw_attrs(attr, &sc->sam_jws[sc->n_sam_jws]);
        sc->n_sam_jws++;
        return;
    }

    if (ctx->in_sets && (0 == strcmp(name, "set"))) {
        sam_jw_t sam_jw;
        get_jw_attrs(attr, &sam_jw);
        if (!BETWEEN(sam_jw.id, 1, MAX_SAM3_LIB_JW)) {
            log_msg("invalid library jw '%s', %d", sam_jw.name, sam_jw.id);
            return;
        }

        sam3_lib_jw[sam_jw.id] = sam_jw;
        return;
    }

    ////////// datarefs ////////////
    if (0 == strcmp(name, "datarefs")) {
        ctx->in_datarefs = true;
        return;
    }

    if (ctx->in_datarefs && (0 == strcmp(name, "dataref"))) {
        ctx->in_dataref = true;
        ctx->cur_dataref = NULL;
        //for (int i = 0; attr[i]; i += 2)
        //    log_msg("dataref %s, %s", attr[i], attr[i+1]);

        if (n_sam_drfs == ctx->max_sam_drfs) {
            ctx->max_sam_drfs += 100;
            sam_drfs = realloc(sam_drfs, ctx->max_sam_drfs * sizeof(sam_drf_t));
            if (sam_drfs == NULL)
                goto oom;
        }

        ctx->cur_dataref = &sam_drfs[n_sam_drfs];
        *(ctx->cur_dataref) = (sam_drf_t){0};

        GET_STR_ATTR(ctx->cur_dataref, name);
        if (ctx->cur_dataref->name[0] == '\0') {
            log_msg("name attribute not found for dataref");
            ctx->cur_dataref = NULL;
        }

        if (lookup_drf(ctx->cur_dataref->name) >= 0) {
            log_msg("duplicate definition for dataref '%s', ingnored", ctx->cur_dataref->name);
            ctx->cur_dataref = NULL;
        }

        if (ctx->cur_dataref) {
            GET_BOOL_ATTR(ctx->cur_dataref, autoplay);
            GET_BOOL_ATTR(ctx->cur_dataref, randomize_phase);
            GET_BOOL_ATTR(ctx->cur_dataref, augment_wind_speed);
        }
        return;
    }

    if (ctx->in_dataref && ctx->cur_dataref && (0 == strcmp(name, "animation"))) {
        sam_drf_t *d = ctx->cur_dataref;
        if (d->n_tv == DRF_MAX_ANIM) {
            log_msg("animation table overflow for %s", d->name);
            return;
        }

        const char *attr_t = lookup_attr(attr, "t");
        const char *attr_v = lookup_attr(attr, "v");

        if (attr_t && attr_v) {
            float t = atof(attr_t);
            float v = atof(attr_v);

            if (d->n_tv > 0 && t == d->t[d->n_tv - 1]) // no double entries
                d->v[d->n_tv - 1] = v;
            else {
                int n = d->n_tv;
                d->t[n] = t;
                d->v[n] = v;
                // save a few cycles in the accessor
                d->s[n] = (v - d->v[n-1]) / (t - d->t[n-1]);
                d->n_tv++;
            }
        }

        return;
    }

    ////////// objects ////////////
    if (0 == strcmp(name, "objects")) {
        ctx->in_objects = true;
        return;
    }

    if (ctx->in_objects && (0 == strcmp(name, "instance"))) {
        scenery_t *sc = ctx->sc;

        if (sc->n_sam_objs == ctx->max_sam_objs) {
            ctx->max_sam_objs += 100;
            sc->sam_objs = realloc(sc->sam_objs, ctx->max_sam_objs * sizeof(sam_obj_t));
            if (sc->sam_objs == NULL)
                goto oom;
        }

        sam_obj_t *obj = &sc->sam_objs[sc->n_sam_objs];

        *obj = (sam_obj_t){0};
        GET_STR_ATTR(obj, id);
        GET_FLOAT_ATTR(obj, latitude);
        GET_FLOAT_ATTR(obj, longitude);
        GET_FLOAT_ATTR(obj, elevation);
        GET_FLOAT_ATTR(obj, heading);
        sc->n_sam_objs++;
        return;
    }

    ////////// animations ////////////
    if (0 == strcmp(name, "gui")) {
        ctx->in_gui = true;
        return;
    }

    if (ctx->in_gui && (0 == strcmp(name, "checkbox"))) {
        scenery_t *sc = ctx->sc;

        if (sc->n_sam_anims == ctx->max_sam_anims) {
            ctx->max_sam_anims += 100;
            sc->sam_anims = realloc(sc->sam_anims, ctx->max_sam_anims * sizeof(sam_anim_t));
            if (sc->sam_anims == NULL)
                goto oom;
        }

        sam_anim_t *anim = &sc->sam_anims[sc->n_sam_anims];

        *anim = (sam_anim_t){0};
        GET_STR_ATTR(anim, label);
        GET_STR_ATTR(anim, title);

        anim->obj_idx = anim->drf_idx = -1;

        const char *inst = lookup_attr(attr, "instance");
        if (inst)
            anim->obj_idx = lookup_obj(sc, inst);

        const char *name = lookup_attr(attr, "dataref");
        if (name)
            anim->drf_idx = lookup_drf(name);

        if (anim->obj_idx >= 0 && anim->drf_idx >= 0)
            sc->n_sam_anims++;
        else
            log_msg("dataref of object not found for checkbox entry");

        return;
    }

    return;

  oom:
    log_msg("Can't allocate memory");
    XML_StopParser(ctx->parser, XML_FALSE);
    return;
}

static void XMLCALL
end_element(void *user_data, const XML_Char *name) {
    expat_ctx_t *ctx = user_data;

    if (0 == strcmp(name, "jetways"))
        ctx->in_jetways = false;

    else if (0 == strcmp(name, "sets"))
        ctx->in_sets = false;

    else if (0 == strcmp(name, "datarefs"))
        ctx->in_datarefs = false;

    else if (0 == strcmp(name, "dataref")) {
        ctx->in_dataref = false;
        if (ctx->cur_dataref) {
            if (ctx->cur_dataref->n_tv >= 2)    // sanity check
                n_sam_drfs++;
            else
                log_msg("too few animation entries for %s", ctx->cur_dataref->name);
        }
    }

    else if (0 == strcmp(name, "objects"))
        ctx->in_objects = false;

    else if (0 == strcmp(name, "gui"))
        ctx->in_gui = false;
}

static int
parse_sam_xml(int fd, scenery_t *sc)
{
    *sc = (scenery_t){0};

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
            log_msg("error reading sam.xml: %s", strerror(errno));
            goto out;
        }

        if (XML_ParseBuffer(parser, len, len == 0) == XML_STATUS_ERROR) {
            log_msg("Parse error at line %lu: %s",
                    XML_GetCurrentLineNumber(parser),
                    XML_ErrorString(XML_GetErrorCode(parser)));
            goto out;
        }

        if (len == 0)
            break;
    }

    rc = 1;

  out:
    if (parser)
        XML_ParserFree(parser);
    parser = NULL;
    return rc;
}

// go through apt.dat and collect stand information from 1300 lines
static int
parse_apt_dat(FILE *f, scenery_t *sc)
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
            *stand = (stand_t){0};
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

    return 1;
}

// make a complete filename for ../sam.xml from a line of scenery_packs.ini
static int
mk_sam_fn(const char *xp_dir, char *line, char *fn, int fn_size)
{
    char *cptr = strchr(line, '\r');
    if (cptr)
        *cptr = '\0';

    cptr = strchr(line, '\n');
    if (cptr)
        *cptr = '\0';

    cptr = strstr(line, "SCENERY_PACK ");
    if (NULL == cptr)
        return 0;
    char *scenery_path = cptr + 13;
    int is_absolute = (scenery_path[0] == '/' || strchr(scenery_path, ':'));

    fn[0] = '\0';

    if (is_absolute) {
        strncpy(fn, scenery_path, fn_size - 100);
    } else {
        strncpy(fn, xp_dir, fn_size - 100);
        strcat(fn, "/");
        strncat(fn, scenery_path, fn_size - 100);
    }

    strcat(fn, "sam.xml");

    // posixify
    for (cptr = fn; *cptr; cptr++)
        if (*cptr == '\\')
            *cptr = '/';

    return 1;
}

#define REALLOC_CHECK(ptr, n, type) \
if (ptr) { \
    ptr = realloc(ptr, (n) * sizeof(type)); \
    if ((n) > 0 && NULL == ptr) { \
        log_msg("out of memory " #ptr); \
        return 0; \
    } \
}

// collect sam.xml from all sceneries
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

    max_sceneries = 100;
    sceneries = realloc(sceneries, max_sceneries * sizeof(scenery_t));
    if (sceneries == NULL) {
        log_msg("Can't allocate memory");
        fclose(scp);
        return 0;
    }

    // sam's default datarefs must be defined first, so we need 2 passes
    // openSAM_Library only
    while (fgets(line, sizeof(line) - 100, scp)) {
        char fn[2000];
        if (!mk_sam_fn(xp_dir, line, fn, sizeof(fn)))
            continue;

        if (NULL == strstr(fn, "/openSAM_Library/"))
            continue;

        //log_msg("Trying '%s'", fn);
        int fd = open(fn, O_RDONLY|O_BINARY);
        if (fd > 0) {
            log_msg("Processing '%s'", fn);

            scenery_t *sc = &sceneries[n_sceneries];
            int rc = parse_sam_xml(fd, sc);
            close(fd);
            if (!rc) {
                fclose(scp);
                return 0;
            }
            break;
        }
    }

    // second pass, everything but openSAM_Library
    rewind(scp);
    while (fgets(line, sizeof(line) - 100, scp)) {
        char fn[2000];
        if (!mk_sam_fn(xp_dir, line, fn, sizeof(fn)))
            continue;

        if (strstr(fn, "/openSAM_Library/"))
            continue;

        // autoortho pretends every file exists but
        // reads give errors
        if (strstr(fn, "/z_ao_"))
            continue;

        char *path_end = strrchr(fn, '/');

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

            int rc = parse_sam_xml(fd, sc);
            close(fd);
            if (rc) {
                // read stands from apt.dat
                strcpy(path_end, "/Earth nav data/apt.dat");

                //log_msg("Trying '%s'", fn);
                FILE *f = fopen(fn, "r");
                if (f) {
                    log_msg("Processing '%s'", fn);
                    rc = parse_apt_dat(f, sc);
                    fclose(f);
                    if (!rc) {
                        fclose(scp);
                        return 0;
                    }
                }

                // don't save empty sceneries
                if (sc->n_sam_jws > 0 || sc->n_stands > 0 || sc->n_sam_anims > 0)
                    n_sceneries++;
            }
        }

        strcpy(path_end, "/libraryjetways.xml");
        //log_msg("Trying '%s'", fn);
        fd = open(fn, O_RDONLY|O_BINARY);
        if (fd > 0) {
            log_msg("Processing '%s'", fn);
            scenery_t dummy;
            int rc = parse_sam_xml(fd, &dummy);
            close(fd);
            if (!rc)
                return 0;
        }
    }

    fclose(scp);

    REALLOC_CHECK(sceneries, n_sceneries, scenery_t);
    REALLOC_CHECK(sam_drfs, n_sam_drfs, sam_drf_t);

    static const float far_skip_dlat = FAR_SKIP / LAT_2_M;

    for (scenery_t *sc = sceneries; sc < sceneries + n_sceneries; sc++) {

        // shrink to actual

        REALLOC_CHECK(sc->sam_jws, sc->n_sam_jws, sam_jw_t);
        REALLOC_CHECK(sc->stands, sc->n_stands, stand_t);
        REALLOC_CHECK(sc->sam_anims, sc->n_sam_anims, sam_anim_t);
        REALLOC_CHECK(sc->sam_objs, sc->n_sam_objs, sam_obj_t);

        // compute the bounding boxes

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

        // don't consider objects as these may be far away (e.g. Aerosoft LSZH)
    }

    return 1;
}

