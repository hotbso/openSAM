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

#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

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

    Scenery* sc;
    SamDrf *cur_dataref;
} expat_ctx_t;

std::vector<Scenery *> sceneries;

SamJw sam3_lib_jw[MAX_SAM3_LIB_JW + 1];

std::vector<SamDrf*> sam_drfs;

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
get_jw_attrs(const XML_Char **attr, SamJw *sam_jw)
{
    *sam_jw = (SamJw){};

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
    for (unsigned int i = 0; i < sam_drfs.size(); i++)
        if (0 == strcmp(sam_drfs[i]->name, name))
            return i;

    return -1;
}

static int
lookup_obj(const Scenery* sc, const char *id)
{
    for (unsigned int i = 0; i < sc->sam_objs.size(); i++)
        if (0 == strcmp(sc->sam_objs[i]->id, id))
            return i;

    return -1;
}

// expat's callbacks
static void XMLCALL
start_element(void *user_data, const XML_Char *name, const XML_Char **attr) {
    expat_ctx_t *ctx = (expat_ctx_t *)user_data;

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
        Scenery* sc = ctx->sc;
        SamJw *jw = new SamJw();
        get_jw_attrs(attr, jw);
        sc->sam_jws.push_back(jw);
        return;
    }

    if (ctx->in_sets && (0 == strcmp(name, "set"))) {
        SamJw sam_jw;
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
        //for (int i = 0; attr[i]; i += 2)
        //    log_msg("dataref %s, %s", attr[i], attr[i+1]);

        auto drf = new SamDrf();
        ctx->cur_dataref = drf;

        GET_STR_ATTR(drf, name);
        if (drf->name[0] == '\0') {
            log_msg("name attribute not found for dataref");
            ctx->cur_dataref = NULL;
            return;
        }

        if (lookup_drf(drf->name) >= 0) {
            log_msg("duplicate definition for dataref '%s', ingnored", drf->name);
            ctx->cur_dataref = NULL;
            return;
        }

        GET_BOOL_ATTR(drf, autoplay);
        GET_BOOL_ATTR(drf, randomize_phase);
        GET_BOOL_ATTR(drf, augment_wind_speed);
        sam_drfs.push_back(drf);
        return;
    }

    if (ctx->in_dataref && ctx->cur_dataref && (0 == strcmp(name, "animation"))) {
        SamDrf *d = ctx->cur_dataref;
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
        Scenery* sc = ctx->sc;

        SamObj *obj = new SamObj();
        GET_STR_ATTR(obj, id);
        GET_FLOAT_ATTR(obj, latitude);
        GET_FLOAT_ATTR(obj, longitude);
        GET_FLOAT_ATTR(obj, elevation);
        GET_FLOAT_ATTR(obj, heading);
        sc->sam_objs.push_back(obj);
        return;
    }

    ////////// animations ////////////
    if (0 == strcmp(name, "gui")) {
        ctx->in_gui = true;
        return;
    }

    if (ctx->in_gui && (0 == strcmp(name, "checkbox"))) {
        Scenery* sc = ctx->sc;

        SamAnim *anim = new SamAnim();
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
            sc->sam_anims.push_back(anim);
        else {
            delete(anim);
            log_msg("dataref of object not found for checkbox entry");
        }
        return;
    }

    return;
}

static void XMLCALL
end_element(void *user_data, const XML_Char *name) {
    expat_ctx_t *ctx = (expat_ctx_t *)user_data;

    if (0 == strcmp(name, "jetways"))
        ctx->in_jetways = false;

    else if (0 == strcmp(name, "sets"))
        ctx->in_sets = false;

    else if (0 == strcmp(name, "datarefs"))
        ctx->in_datarefs = false;

    else if (0 == strcmp(name, "dataref")) {
        ctx->in_dataref = false;
        if (ctx->cur_dataref && ctx->cur_dataref->n_tv < 2)    // sanity check
            log_msg("too few animation entries for %s", ctx->cur_dataref->name);
    }

    else if (0 == strcmp(name, "objects"))
        ctx->in_objects = false;

    else if (0 == strcmp(name, "gui"))
        ctx->in_gui = false;
}

static bool
parse_sam_xml(const std::string& fn, Scenery* sc)
{
    bool rc = false;
    int fd = open(fn.c_str(), O_RDONLY|O_BINARY);
    if (fd < 0)
        return 0;

    log_msg("Processing '%s'", fn.c_str());
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

    rc = true;

  out:
    close(fd);
    if (parser)
        XML_ParserFree(parser);
    return rc;
}

// go through apt.dat and collect stand information from 1300 lines
static bool
parse_apt_dat(const std::string& fn, Scenery* sc)
{
    std::ifstream apt(fn);
    if (apt.fail())
        return false;

    log_msg("Processing '%s'", fn.c_str());

    std::string line;
    line.reserve(2000);          // can be quite long

    while (std::getline(apt, line)) {
        size_t i = line.find('\r');
        if (i != std::string::npos)
            line.resize(i);

        if (line.find("1300 ") == 0) {
            //log_msg("%s", line);
            line.erase(0, 5);
            Stand *stand = new Stand();
            int len;
            int n = sscanf(line.c_str(), "%f %f %f %*s %*s %n",
                           &stand->lat, &stand->lon, &stand->hdgt, &len);
            if (3 == n) {
                strncpy(stand->id, line.c_str() + len, sizeof(stand->id) - 1);
                //log_msg("%d %d, %f %f %f '%s'", n, len, stand->lat, stand->lon, stand->hdgt, stand->id);

                stand->hdgt = RA(stand->hdgt);
                stand->sin_hdgt = sinf(D2R * stand->hdgt);
                stand->cos_hdgt = cosf(D2R * stand->hdgt);
                sc->stands.push_back(stand);
            } else {
                delete(stand);
            }
        }

    }

    apt.close();
    return true;
}

// SceneryPacks contructor
SceneryPacks::SceneryPacks(const std::string& xp_dir)
{
    std::string scpi_name(xp_dir + "/Custom Scenery/scenery_packs.ini");

    std::ifstream scpi(scpi_name);
    if (scpi.fail()) {
        log_msg("Can't open '%s'", scpi_name.c_str());
        return;
    }

    sc_paths.reserve(500);
    std::string line;

    while (std::getline(scpi, line)) {
        size_t i;
        if ((i = line.find('\r')) != std::string::npos)
            line.resize(i);

        if (line.find("SCENERY_PACK ") != 0 || line.find("*GLOBAL_AIRPORTS*") != std::string::npos)
            continue;

        // autoortho pretends every file exists but
        // reads give errors
        if (line.find("/z_ao_") != std::string::npos)
            continue;

        line.erase(0, 13);
        std::string sc_path;
        bool is_absolute = (line[0] == '/' || line.find(':') != std::string::npos);
        if (is_absolute)
            sc_path = line;
        else
            sc_path = xp_dir + "/" + line;

        // posixify
        for (unsigned i = 0; i < sc_path.size(); i++)
            if (sc_path[i] == '\\')
                sc_path[i] = '/';

        if (sc_path.find("/openSAM_Library/") != std::string::npos) {
            openSAM_Library_path = sc_path;
            continue;
        }

        if (sc_path.find("/SAM_Library/") != std::string::npos) {
            SAM_Library_path = sc_path;
            continue;
        }

        sc_paths.push_back(sc_path);
    }

    scpi.close();
    sc_paths.shrink_to_fit();
    if (openSAM_Library_path.size() == 0)
        throw OsEx("ERROR: openSAM_Library is not installed, bye!");
}

// collect sam.xml from all sceneries
void
collect_sam_xml(const SceneryPacks &scp)
{
    // drefs from openSAM_Library must come first
    Scenery dummy;
    if (scp.openSAM_Library_path.size() == 0 ||
        !parse_sam_xml(scp.openSAM_Library_path + "sam.xml", &dummy))
        throw OsEx("ERROR: openSAM_Library is not installed or inaccessible, bye!");

    if (scp.SAM_Library_path.size() > 0) {
        Scenery dummy;
        if (!parse_sam_xml(scp.SAM_Library_path + "libraryjetways.xml", &dummy))
            log_msg("Warning: SAM_Library is installed but 'SAM_Library/libraryjetways.xml' could not be processed");
    }

    for (auto sc_path : scp.sc_paths) {
        Scenery* sc = new Scenery();
        if (!parse_sam_xml(sc_path + "sam.xml", sc)) {
            delete(sc);
            continue;
        }

        // read stands from apt.dat
        parse_apt_dat(sc_path + "Earth nav data/apt.dat", sc);

        // don't save empty sceneries
        if (sc->sam_jws.size() == 0 && sc->stands.size() == 0 && sc->sam_anims.size() == 0) {
            delete(sc);
            continue;
        }

        static const float far_skip_dlat = FAR_SKIP / LAT_2_M;

        // shrink to actual
        sc->sam_jws.shrink_to_fit();
        sc->stands.shrink_to_fit();
        sc->sam_anims.shrink_to_fit();
        sc->sam_objs.shrink_to_fit();

        // compute the bounding boxes

        sc->bb_lat_min = sc->bb_lon_min = 1000.0f;
        sc->bb_lat_max = sc->bb_lon_max = -1000.0f;

        for (auto jw : sc->sam_jws) {
            jw->bb_lat_min = jw->latitude - far_skip_dlat;
            jw->bb_lat_max = jw->latitude + far_skip_dlat;

            float far_skip_dlon = far_skip_dlat / cosf(jw->latitude * D2R);
            jw->bb_lon_min = RA(jw->longitude - far_skip_dlon);
            jw->bb_lon_max = RA(jw->longitude + far_skip_dlon);

            sc->bb_lat_min = std::min(sc->bb_lat_min, jw->bb_lat_min);
            sc->bb_lat_max = std::max(sc->bb_lat_max, jw->bb_lat_max);

            sc->bb_lon_min = std::min(sc->bb_lon_min, jw->bb_lon_min);
            sc->bb_lon_max = std::max(sc->bb_lon_max, jw->bb_lon_max);
        }

        for (auto stand : sc->stands) {
            float far_skip_dlon = far_skip_dlat / cosf(stand->lat * D2R);

            sc->bb_lat_min = std::min(sc->bb_lat_min, stand->lat - far_skip_dlat);
            sc->bb_lat_max = std::max(sc->bb_lat_max, stand->lat + far_skip_dlat);

            sc->bb_lon_min = std::min(sc->bb_lon_min, stand->lon - far_skip_dlon);
            sc->bb_lon_max = std::max(sc->bb_lon_max, stand->lon + far_skip_dlon);
        }

        // don't consider objects as these may be far away (e.g. Aerosoft LSZH)

        sceneries.push_back(sc);
    }

    sceneries.shrink_to_fit();
    sam_drfs.shrink_to_fit();
}

