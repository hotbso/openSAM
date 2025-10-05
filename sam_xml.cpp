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
#include "samjw.h"
#include "os_dgs.h"
#include "os_anim.h"

// context for element handlers
struct ExpatCtx {
    XML_Parser parser;
    bool in_jetways;
    bool in_sets;
    bool in_datarefs, in_dataref;
    bool in_objects;
    bool in_gui;

    Scenery* sc;
    SamDrf *cur_dataref;
};

std::vector<Scenery *> sceneries;

std::vector<SamJw*> lib_jw;
int max_lib_jw_id;

std::vector<SamDrf*> sam_drfs;

static const int BUFSIZE = 4096;

static const char *
LookupAttr(const XML_Char **attr, const char *name) {
    for (int i = 0; attr[i]; i += 2)
        if (0 == strcmp(attr[i], name))
            return attr[i+1];

    return NULL;
}

#define GET_INT_ATTR(ptr, n) \
{ \
    const char *val = LookupAttr(attr, #n); \
    if (val) \
        ptr->n = atoi(val); \
}

#define GET_FLOAT_ATTR(ptr, n) \
{ \
    const char *val = LookupAttr(attr, #n); \
    if (val) \
        ptr->n = atof(val); \
}

#define GET_STR_ATTR(ptr, n) \
{ \
    const char *val = LookupAttr(attr, #n); \
    if (val) \
        strncpy(ptr->n, val, sizeof(ptr->n) - 1); \
}

#define GET_BOOL_ATTR(ptr, n) \
{ \
    const char *val = LookupAttr(attr, #n); \
    ptr->n = (val && (0 == strcmp(val, "true"))); \
}

static void
GetJwAttrs(const XML_Char **attr, SamJw *sam_jw)
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

    const char *val = LookupAttr(attr, "forDoorLocation");
    if (val) {
        if (0 == strcmp(val, "LF2"))
            sam_jw->door = 1;

        else if (0 == strcmp(val, "LU1"))
            sam_jw->door = 2;
    }
}

static int
LookupDrf(const char *name)
{
    for (unsigned int i = 0; i < sam_drfs.size(); i++)
        if (0 == strcmp(sam_drfs[i]->name, name))
            return i;

    return -1;
}

static int
LookupObj(const Scenery* sc, const char *id)
{
    for (unsigned int i = 0; i < sc->sam_objs.size(); i++)
        if (0 == strcmp(sc->sam_objs[i]->id, id))
            return i;

    return -1;
}

// expat's callbacks
static void XMLCALL
StartElement(void *user_data, const XML_Char *name, const XML_Char **attr) {
    ExpatCtx *ctx = (ExpatCtx *)user_data;
    Scenery* sc = ctx->sc;

    if (sc && (0 == strcmp(name, "scenery"))) {
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

    if (sc && ctx->in_jetways && (0 == strcmp(name, "jetway"))) {
        SamJw *jw = new SamJw();
        GetJwAttrs(attr, jw);
        // simple sanity check, e.g Aerosoft LEBL has bogus values
        if (BETWEEN(jw->latitude, -85.0f, 85.0f) && BETWEEN(jw->longitude, -180.0f, 180.0f))
            sc->sam_jws.push_back(jw);
        else {
            LogMsg("Jetway with invalid lat,lon: %0.6f, %0.6f ignored", jw->latitude, jw->longitude);
            delete(jw);
        }
        return;
    }

    if (ctx->in_sets && (0 == strcmp(name, "set"))) {
        SamJw *sam_jw = new SamJw;
        GetJwAttrs(attr, sam_jw);

        if (sam_jw->id >= (int)lib_jw.size())
            lib_jw.resize(sam_jw->id + 20);
        if (lib_jw[sam_jw->id])
            LogMsg("duplicate jetway id detected: %d", sam_jw->id);
        lib_jw[sam_jw->id] = sam_jw;
        max_lib_jw_id = std::max(max_lib_jw_id, sam_jw->id);
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
        //    LogMsg("dataref %s, %s", attr[i], attr[i+1]);

        auto drf = new SamDrf();
        ctx->cur_dataref = drf;

        GET_STR_ATTR(drf, name);
        if (drf->name[0] == '\0') {
            LogMsg("name attribute not found for dataref");
            ctx->cur_dataref = NULL;
            return;
        }

        if (LookupDrf(drf->name) >= 0) {
            LogMsg("duplicate definition for dataref '%s', ignored", drf->name);
            ctx->cur_dataref = NULL;
            return;
        }

        GET_BOOL_ATTR(drf, autoplay);
        GET_BOOL_ATTR(drf, randomize_phase);
        GET_BOOL_ATTR(drf, augment_wind_speed);
        drf->t.reserve(10);
        drf->v.reserve(10);
        drf->s.reserve(10);
        sam_drfs.push_back(drf);
        return;
    }

    if (ctx->in_dataref && ctx->cur_dataref && (0 == strcmp(name, "animation"))) {
        SamDrf *d = ctx->cur_dataref;
        const char *attr_t = LookupAttr(attr, "t");
        const char *attr_v = LookupAttr(attr, "v");

        if (attr_t && attr_v) {
            float t = atof(attr_t);
            float v = atof(attr_v);

            if (d->n_tv > 0 && t == d->t[d->n_tv - 1]) // no double entries
                d->v[d->n_tv - 1] = v;
            else {
                int n = d->n_tv;
                d->t.push_back(t);
                d->v.push_back(v);
                // save a few cycles in the accessor
                float s = n > 0 ? (v - d->v[n-1]) / (t - d->t[n-1]) : 0.0f;
                d->s.push_back(s);
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

    if (sc && ctx->in_objects && (0 == strcmp(name, "instance"))) {
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

    if (sc && ctx->in_gui && (0 == strcmp(name, "checkbox"))) {
        SamAnim *anim = new SamAnim();
        GET_STR_ATTR(anim, label);
        GET_STR_ATTR(anim, title);

        anim->obj_idx = anim->drf_idx = -1;

        const char *inst = LookupAttr(attr, "instance");
        if (inst)
            anim->obj_idx = LookupObj(sc, inst);

        const char *name = LookupAttr(attr, "dataref");
        if (name)
            anim->drf_idx = LookupDrf(name);

        if (anim->obj_idx >= 0 && anim->drf_idx >= 0)
            sc->sam_anims.push_back(anim);
        else {
            delete(anim);
            LogMsg("dataref of object not found for checkbox entry");
        }
        return;
    }

    return;
}

static void XMLCALL
EndElement(void *user_data, const XML_Char *name) {
    ExpatCtx *ctx = (ExpatCtx *)user_data;

    if (0 == strcmp(name, "jetways"))
        ctx->in_jetways = false;

    else if (0 == strcmp(name, "sets"))
        ctx->in_sets = false;

    else if (0 == strcmp(name, "datarefs"))
        ctx->in_datarefs = false;

    else if (0 == strcmp(name, "dataref")) {
        ctx->in_dataref = false;
        auto drf = ctx->cur_dataref;
        if (drf) {
            drf->t.shrink_to_fit();
            drf->v.shrink_to_fit();
            drf->s.shrink_to_fit();
            if (drf->n_tv < 2)    // sanity check
                LogMsg("too few animation entries for %s", drf->name);
        }
    }

    else if (0 == strcmp(name, "objects"))
        ctx->in_objects = false;

    else if (0 == strcmp(name, "gui"))
        ctx->in_gui = false;
}

static bool
ParseSamXml(const std::string& fn, Scenery* sc = nullptr)
{
    bool rc = false;
    int fd = open(fn.c_str(), O_RDONLY|O_BINARY);
    if (fd < 0)
        return 0;

    LogMsg("Processing '%s'", fn.c_str());
    XML_Parser parser = XML_ParserCreate(NULL);
    if (NULL == parser)
        goto out;

    ExpatCtx ctx;
    ctx = {};
    ctx.parser = parser;
    ctx.sc = sc;

    XML_SetUserData(parser, &ctx);
    XML_SetElementHandler(parser, StartElement, EndElement);

    for (;;) {
        void *buf = XML_GetBuffer(parser, BUFSIZE);
        int len = read(fd, buf, BUFSIZE);
        if (len < 0) {
            LogMsg("error reading sam.xml: %s", strerror(errno));
            goto out;
        }

        if (XML_ParseBuffer(parser, len, len == 0) == XML_STATUS_ERROR) {
            LogMsg("Parse error at line %lu: %s",
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
ParseAptDat(const std::string& fn, Scenery* sc)
{
    std::ifstream apt(fn);
    if (apt.fail())
        return false;

    LogMsg("Processing '%s'", fn.c_str());

    std::string line;
    line.reserve(2000);          // can be quite long

    while (std::getline(apt, line)) {
        // 1302 icao_code ENRM
        if (line.starts_with("1302 icao_code ")) {
            sc->arpt_icao = line.substr(15, 4);
            continue;
        }

        if (line.find("1300 ") == 0) {
            if (line.back() == '\r')
                line.pop_back();
            //LogMsg("%s", line);
            line.erase(0, 5);
            Stand *stand = new Stand();
            int len;
            int n = sscanf(line.c_str(), "%f %f %f %*s %*s %n",
                           &stand->lat, &stand->lon, &stand->hdgt, &len);
            if (3 == n) {
                strncpy(stand->id, line.c_str() + len, sizeof(stand->id) - 1);
                //LogMsg("%d %d, %f %f %f '%s'", n, len, stand->lat, stand->lon, stand->hdgt, stand->id);

                stand->hdgt = RA(stand->hdgt);
                stand->sin_hdgt = sinf(kD2R * stand->hdgt);
                stand->cos_hdgt = cosf(kD2R * stand->hdgt);
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
        LogMsg("Can't open '%s'", scpi_name.c_str());
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
        throw OsEx("openSAM_Library is not installed!");
}

// collect sam.xml from all sceneries
void
CollectSamXml(const SceneryPacks &scp)
{
    lib_jw.reserve(50);

    // drefs from openSAM_Library must come first
    if (scp.openSAM_Library_path.size() == 0 ||
        !ParseSamXml(scp.openSAM_Library_path + "sam.xml"))
        throw OsEx("openSAM_Library is not installed or inaccessible!");

    if (scp.SAM_Library_path.size() > 0) {
        if (!ParseSamXml(scp.SAM_Library_path + "libraryjetways.xml"))
            LogMsg("Warning: SAM_Library is installed but 'SAM_Library/libraryjetways.xml' could not be processed");
    }

    for (auto & sc_path : scp.sc_paths) {
        ParseSamXml(sc_path + "libraryjetways.xml");    // always try libraryjetways.xml

        Scenery* sc = new Scenery();
        if (!ParseSamXml(sc_path + "sam.xml", sc)) {
            delete(sc);
            continue;
        }

        // read stands from apt.dat
        ParseAptDat(sc_path + "Earth nav data/apt.dat", sc);

        // don't save empty sceneries
        if (sc->sam_jws.size() == 0 && sc->stands.size() == 0 && sc->sam_anims.size() == 0) {
            delete(sc);
            continue;
        }

        static constexpr float far_skip_dlat = kFarSkip / kLat2M;

        // shrink to actual
        sc->sam_jws.shrink_to_fit();
        sc->stands.shrink_to_fit();
        sc->sam_anims.shrink_to_fit();
        sc->sam_objs.shrink_to_fit();

        // compute the bounding boxes

        sc->bb_lat_min = sc->bb_lon_min = 1000.0f;
        sc->bb_lat_max = sc->bb_lon_max = -1000.0f;

        for (auto jw : sc->sam_jws) {
            float far_skip_dlon = far_skip_dlat / cosf(jw->latitude * kD2R);
            sc->bb_lat_min = std::min(sc->bb_lat_min, jw->latitude - far_skip_dlat);
            sc->bb_lat_max = std::max(sc->bb_lat_max, jw->latitude + far_skip_dlat);

            sc->bb_lon_min = std::min(sc->bb_lon_min, RA(jw->longitude - far_skip_dlon));
            sc->bb_lon_max = std::max(sc->bb_lon_max, RA(jw->longitude + far_skip_dlon));
        }

        for (auto stand : sc->stands) {
            float far_skip_dlon = far_skip_dlat / cosf(stand->lat * kD2R);

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
    lib_jw.resize(max_lib_jw_id + 1);
}

