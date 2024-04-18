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

sam_jw_t *sam_jws;
int n_sam_jws, max_sam_jws;


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
    value[len] = '\0';
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
}

static int
read_sam_xml(FILE *f)
{
    char line[2000];    // can be quite long

    while (fgets(line, sizeof(line) - 1, f)) {
        char *cptr = strstr(line, "<jetway ");
        if (NULL == cptr)
            continue;

        if ((cptr = strchr(line, '\r')))
            *cptr = '\0';

        //log_msg("%s", line);
        if (n_sam_jws == max_sam_jws) {
            max_sam_jws += 100;
            sam_jws = realloc(sam_jws, max_sam_jws * sizeof(sam_jw_t));
            if (sam_jws == NULL) {
                log_msg("Can't allocate memory");
                return 0;
            }
        }

        get_sam_props(line, &sam_jws[n_sam_jws]);
        n_sam_jws++;
    }

    return 1;
}

int
collect_sam_xml(const char *xp_dir)
{
    const char *fn = "E:/X-Plane-12-test/Custom Scenery/Captain7 - 29Palms - EDDN Nuremberg 2/sam.xml";
    log_msg("Trying '%s'", fn);
    FILE *f = fopen(fn, "r");
	if (f) {
        log_msg("Processing '%s'", fn);
        int rc = read_sam_xml(f);
        fclose(f);
        if (!rc)
            return 0;
	}

    return 1;
}

