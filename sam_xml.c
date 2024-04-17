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
    //printf("%15s %8.3f\n", prop, res);
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

    const char * cptr;

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

int
main(int argc, char **argv) {

	char *docname = "sam.xml";
    FILE *f = fopen(docname, "r");
	if (f == NULL ) {
		fprintf(stderr,"Document not parsed successfully. \n");
		return 1;
	}


    char line[500];

    while (fgets(line, sizeof(line) - 1, f)) {
        char *cptr = strstr(line, "<jetway ");
        if (NULL == cptr)
            continue;
        int len = strlen(line);
        if (cptr = strchr(line, '\r'))
            *cptr = '\0';
        //printf("%s", line);
        if (n_sam_jws == max_sam_jws) {
            max_sam_jws += 100;
            sam_jws = realloc(sam_jws, max_sam_jws * sizeof(sam_jw_t));
            if (sam_jws == NULL) {
                fprintf(stderr, "Can't allocate memory.\n");
                exit(1);
            }
        }

        get_sam_props(line, &sam_jws[n_sam_jws]);
        n_sam_jws++;
    }

    fclose(f);
    for (int i = 0; i < n_sam_jws; i++) {
        sam_jw_t *sam_jw = &sam_jws[i];
        printf("%s %5.6f %5.6f\n", sam_jw->name, sam_jw->latitude, sam_jw->longitude);
    }
	return (1);
}
