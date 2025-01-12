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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "openSAM.h"
#include "jwctrl.h"

typedef struct WAV_FORMAT {
	short		format;
	short		num_channels;
	int			sample_rate;
	int			byte_rate;
	short		block_align;
	short		bits_per_sample;
} WAV_FORMAT;

typedef struct CHUNK_HEADER
{
	char id[4];
	unsigned int size;
} CHUNK_HEADER;

typedef struct WAV_HEADER
{
	CHUNK_HEADER riffheader;
	char wav[4];
	CHUNK_HEADER fmtheader;
	WAV_FORMAT fmt;
} WAV_HEADER;


//
// A super simple wav file reader. I don't know and don't care whether it reads any other
// file than the ones supplied here.
//
void
read_wav(const std::string& fname, Sound& sound)
{
    sound = {};

	FILE *f = fopen(fname.c_str(), "rb");
    if (NULL == f) {
        log_msg("can't open wav %s", fname.c_str());
        return;
    }

    WAV_HEADER header;
    CHUNK_HEADER chunk;

    if (1 != fread(&header, sizeof(header), 1, f)) {
        log_msg("can't read header");
        fclose(f);
        return;
    }

    if (memcmp(header.riffheader.id, "RIFF", 4)
        || memcmp(header.wav, "WAVE", 4)
        || memcmp(header.fmtheader.id, "fmt ", 4)
        || header.fmtheader.size < sizeof(WAV_FORMAT)) {
        log_msg("wav invalid header");
        fclose(f);
        return;
    }

    fseek(f, header.fmtheader.size - sizeof(WAV_FORMAT), SEEK_CUR);

    // loop over chunks and find first data chunk
    while(1 == fread(&chunk, sizeof(chunk), 1, f)) {
        printf("chunk '%s' %d\n", chunk.id, chunk.size);
        if (0 == memcmp(chunk.id, "data", 4)) {
            void *data = malloc(chunk.size);
            if (NULL == data) {
                log_msg("Can't malloc data for wav file");
                fclose(f);
                return;
            }

            if (1 != fread(data, chunk.size, 1, f)) {
                log_msg("error reading wav file");
                fclose(f);
                free(data);
                return;
            }

            fclose(f);

            sound.data = data;
            sound.size = chunk.size;
            sound.sample_rate = header.fmt.sample_rate;
            sound.num_channels = header.fmt.num_channels;
            return;
        } else
            fseek(f, chunk.size, SEEK_CUR); // skip over chunk
    }

    fclose (f);
	log_msg("can't find data chunk in %s", fname.c_str());
    return;
}
