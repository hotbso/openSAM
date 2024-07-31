/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch
              (C) Jonathan Harris 2013

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

#include "openSAM.h"
#include "os_jw.h"
#include "os_jw_impl.h"

#if APL
#  include <OpenAL/al.h>
#  include <OpenAL/alc.h>
#else
#  include <AL/al.h>
#  include <AL/alc.h>
#endif

#include <XPLMCamera.h>

sound_t alert;

static ALuint snd_src;
static ALuint snd_buffer;
static const ALfloat zero[3];
static int paused;

static XPLMDataRef audio_dr, paused_dr, view_external_dr;

static const float GAIN_EXTERNAL = 1.0f;
static const float GAIN_INTERNAL = 0.5f;	/* Quieter in internal views */

#define CHECKERR(msg) \
    { ALuint e = alGetError(); \
     if (e != AL_NO_ERROR) { \
        {log_msg("%s: %d", msg, e); \
        return 0; } \
    } }

int
sound_init()
{
    if (NULL == alcGetCurrentContext()) {
        log_msg("cannot open XP11's openAL context");
        return 0;
    }

    audio_dr = XPLMFindDataRef("sim/operation/sound/sound_on");
    paused_dr = XPLMFindDataRef("sim/time/paused");
    view_external_dr = XPLMFindDataRef("sim/graphics/view/view_is_external");


    alGenSources(1, &snd_src);
    CHECKERR("can't create sound source");

    alGenBuffers(1, &snd_buffer);
    CHECKERR("can't generate sound buffer");

    alBufferData(snd_buffer, alert.num_channels == 2 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16,
                 alert.data, alert.size, alert.sample_rate);
    CHECKERR("alBufferData");

    alSourcei(snd_src, AL_BUFFER, snd_buffer);
    alSourcef(snd_src, AL_PITCH, 1.0f);
    alSourcef(snd_src, AL_GAIN, 1.0f);
    alSourcei(snd_src, AL_SOURCE_RELATIVE, AL_TRUE);	/* Because we're not allowed to manipulate the listener */
    alSourcei(snd_src, AL_LOOPING, AL_TRUE);
    alSourcefv(snd_src, AL_POSITION, zero);
    alSourcefv(snd_src, AL_VELOCITY, zero);
    if (alGetError()) {
        log_msg("sound init error");
        return 0;
    }

    return 1;
}
void
alert_on(jw_ctx_t *ajw)
{
    if (0 == snd_src)
        return;

    alert_setpos(ajw);
    alSourcePlay(snd_src);
}

void
alert_off(jw_ctx_t *ajw)
{
    UNUSED(ajw);
    if (0 == snd_src)
        return;

    alSourceStop(snd_src);
}

void
alert_setpos(jw_ctx_t *ajw)
{
    if (0 == snd_src)
        return;

    const sam_jw_t *jw = ajw->jw;

    // Pause sound while sim is paused
    if (XPLMGetDatai(paused_dr)) {
        if (!paused)
            alSourcePause(snd_src);
        paused = 1;
        return;
    }

    if (paused) {
        alSourcePlay(snd_src);
        paused = 0;
    }

    // Calculate relative location ignoring tilt
    XPLMCameraPosition_t camera;
    XPLMReadCameraPosition(&camera);
    camera.heading *= D2R;
    float cos_h = cosf(camera.heading);
    float sin_h = sinf(camera.heading);

    float rot1 = RA((jw->rotate1 + jw->psi) - 90.0f);
    float dx = jw->x + (jw->extent + jw->cabinPos) * cosf(rot1 * D2R) - camera.x;
    float dy = jw->y + jw->height - camera.y;
    float dz = jw->z + (jw->extent + jw->cabinPos) * sinf(rot1 * D2R) - camera.z;

    ALfloat snd_rel[3];
    snd_rel[0] =  cos_h * dx + sin_h * dz;
    snd_rel[2] = -sin_h * dx + cos_h * dz;
    snd_rel[1] = dy;

    //log_msg("snd_rel: %0.1f, %0.1f, %0.1f", snd_rel[0], snd_rel[1], snd_rel[2]);
    alSourcefv(snd_src, AL_POSITION, snd_rel);
    alSourcef(snd_src, AL_GAIN, XPLMGetDatai(view_external_dr) ? GAIN_EXTERNAL : GAIN_INTERNAL);
}
