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

#include "openSAM.h"
#include "os_jw.h"
#include "os_jw_impl.h"

sound_t alert;

int
sound_init()
{
    return 1;
};

static void
alert_complete(void *ref, FMOD_RESULT status)
{
    UNUSED(status);

    jw_ctx_t *ajw = ref;
    ajw->alert_chn = NULL;
}

void
alert_on(jw_ctx_t *ajw)
{
    if (ajw->alert_chn)
        return;
    ajw->alert_chn = XPLMPlayPCMOnBus(alert.data, alert.size, FMOD_SOUND_FORMAT_PCM16,
                                      alert.sample_rate, alert.num_channels, 1,
                                      xplm_AudioExteriorUnprocessed,
                                      alert_complete, ajw);

    alert_setpos(ajw);
    XPLMSetAudioFadeDistance(ajw->alert_chn, 20.0f, 150.0f );
    XPLMSetAudioVolume(ajw->alert_chn, 1.3f);
}

void
alert_off(jw_ctx_t *ajw)
{
    if (ajw->alert_chn)
        XPLMStopAudio(ajw->alert_chn);
    ajw->alert_chn = NULL;
}

void
alert_setpos(jw_ctx_t *ajw)
{
    const sam_jw_t *jw = ajw->jw;

    static FMOD_VECTOR vel = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR pos;

    float rot1 = RA((jw->rotate1 + jw->psi) - 90.0f);
    pos.x = jw->x + (jw->extent + jw->cabinPos) * cosf(rot1 * D2R);
    pos.y = jw->y + jw->height;
    pos.z = jw->z + (jw->extent + jw->cabinPos) * sinf(rot1 * D2R);
    XPLMSetAudioPosition(ajw->alert_chn, &pos, &vel);
}
