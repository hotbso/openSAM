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
#if 0
    sam_jw_t *jw = ajw->jw;

    static FMOD_VECTOR velocity = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR position = {jw->xml_x, jw->xml_y + 1.5f, jw->xml_z};

    XPLMSetAudioPosition(ajw->alert_chn, &position, &velocity);
#endif
}

void
alert_off(jw_ctx_t *ajw)
{
    if (ajw->alert_chn)
        XPLMStopAudio(ajw->alert_chn);
    ajw->alert_chn = NULL;
}
