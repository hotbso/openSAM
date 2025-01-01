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

    JwCtrl *ajw = (JwCtrl *)ref;
    ajw->alert_chn = NULL;
}

auto JwCtrl::alert_on() -> void
{
    if (alert_chn)
        return;
    alert_chn = XPLMPlayPCMOnBus(alert.data, alert.size, FMOD_SOUND_FORMAT_PCM16,
                                      alert.sample_rate, alert.num_channels, 1,
                                      xplm_AudioExteriorUnprocessed,
                                      alert_complete, this);

    alert_setpos();
    XPLMSetAudioFadeDistance(alert_chn, 20.0f, 150.0f );
    XPLMSetAudioVolume(alert_chn, 1.3f);
}

auto JwCtrl::alert_off() -> void
{
    if (alert_chn)
        XPLMStopAudio(alert_chn);
    alert_chn = NULL;
}

auto JwCtrl::alert_setpos() -> void
{
    static FMOD_VECTOR vel = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR pos;

    float rot1 = RA((jw->rotate1 + jw->psi) - 90.0f);
    pos.x = jw->x + (jw->extent + jw->cabinPos) * cosf(rot1 * D2R);
    pos.y = jw->y + jw->height;
    pos.z = jw->z + (jw->extent + jw->cabinPos) * sinf(rot1 * D2R);
    XPLMSetAudioPosition(alert_chn, &pos, &vel);
}
