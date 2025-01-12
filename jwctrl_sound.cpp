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
#include "samjw.h"
#include "jwctrl.h"


bool
JwCtrl::sound_dev_init()
{
    return true;
};

static void
alert_complete(void *ref, FMOD_RESULT status)
{
    UNUSED(status);

    JwCtrl *ajw = (JwCtrl *)ref;
    ajw->alert_chn_ = NULL;
}

void
JwCtrl::alert_on()
{
    if (alert_chn_)
        return;
    alert_chn_ = XPLMPlayPCMOnBus(alert_.data, alert_.size, FMOD_SOUND_FORMAT_PCM16,
                                      alert_.sample_rate, alert_.num_channels, 1,
                                      xplm_AudioExteriorUnprocessed,
                                      alert_complete, this);

    alert_setpos();
    XPLMSetAudioFadeDistance(alert_chn_, 20.0f, 150.0f );
    XPLMSetAudioVolume(alert_chn_, 1.3f);
}

void
JwCtrl::alert_off()
{
    if (alert_chn_)
        XPLMStopAudio(alert_chn_);
    alert_chn_ = NULL;
}

void
JwCtrl::alert_setpos()
{
    static FMOD_VECTOR vel = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR pos;

    float rot1 = RA((jw_->rotate1 + jw_->psi) - 90.0f);
    pos.x = jw_->x + (jw_->extent + jw_->cabinPos) * cosf(rot1 * D2R);
    pos.y = jw_->y + jw_->height;
    pos.z = jw_->z + (jw_->extent + jw_->cabinPos) * sinf(rot1 * D2R);
    XPLMSetAudioPosition(alert_chn_, &pos, &vel);
}
