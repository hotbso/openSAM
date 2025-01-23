/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2025  Holger Teutsch

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

#include <cassert>
#include "openSAM.h"
#include "plane.h"

#include "mpplane_xpilot.h"

static MpAdapter *active_adapter;

MpAdapter::MpAdapter()
{
}

MpAdapter::~MpAdapter()
{
    mp_planes_.clear();
    active_adapter = nullptr;
}

MpAdapter *MpAdapter_factory()
{
    // ensure that we only have one active adapter
    assert(active_adapter == nullptr);

    if (MpAdapter_xPilot::probe())
        active_adapter = new MpAdapter_xPilot();

    return active_adapter;
}

float
MpAdapter::jw_state_machine() {
    float jw_loop_delay = 10.0;
    for (auto & p : mp_planes_)
        jw_loop_delay = std::min(p.second->jw_state_machine(), jw_loop_delay);
    return jw_loop_delay;
}
