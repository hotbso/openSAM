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
#ifndef _MPADAPTER_LT_H_
#define _MPADAPTER_LT_H_

#include <memory>
#include "mpadapter.h"
#include "LTAPI.h"

class MpAdapter_lt : public MpAdapter {
    LTAPIConnect lt_connect_;

    friend  std::unique_ptr<MpAdapter> MpAdapter_factory();

  protected:
    static bool probe();        // probe whether xPilot is active
    MpAdapter_lt();

  public:
    ~MpAdapter_lt();
    const char* personality() const override { return "liveTraffic"; };
    float update() override;
};
#endif
