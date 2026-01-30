//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2025, 2026  Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#ifndef _MPADAPTER_H_
#define _MPADAPTER_H_

#include <unordered_map>
#include <memory>

#include "plane.h"

// Wrapper around the different plugins providing multiplayer planes
// xPilot, TGXP, liveTraffic, ...
class MpAdapter {
  protected:
    std::unordered_map<std::string, std::unique_ptr<Plane>> mp_planes_;
    MpAdapter() {};

  public:
    virtual ~MpAdapter();
    virtual const char* personality() const = 0;

    virtual float update() = 0;     // update status of MP planes
    float JwStateMachine();       // return delay to next call
};

// hopefully will detect which plugin is active and returns the appropriate service
extern std::unique_ptr<MpAdapter> MpAdapter_factory();  // no supported MP plugin -> nullptr
#endif
