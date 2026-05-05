//
//    AutoDGS / openSAM: Sam1Legacy
//
//    Copyright (C) 2026 Holger Teutsch
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

#ifndef _SAM1_DGS_H_
#define _SAM1_DGS_H_

#include <memory>
#include <string>

#include "dgs/dgs.h"

namespace dgs {

enum Sam1State { SAM1_TRACK = 1, SAM1_STOP_ZONE, SAM1_IDLE };
static constexpr float SAM1_LATERAL_OFF = 10.0f;  // switches off VDGS

// SAM1 legacy dataref values, filled by the SAM1LegacyDGS device
struct SAM1DRefs {
    bool is_active;
    float dgs_x, dgs_z;
    int status;
    float lateral;
    float longitudinal;
};

extern SAM1DRefs sam1_drefs;

extern bool InitSam1Legacy();
extern std::unique_ptr<DGS> CreateSam1Legacy(const std::string& name);

} // namespace dgs
#endif
