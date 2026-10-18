//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2025, 2026  Holger Teutsch
//    Copyright (C) 2026  Skirmish Mode Games Inc (mpadapter_batc: the BeyondATC traffic adapter)
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

#pragma once

#include <memory>

#include "mpadapter.h"

// BeyondATC (https://www.beyondatc.net/) injects its AI traffic into X-Plane through its own
// plugin, which publishes the traffic in the "beyondatc/ai/*" datarefs (one slot per drawn
// aircraft) together with the jetway intent that BeyondATC decided for each aircraft:
//   0 = no request, 1 = parked at its stand, a jetway may dock, 2 = leaving, undock now.
// This adapter follows that intent: it creates a plane on 1, takes 2 as "beacon on" and
// deletes the plane when it undocked and the intent is 0 again, or when the aircraft is gone.
class MpAdapter_batc : public MpAdapter {
    // for performance reasons we fetch the whole dref vectors into arrays
    int vector_size_{0};

    std::unique_ptr<int[]> id_val_, on_ground_val_, jetway_val_;
    std::unique_ptr<char[]> acf_type_val_, flight_id_val_;
    std::unique_ptr<float[]> x_val_, y_val_, z_val_, psi_val_;

    friend std::unique_ptr<MpAdapter> MpAdapter_factory();

  protected:
    static bool probe();  // probe whether the BeyondATC plugin is active
    MpAdapter_batc();

  public:
    ~MpAdapter_batc();
    const char* personality() const override { return "BeyondATC"; };
    float update() override;
};
