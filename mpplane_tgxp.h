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
#ifndef _MP_PLANE_TGXP_H_
#define _MP_PLANE_TGXP_H_

#include <memory>

class MpAdapter_tgxp : public MpAdapter {
    // for performance reasons we fetch the whole dref vectors into arrays
    int vector_size_{0}, byte_area_size_{0};

    std::unique_ptr<int[]> flight_phase_val_;
    std::unique_ptr<int[]> traffic_type_val_;
    std::unique_ptr<char[]> acf_type_val_, flight_id_val_;
    std::unique_ptr<float[]> x_val_, y_val_, z_val_, psi_val_;

    friend  MpAdapter* MpAdapter_factory();

  protected:
    static bool probe();        // probe whether xPilot is active
    MpAdapter_tgxp();

  public:
    ~MpAdapter_tgxp();
    float update() override;
};
#endif
