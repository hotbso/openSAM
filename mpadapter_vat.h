//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
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

// Virtual Air Traffic Adapter for X-Plane

#pragma once

#include <memory>

#include "mpadapter.h"

class MpAdapter_vat : public MpAdapter {
    int json_buf_len_{0};
    std::unique_ptr<char[]> json_buf_;

    friend  std::unique_ptr<MpAdapter> MpAdapter_factory();

  protected:
    static bool probe();        // probe whether xPilot is active
    MpAdapter_vat();

  public:
    ~MpAdapter_vat();
    const char* personality() const override { return "VAT"; };
    float update() override;
};
