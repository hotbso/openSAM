//
//    openSAM: manage DGS for X Plane
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

#pragma once

#include <string>
#include <memory>

#include "dyn_display.h"

namespace dgs {
namespace logo {

extern std::unique_ptr<DynDisplay::Image> airline_logo;

// is asynchronous, will return immediately, the logo will be available later
// resize it to width = height = wh, if the source is not square, reject it
extern void LoadAirlineLogo(const std::string& airline_icao, int wh);

//
// Check for download and activate the new cdm info
// return true if download is still in progress
// if the logo download succeeded airline_logo is != nullptr
extern bool CheckAsyncDownload();

} // namespace logo

} // namespace dgs
