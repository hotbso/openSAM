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

#include "xp12_jw_instance.h"
#include "log_msg.h"

// Constructor goes here in order to avoid linking XPLM into scenery_test.cpp

XP12JwInstance::XP12JwInstance(const std::string_view arpt_icao, const std::string_view stand_name, double lat, double lon, float psi, int style,
                               int length_code, float initial_extent, float initial_rot2)
    : arpt_icao_(arpt_icao) {

    base_name_ = stand_name;
    name_ = base_name_;
    style_code_ = style;
    length_code_ = length_code;

    // 'model parameters'
    height_ = 3.9f;
    wheel_pos_ = -3.0f;
    cabin_pos_ = 0.0f;
    cabin_length_ = 2.8f;
    wheel_diameter_ = 0.8f;
    wheel_distance_ = 1.8f;
    min_rot1_ = -120.0f;
    max_rot1_ = 90.0f;
    min_rot2_ = -90.0f;
    max_rot2_ = 70.0f;
    min_rot3_ = -10.0f;
    max_rot3_ = 10.0f;

    static constexpr float extra_extent = 0.0f;

    if (length_code == 0) {
        min_extent_ = 11.0f;
        max_extent_ = 23.0f + extra_extent;
    } else if (length_code == 1) {
        min_extent_ = 14.0f;
        max_extent_ = 29.0f + extra_extent;
    } else if (length_code == 2) {
        min_extent_ = 17.0f;
        max_extent_ = 38.0f + extra_extent;
    } else {
        if (length_code != 3)
            LogMsg("invalid length_code: %d, expected 1, 2, or 3, assuming 3", length_code);
        min_extent_ = 20.0f;
        max_extent_ = 47.0f + extra_extent;
    }

    // instance parameters
    latitude_ = lat;
    longitude_ = lon;
    heading_ = psi;
    psi_ = psi;
    min_rot1_ = -90.0f;
    max_rot1_ = 45.0f;
    // initial_extent in WED may be bogus
    initial_extent_ = std::clamp(initial_extent, min_extent_, max_extent_);
    initial_rot2_ = initial_rot2;
    initial_rot3_ = -2.0f;
    wheelrotatec_ = 0.5f * initial_rot2;
    drawinfo_.structSize = sizeof(XPLMDrawInfo_t);
    drawinfo_.pitch = 0.0f;
    drawinfo_.roll = 0.0f;
    drawinfo_.heading = psi;

    // LogMsg(
    //     "XP12JwInstance created with lat: %f, lon: %f, psi: %f, style: %d, length_code: %d, initial_extent: %f, "
    //     "intial_rot2: %f",
    //     latitude_, longitude_, heading_, style, length_code, this->initial_extent_, this->initial_rot2_);
}

XP12JwInstance::~XP12JwInstance() {
    // lives forever
}
