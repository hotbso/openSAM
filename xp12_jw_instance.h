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

#pragma once

#include <string>
#include <string_view>

#include "XPLMInstance.h"
#include "XPLMScenery.h"

#include "samjw.h"
#include "log_msg.h"

// Augmentation for XP12 jetway instances
class XP12JwInstance : public SamJw {
    const std::string arpt_icao_;
    int style_code_, length_code_;

    bool altitude_is_estimate_{true};
    XPLMDrawInfo_t drawinfo_;
    XPLMInstanceRef instance_ref_{nullptr};

    // update drawinfo_ from global coords / ref_gen
    bool UpdateDrawinfo() noexcept;      // -> changed

   public:
    XP12JwInstance(const std::string_view arpt_icao, const std::string_view stand_name, double lat, double lon, float psi, int style,
                   int length_code, float initial_extent, float initial_rot2);
    virtual ~XP12JwInstance();    // jetways live forever

    std::string repr() const override { return arpt_icao_ + ": " + name_; }
    void CreateInstance() override;
    void RemoveInstance() override;
    void UpdateInstance() override;
    bool is_xp12_instanced_jw() const override { return true; }

    // Show all instances around a specific location, e.g. 3000 meters
    static void InstanceAround(float lat, float lon, float distance);

    // Remove all instances, e.g. in XPLMDisable
    static void RemoveAll();

    // Initialize and Finalize
    static void Initialize();
    static void Finalize();
};
