//
//    openSAM: manage DGS and jetways for X Plane
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

#include "XPLMScenery.h"
#include "XPLMDataAccess.h"

struct Ofp;

namespace dgs {

enum EqStatusVal { kEqUnknown, kEqOff, kEqOn };

struct EqStatus {
    int chocks, gpu, pca, pbb; // EqStatus for chocks, gpu, pca, pbb
};

enum Mode { kIdle, kDeparture, kArrival, kParked, kDeboarding };

struct GuidanceParams;

// Abstract base class for Marshaller and VDGS
class DGS {
   public:
    virtual ~DGS() = 0;

    // mandatory overrides
    virtual void SetMode(Mode mode) = 0;
    virtual void SetPos(const XPLMDrawInfo_t& drawinfo) = 0;  // move to new position only
    virtual void SetGuidanceParams(const GuidanceParams& params) = 0;
    virtual bool HasEqStatus() const noexcept = 0;  // whether this DGS can show eq status (chocks, gpu, pca, pbb)
    virtual bool isVdgs() const noexcept = 0;  // whether this DGS is a VDGS (vs. Marshaller)

    // optional overrides
    virtual void SetPaxNo([[maybe_unused]] int pax_no) {};
    virtual void SetOfpData([[maybe_unused]] const Ofp& ofp) {};
    virtual float Tick() { return 5.0f; };  // for VDGS, update display (e.g. scroll text), update eq status, etc., return delay to next update
};

// maps datarefs
extern bool InitDGS(const std::string& res_dir);

// Create dgs instances, succeeds or throws
extern std::unique_ptr<DGS> CreateMarshaller(const std::string& name);
extern std::unique_ptr<DGS> CreateSafedock_T2_24( const std::string& name,const std::string& arpt_icao, float height, bool display_only = false);
extern std::unique_ptr<DGS> CreateSafedock_X(const std::string& name, const std::string& arpt_icao, float height, bool display_only = false);
}  // namespace dgs

// global stuff to be defined elsewhere, likely by the main plugin code, e.g. in opensam.cpp
// we include it here in order to trigger a compile error if the main plugin code defines it differently
extern float now;
extern XPLMDataRef acf_icao_dr, acf_cg_y_dr, acf_cg_z_dr, acf_gear_z_dr, total_running_time_sec_dr, eng_running_dr, beacon_dr, sin_wave_dr;
extern std::string base_dir;
extern dgs::EqStatusVal PbbEqStatus();
extern XPLMProbeInfo_t probeinfo;
extern XPLMProbeRef probe_ref;
