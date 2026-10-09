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

#include "sam1_dgs.h"

#include <cassert>

#include "dgs/dgs_impl.h"
#include "log_msg.h"

namespace dgs {

SAM1DRefs sam1_drefs;

// A Sam1Legacy implementation for a DGS
class Sam1Legacy : public DGS {
    Mode mode_ = kIdle;
    Mode prev_mode_ = kIdle;

    const std::string name_;  // for logging only
    XPLMDrawInfo_t drawinfo_{};

   public:
    Sam1Legacy(const std::string& name);
    ~Sam1Legacy() override;
    bool HasEqStatus() const noexcept override { return false; }
    bool isVdgs() const noexcept override { return false; }

    void SetGuidanceParams(const GuidanceParams& params) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo, float height) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo) override;
    void SetMode(Mode mode) override;
    void UpdateInstance() override {};  // no instance to update
};

// external interface
bool InitSam1Legacy() {
    return true;
}

std::unique_ptr<DGS> CreateSam1Legacy(const std::string& name) {
    return std::make_unique<Sam1Legacy>(name);
}

// Sam1Legacy implementation
Sam1Legacy::Sam1Legacy(const std::string& name) : name_(name) {
    LogMsg("Creating Sam1Legacy instance for stand '%s'", name_.c_str());
}

void Sam1Legacy::SetMode(Mode mode) {
    if (mode_ == mode)
        return;

    prev_mode_ = mode_;
    mode_ = mode;

    LogMsg("Sam1Legacy '%s' mode changed from %d to %d", name_.c_str(), (int)prev_mode_, (int)mode_);

    if (mode_ >= kArrival) {
        sam1_drefs.is_active = true;
        sam1_drefs.dgs_x = drawinfo_.x;
        sam1_drefs.dgs_z = drawinfo_.z;
    } else if (mode_ == kIdle && prev_mode_ >= kArrival) // the DGS was active and now it's idle
        sam1_drefs.is_active = false;   // deactivate
}

Sam1Legacy::~Sam1Legacy() {
    if (mode_ > kIdle)
        sam1_drefs.is_active = false;
}

void Sam1Legacy::SetGuidanceParams(const GuidanceParams& dgs_params) {
    // translate into compatible SAM1 values
    sam1_drefs.lateral = -dgs_params.ref_x;
    sam1_drefs.longitudinal = std::min(dgs_params.ref_z, 30.0f);

    if (dgs_params.status == 0 || dgs_params.status == kDgsGstIdentified) {
        sam1_drefs.status = SAM1_TRACK;
        if (sam1_drefs.longitudinal <= 0.0f) {
            sam1_drefs.status = SAM1_STOP_ZONE;
            sam1_drefs.lateral = 0.0f;
        }
    } else if (dgs_params.status == kDgsGstStop) {
        sam1_drefs.status = SAM1_STOP_ZONE;
        sam1_drefs.lateral = 0.0f;
    } else if (dgs_params.status == kDgsGstOk) {
        sam1_drefs.status = SAM1_STOP_ZONE;
        sam1_drefs.lateral = 0.0f;
        sam1_drefs.longitudinal = -0.1f;
    } else if (dgs_params.status == kDgsGstTooFar) {
        sam1_drefs.status = SAM1_TRACK;
    } else {
        sam1_drefs.status = SAM1_IDLE;
        sam1_drefs.lateral = SAM1_LATERAL_OFF;
        sam1_drefs.longitudinal = 0.0f;
    }

    static float log_ts;
    if (now > log_ts + 2.0f) {
        log_ts = now;
        LogMsg("SAM1, active %d, x: %0.1f, status: %d, lateral: %0.1f, longitudinal: %0.1f",
               sam1_drefs.is_active, sam1_drefs.dgs_x, (int)sam1_drefs.status, sam1_drefs.lateral, sam1_drefs.longitudinal);
    }
}

void Sam1Legacy::SetPos(const XPLMDrawInfo_t& drawinfo, [[maybe_unused]]float height) {
    drawinfo_ = drawinfo;
    LogMsg("SAM1 position updated, x: %0.1f, y: %0.1f, z: %0.1f", drawinfo_.x, drawinfo_.y, drawinfo_.z);
    if (mode_ >= kArrival) {
        sam1_drefs.dgs_x = drawinfo_.x;
        sam1_drefs.dgs_z = drawinfo_.z;
    }
}

void Sam1Legacy::SetPos(const XPLMDrawInfo_t& drawinfo) {
    SetPos(drawinfo, 0.0f);
}

}  // namespace dgs
