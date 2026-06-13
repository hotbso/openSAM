//
//    AutoDGS / openSAM: Marshaller
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

#include <cassert>
#include <cstring>

#include "XPLMInstance.h"

#include "log_msg.h"
#include "dgs_impl.h"

namespace dgs {

// A Marshaller implementation for a DGS
class Marshaller : public DGS {
    Mode mode_ = kIdle;
    const std::string name_;  // for logging only
    XPLMInstanceRef inst_ref_ {};
    float drefs_[DGS_DR_NUM] {};
    XPLMDrawInfo_t drawinfo_ {};
    float sin_wave_prev_ = 0.0;
    int lrprev_ = 0;
    int trackprev_ = 0;
    float distance_prev_ = 0.0;

   public:
    Marshaller(const std::string& name);
    ~Marshaller() override;
    bool HasEqStatus() const noexcept override { return false; }
    bool isVdgs() const noexcept override { return false; }

    void SetGuidanceParams(const GuidanceParams& params) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo) override;
    void SetMode(Mode mode) override;
};

static XPLMObjectRef obj_ref;

// external interface
bool InitMarshaller(const std::string& res_dir) {
    std::string obj_fn = res_dir + "Marshaller.obj";
    obj_ref = XPLMLoadObject(obj_fn.c_str());
    if (obj_ref == nullptr) {
        LogMsg("Can't load marshaller object from '%s'", obj_fn.c_str());
        return false;
    }

    LogMsg("Marshaller object loaded from '%s'", obj_fn.c_str());
    return true;
}

std::unique_ptr<DGS> CreateMarshaller(const std::string& name) {
    assert(obj_ref != nullptr);
    return std::make_unique<Marshaller>(name);
}

// Marshaller implementation
Marshaller::Marshaller(const std::string& name) : name_(name) {
    LogMsg("Creating Marshaller instance for stand '%s'", name_.c_str());
}

void Marshaller::SetMode(Mode mode) {
    if (mode_ == mode)
        return;

    mode_ = mode;

    if (mode_ == kArrival) {
        inst_ref_ = XPLMCreateInstance(obj_ref, dgs_dlist_dr);
        LogMsg("Marshaller mode set to ARRIVAL, instance created");
    } else {
        if (inst_ref_)
            XPLMDestroyInstance(inst_ref_);
        inst_ref_ = nullptr;
    }
}

Marshaller::~Marshaller() {
    if (inst_ref_)
        XPLMDestroyInstance(inst_ref_);
}

void Marshaller::SetGuidanceParams(const GuidanceParams& dgs_params) {
    if (inst_ref_ == nullptr)
        return;

    GuidanceParams params = dgs_params;  // make a copy to modify

    // For the Marshaller sync change of straight ahead / turn commands with arm position
    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    bool phase180 = (sin_wave_prev_ > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev_ = sin_wave;

    if (!phase180) {
        params.lr = lrprev_;

        if (params.track == 3 && trackprev_ == 2) {
            params.track = trackprev_;
            params.distance = distance_prev_;
        }
    }

    memset(drefs_, 0, sizeof(drefs_));
    drefs_[DGS_DR_STATUS] = params.status;
    drefs_[DGS_DR_DISTANCE] = params.distance;
    drefs_[DGS_DR_TRACK] = params.track;
    drefs_[DGS_DR_LR] = params.lr;
    XPLMInstanceSetPosition(inst_ref_, &drawinfo_, drefs_);
    // LogMsg("Marshaller params updated, status: %d, track: %d, lr: %d, distance: %0.1f", params.status, params.track,
    //        params.lr, params.distance);

    trackprev_ = params.track;
    lrprev_ = params.lr;
    distance_prev_ = params.distance;
}

void Marshaller::SetPos(const XPLMDrawInfo_t& drawinfo) {
    drawinfo_ = drawinfo;
    LogMsg("Marshaller position updated, x: %0.1f, y: %0.1f, z: %0.1f", drawinfo_.x, drawinfo_.y, drawinfo_.z);

    if (inst_ref_ == nullptr)
        return;

    XPLMInstanceSetPosition(inst_ref_, &drawinfo_, drefs_);
}

} // namespace dgs
