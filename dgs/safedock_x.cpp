//
//    AutoDGS / openSAM: Safedock-X VDGS
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
#include <cmath>
#include <cstring>
#include <print>

#include "XPLMInstance.h"

#include "simbrief.h"
#include "log_msg.h"
#include "dgs_impl.h"
#include "plane.h"

namespace dgs {

static const char* null_dlist[] = {nullptr};

static XPLMObjectRef box_obj, base_obj, display_obj, idle_display_obj;

//------------------------------------------------------------------------------------
class Safedock_X : public DGS {
    Mode mode_ = kIdle;

    const std::string name_;  // for logging only
    const std::string arpt_icao_;
    std::string display_name_;  // for use in the VDGS

    XPLMInstanceRef box_inst_ref_{}, pole_base_inst_ref_{},  // static model
        display_inst_ref_{};                                 // display

    float drefs_[DGS_DR_NUM]{};

    XPLMDrawInfo_t pb_drawinfo_{};  // for pole base, on ground
    float height_;
    XPLMDrawInfo_t drawinfo_{};     // height adjusted for box and display

    std::unique_ptr<ScrollTxt> scroll_txt_;
    int pax_no_ = 0;
    float parked_ts_ = 0.0;  // parked mode start time,

   public:
    Safedock_X(const std::string& name, const std::string& arpt_icao,  float height, bool display_only);
    ~Safedock_X() override;

    bool HasEqStatus() const noexcept override { return true; }
    bool isVdgs() const noexcept override { return true; }

    void SetGuidanceParams(const GuidanceParams& params) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo) override;
    void SetMode(Mode mode) override;
    void SetPaxNo(int pax_no) override;
    void SetOfpData(const Ofp& ofp) override;

    float Tick() override;

    void UpdateInstance();  // update instance position and drefs
};

// external interface
bool InitSafedock_X(const std::string& res_dir) {
    std::string obj_fn = res_dir + "Safedock-X-display.obj";
    display_obj = XPLMLoadObject(obj_fn.c_str());
    if (display_obj == nullptr) {
        LogMsg("Can't load Safedock_X display object from '%s'", obj_fn.c_str());
        return false;
    }

    obj_fn = res_dir + "Safedock-X-idle.obj";
    idle_display_obj = XPLMLoadObject(obj_fn.c_str());
    if (idle_display_obj == nullptr) {
        LogMsg("Can't load Safedock_X idle object from '%s'", obj_fn.c_str());
        return false;
    }

    std::string on = res_dir + "pole_base.obj";
    base_obj = XPLMLoadObject(on.c_str());
    if (base_obj == nullptr) {
        LogMsg("Can't load pole base object from '%s'", on.c_str());
        return false;
    }

    on = res_dir + "Safedock-X-pole.obj";
    box_obj = XPLMLoadObject(on.c_str());
    if (box_obj == nullptr) {
        LogMsg("Can't load box object from '%s'", on.c_str());
        return false;
    }

    LogMsg("Safedock_X display objects loaded from '%s'", res_dir.c_str());
    return true;
}

std::unique_ptr<DGS> CreateSafedock_X(const std::string& name, const std::string& arpt_icao, float height, bool display_only) {
    assert(display_obj != nullptr);
    return std::make_unique<Safedock_X>(name, arpt_icao, height, display_only);
}

//------------------------------------------------------------------------------------
Safedock_X::Safedock_X(const std::string& name, const std::string& arpt_icao, float height, bool display_only)
    : name_(name), arpt_icao_(arpt_icao), height_(height) {
    LogMsg("Creating Safedock_X instance for stand '%s'", name_.c_str());
    // create display name
    // a stand name can be anything between "1" and "Gate A 40 (Class C, Terminal 3)"
    // we try to extract the net name "A 40" in the latter case

    if (name_.starts_with("Stand"))
        display_name_ = name_.substr(5);
    else if (name_.starts_with("Gate"))
        display_name_ = name_.substr(4);
    else if (name_.starts_with("Ramp"))
        display_name_ = name_.substr(4);
    else
        display_name_ = name_;

    // trim leading whitespace
    display_name_.erase(0, display_name_.find_first_not_of(" "));

    // delete stuff following and including a "({,;"
    if (display_name_.length() > kR1Nchar) {
        const auto i = display_name_.find_first_of("({,;");
        if (i != std::string::npos)
            display_name_.resize(i);
    }

    // trim trailing whitespace
    display_name_.erase(display_name_.find_last_not_of(" ") + 1);

    if (display_name_.length() > kR1Nchar)
        display_name_.clear();  // give up

    display_inst_ref_ = XPLMCreateInstance(display_obj, dgs_dlist_dr);
    if (display_inst_ref_ == nullptr) {
        LogMsg("Can't create instance for Safedock_X display");
        throw std::runtime_error("Failed to create instance for Safedock_X display");
    }

    if (!display_only) {
        box_inst_ref_ = XPLMCreateInstance(box_obj, null_dlist);
        assert(box_inst_ref_);
        pole_base_inst_ref_ = XPLMCreateInstance(base_obj, null_dlist);
    }

    LogMsg("Safedock_X instance created for stand '%s', display name: '%s'", name_.c_str(), display_name_.c_str());
}

Safedock_X::~Safedock_X() {
    if (display_inst_ref_)
        XPLMDestroyInstance(display_inst_ref_);
    if (box_inst_ref_)
        XPLMDestroyInstance(box_inst_ref_);
    if (pole_base_inst_ref_)
        XPLMDestroyInstance(pole_base_inst_ref_);
}

void Safedock_X::SetGuidanceParams(const GuidanceParams& params) {
    int d_0 = 0;
    int d_01 = 0;
    // according to Safegate_SDK_UG_Pilots_v1.10_s.pdf
    // > 3m: 1.0 m decrements, <= 3m 0.2m decrements
    if (0.0f <= params.distance && params.distance< 10.0f) {
        d_0 = params.distance;
        if (d_0 < 3) {
            int d = (params.distance - d_0) * 10.0f;
            d &= ~1;  // make it even = 0.2m increments
            d_01 = d;
        }
    }

    float distance = ((float)((int)((params.distance) * 2))) / 2;  // multiple of 0.5m

    // decide whether to show the SLOW indication
    // depends on distance and ground speed
    float gs = XPLMGetDataf(ground_speed_dr);
    bool slow = (params.track > 1) && ((params.distance > 20.0f && gs > 4.0f) ||
                                       (10.0f < params.distance && params.distance <= 20.0f && gs > 3.0f) ||
                                       (params.distance <= 10.0f && gs > 2.0f));

    memset(drefs_, 0, sizeof(drefs_));
    DGSFillUTCBrightness(drefs_);

    drefs_[DGS_DR_STATUS] = params.status;
    drefs_[DGS_DR_TRACK] = params.track;
    drefs_[DGS_DR_DISTANCE] = distance;
    drefs_[DGS_DR_DISTANCE_0] = d_0;
    drefs_[DGS_DR_DISTANCE_01] = d_01;
    drefs_[DGS_DR_XTRACK] = params.xtrack;
    drefs_[DGS_DR_LR] = params.lr;

    if (slow) {
        drefs_[DGS_DR_ICAO_0] = 'S';
        drefs_[DGS_DR_ICAO_1] = 'L';
        drefs_[DGS_DR_ICAO_2] = 'O';
        drefs_[DGS_DR_ICAO_3] = 'W';
    } else
        for (int i = 0; i < 4; i++)
            drefs_[DGS_DR_ICAO_0 + i] = (int)plane->acf_icao_[i];

    int blink = ((size_t)now) & 1;  // blink at 1Hz e.g. when left/right command is active
    drefs_[DGS_DR_BLINK] = blink;

    float move = std::fmod(0.6f * now, 1.0f);  // move from 0 to 1 every second e.g. for lead in indication
    drefs_[DGS_DR_MOVE] = move;

    if (params.status == kDgsGstOk) {
        EqStatus eq_status;
        plane->GetEqStatus(eq_status);
        drefs_[DGS_DR_CHK] = eq_status.chocks;
    }

    UpdateInstance();
}

void Safedock_X::SetPos(const XPLMDrawInfo_t& drawinfo) {
    pb_drawinfo_ = drawinfo;
    drawinfo_ = drawinfo;
    LogMsg("Safedock_X position updated for stand '%s', x: %0.1f, y: %0.1f, z: %0.1f", name_.c_str(), drawinfo_.x,
           drawinfo_.y, drawinfo_.z);
    drawinfo_.y += height_;
    UpdateInstance();
}

void Safedock_X::SetMode(Mode mode) {
    if (mode_ == mode)
        return;

    mode_ = mode;
    assert(display_inst_ref_ != nullptr);

    LogMsg("SetMode %d stand: '%s'", mode_, name_.c_str());

    parked_ts_ = 0.0f;  // reset parked mode start time

    if (mode_ == kIdle) {
        scroll_txt_ = nullptr;

        memset(drefs_, 0, sizeof(drefs_));
        DGSFillUTCBrightness(drefs_);

        int n = display_name_.length();
        for (int i = 0; i < n; i++)
            drefs_[DGS_DR_R1C0 + i] = display_name_[i];
        drefs_[DGS_DR_R1_SCROLL] = (5 * 16 - (n * 12 - 2)) / 2;  // center
        if (display_inst_ref_)
            XPLMDestroyInstance(display_inst_ref_);
        display_inst_ref_ = XPLMCreateInstance(idle_display_obj, dgs_dlist_dr);
    } else if (mode_ == kParked) {
        parked_ts_ = now;
        int zm = XPLMGetDatai(zulu_time_minutes_dr);
        int zh = XPLMGetDatai(zulu_time_hours_dr);
        scroll_txt_ =  std::make_unique<ScrollTxt>(std::format("{} AIBT {:02d}{:02d}   ", plane->callsign_, zh, zm));
    } else if (mode_ == kDeboarding) {
        // keep the scroll text from parked mode, but add "DEBOARDING" to it
    } else {  // departure or arrival
        if (display_name_.empty())
            scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + "   ");
        else
            scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + " STAND " + display_name_ + "   ");

        if (display_inst_ref_)
            XPLMDestroyInstance(display_inst_ref_);
        display_inst_ref_ = XPLMCreateInstance(display_obj, dgs_dlist_dr);
    }

    UpdateInstance();
}

void Safedock_X::SetPaxNo(int pax_no) {
    pax_no_ = pax_no;
}

void Safedock_X::SetOfpData(const Ofp& ofp) {
    std::string ofp_str = ofp.GenDepartureStr();
    LogMsg("SetOfpData for stand '%s', OFP departure str: '%s'", name_.c_str(), ofp_str.c_str());
    if (display_name_.empty())
        scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + "   " + ofp_str + "   ");
    else
        scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + " STAND " + display_name_ + "   " +
                                                    ofp_str + "   ");
}

float Safedock_X::Tick() {
    float delay = 1.0f;

    memset(drefs_, 0, sizeof(drefs_));
    DGSFillUTCBrightness(drefs_);

    if (scroll_txt_) {
        scroll_txt_->Tick(drefs_);
        delay = 0.05f;
    } else {
        int n = display_name_.length();
        for (int i = 0; i < n; i++)
            drefs_[DGS_DR_R1C0 + i] = display_name_[i];
        drefs_[DGS_DR_R1_SCROLL] = (5 * 16 - (n * 12 - 2)) / 2;  // center
    }

    bool show_eq_status_small = false;

    if (mode_ == kParked || mode_ == kDeparture || mode_ == kDeboarding) {
        //LogMsg("Tick for parked stand '%s', refreshing eq status", name_.c_str());

        EqStatus eq_status;
        plane->GetEqStatus(eq_status);
        if (now < parked_ts_ + 10.0f)
            drefs_[DGS_DR_STATUS_1] = 1;  // parked
        else {
            show_eq_status_small = true;
            drefs_[DGS_DR_STATUS_1] = 2;  // show eq status after 10s in parked mode
        }

        drefs_[DGS_DR_CHK] = eq_status.chocks;
        drefs_[DGS_DR_GPU] = eq_status.gpu;
        drefs_[DGS_DR_PCA] = eq_status.pca;
        drefs_[DGS_DR_PBB] = eq_status.pbb;
    }

    if (mode_ == kDeparture && pax_no_ > 0) {
        int pn[3]{-1, -1, -1};
        for (int i = 0; i < 3; i++) {
            pn[i] = pax_no_ % 10;
            pax_no_ /= 10;
            if (pax_no_ == 0)
                break;
        }
        drefs_[DGS_DR_BOARDING] = 1;
        for (int i = 0; i < 3; i++)
            drefs_[DGS_DR_PAXNO_0 + i] = pn[i];
    }

    if (mode_ == kDeboarding && show_eq_status_small && pax_no_ > 0) {
        int blink = ((size_t)now) & 1;  // blink at 1Hz
        drefs_[DGS_DR_DEBOARDING] = blink;
        delay = std::min(delay, 0.2f);
        //LogMsg("Tick for deboarding stand '%s', pax no: %d, dref: %f", name_.c_str(), pax_no_, drefs_[DGS_DR_DEBOARDING]);
    }

    UpdateInstance();
    return delay;
}

void Safedock_X::UpdateInstance() {
    XPLMInstanceSetPosition(display_inst_ref_, &drawinfo_, drefs_);

    if (box_inst_ref_)
        XPLMInstanceSetPosition(box_inst_ref_, &drawinfo_, nullptr);

    if (pole_base_inst_ref_)
        XPLMInstanceSetPosition(pole_base_inst_ref_, &pb_drawinfo_, nullptr);
}

} // namespace dgs
