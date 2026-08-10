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
#include <filesystem>
#include <vector>

#include "XPLMInstance.h"

#include "simbrief.h"
#include "log_msg.h"
#include "dgs_impl.h"
#include "plane.h"
#include "time_code_block.h"

#include "dyn_display.h"
#include "airline_logo.h"

namespace dgs {

static const char* null_dlist[] = {nullptr};

static XPLMObjectRef box_pole_obj, box_obj, base_obj, display_obj, idle_display_obj;

// wait until the Safedock-X is fully initialized, simbrief loaded etc. before showing the first display
static constexpr float kStartupDelay = 5.0f;    // s
static constexpr float kDisplayWidth = 0.92f;   // m
static constexpr float kDisplayHeight = 1.15f;  // m
static constexpr int kLogoSize = 128;           // px, width = height, for airline logo

// Blender coordinates of the lower left corner of the display in meters, relative to the Safedock-X model origin
static constexpr float kDisplayOffsetX = -0.465f;
static constexpr float kDisplayOffsetY = -0.095f;
static constexpr float kDisplayOffsetZ = 1.043f;

static constexpr float kTxtHeight = 0.16f;    // m, of full font = ascend + descend
static constexpr float kLineSpacing = 0.14f;  // m

static DynDisplay::Font* dd_font = nullptr;
static DynDisplay::Image* eq_syms_img = nullptr;  // for chocks, gpu, pca, pbb symbols

static constexpr DynDisplay::Color kTextColor = {252, 253, 193, 255};
static constexpr DynDisplay::Color kTextRed= {255, 25, 25, 255};
static constexpr DynDisplay::Color kTextGreen = {25, 255, 25, 255};
static constexpr DynDisplay::Color kTextYellow = {255, 255, 25, 255};

static constexpr float kCycleTime = 5.0f;  // seconds to show each display in the departure sequence

class Safedock_X;

class CdmPage : public DynDisplay::Page {
   private:
    Safedock_X* sdx_;
    int ofp_seqno_ = 0;  // last OFP sequence number shown on this page
    int zulu_m_ = -1;    // last zulu time in minutes shown on this page

   public:
    CdmPage(Safedock_X* sdx);
    CdmPage(const CdmPage&) = delete;
    CdmPage(CdmPage&&) = delete;
    CdmPage& operator=(const CdmPage&) = delete;
    CdmPage& operator=(CdmPage&&) = delete;

    bool Update() override;
    void UserClearCb() override { ofp_seqno_ = 0; zulu_m_ = -1; }
};

class StandPage : public DynDisplay::Page {
   private:
    Safedock_X* sdx_;
    int ofp_seqno_ = 0;  // last OFP sequence number shown on this page
    int zulu_m_ = -1;    // last zulu time in minutes shown on this page

    int pax_no_disp_ = 0; // last passenger number shown on this page

   public:
    StandPage(Safedock_X* sdx);
    StandPage(const StandPage&) = delete;
    StandPage(StandPage&&) = delete;
    StandPage& operator=(const StandPage&) = delete;
    StandPage& operator=(StandPage&&) = delete;

    bool Update() override;
    void UserClearCb() override {
        ofp_seqno_ = 0;
        zulu_m_ = -1;
        pax_no_disp_ = 0;
    }
};

class EqPage : public DynDisplay::Page {
   private:
    Safedock_X* sdx_;
    int ofp_seqno_ = 0;   // last OFP sequence number shown on this page
    EqStatus eq_status_;  // last eq status shown on this page
    int zulu_m_ = -1;    // last zulu time in minutes shown on this page

   public:
    EqPage(Safedock_X* sdx);
    EqPage(const EqPage&) = delete;
    EqPage(EqPage&&) = delete;
    EqPage& operator=(const EqPage&) = delete;
    EqPage& operator=(EqPage&&) = delete;

    bool Update() override;
    void UserClearCb() override {
        ofp_seqno_ = 0;
        eq_status_ = {};
    }
};

//------------------------------------------------------------------------------------
class Safedock_X : public DGS {
   public:
    Mode mode_ = kUndefined;

    const std::string name_;  // for logging only
    const std::string arpt_icao_;
    std::string display_name_;  // for use in the VDGS

    ObjInstRef box_inst_ref_, pole_base_inst_ref_,  // static model
               display_inst_ref_;                     // display

    float drefs_[DGS_DR_NUM]{};

    XPLMDrawInfo_t pb_drawinfo_{};  // for pole base, on ground
    float height_;
    XPLMDrawInfo_t drawinfo_{};     // height adjusted for box and display

    std::unique_ptr<ScrollTxt> scroll_txt_;
    int pax_no_ = 0, pax_no_prev_ = 0;
    float parked_ts_ = 0.0;  // parked mode start time,
    bool pole_;
    float startup_ts_ = 0.0;  // time when the Safedock_X instance was created

    // boarding detection
    float pax_change_ts_ = 0.0f;           // time when the pax_no_ was last changed
    bool boarding_ = false;                // whether boarding is active
    bool boarding_completed_ = false;      // whether boarding has completed
    float boarding_completed_ts_ = 1.0E9;  // time when boarding was completed


    CdmPage cdm_page_;       // page showing CDM info
    StandPage stand_page_;   // page showing stand name + OFP
    EqPage eq_page_;         // page showing equipment status
    int dep_page_seq_ = 0;   // for departure, which page to show next
    float cycle_ts_ = 0.0f;  // time when the current display was shown, for cycling through the displays
    std::vector<DynDisplay::Page*> dep_pages_;

    Safedock_X(const std::string& name, const std::string& arpt_icao, bool display_only, bool pole);
    ~Safedock_X() override;

    bool HasEqStatus() const noexcept override { return true; }
    bool isVdgs() const noexcept override { return true; }

    void SetGuidanceParams(const GuidanceParams& params) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo, float height) override;
    void SetPos(const XPLMDrawInfo_t& drawinfo) override;
    void SetMode(Mode mode) override;
    void SetPaxNo(int pax_no) override;
    void NotifyOfpUpdate() override;

    float Tick() override;

    void UpdateInstance() override;  // update instance position and drefs
};

// external interface
bool InitSafedock_X(const std::string& res_dir) {
    dd_font = new DynDisplay::Font(base_dir + "resources/Roboto-Medium.ttf");
    eq_syms_img = new DynDisplay::Image(res_dir + "Safedock-X-equipment_syms.png");

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
    box_pole_obj = XPLMLoadObject(on.c_str());
    if (box_pole_obj == nullptr) {
        LogMsg("Can't load box pole object from '%s'", on.c_str());
        return false;
    }

    on = res_dir + "Safedock-X.obj";
    box_obj = XPLMLoadObject(on.c_str());
    if (box_obj == nullptr) {
        LogMsg("Can't load box object from '%s'", on.c_str());
        return false;
    }

    LogMsg("Safedock_X display objects loaded from '%s'", res_dir.c_str());
    return true;
}

std::unique_ptr<DGS> CreateSafedock_X(const std::string& name, const std::string& arpt_icao, bool display_only, bool pole) {
    assert(display_obj != nullptr);
    return std::make_unique<Safedock_X>(name, arpt_icao, display_only, pole);
}

//------------------------------------------------------------------------------------
Safedock_X::Safedock_X(const std::string& name, const std::string& arpt_icao, bool display_only,
                       bool pole)
    : name_(name), arpt_icao_(arpt_icao), pole_(pole), cdm_page_(this), stand_page_(this), eq_page_(this) {
    LogMsg("Creating Safedock_X instance for stand '%s'", name_.c_str());

    display_name_ = ExtractDisplayName(name_, kR1Nchar);

    if (!display_only) {
        if (pole_) {
            box_inst_ref_ = CreateInstance(box_pole_obj, null_dlist);
            pole_base_inst_ref_ = CreateInstance(base_obj, null_dlist);
        } else
            box_inst_ref_ = CreateInstance(box_obj, null_dlist);
    }

    SetMode(kIdle);
    startup_ts_ = now;
    dep_pages_ = {&stand_page_, &cdm_page_, &eq_page_};
    LogMsg("Safedock_X instance created for stand '%s', display name: '%s'", name_.c_str(), display_name_.c_str());
}

Safedock_X::~Safedock_X() {
    LogMsg("Destroying Safedock_X instance for stand '%s'", name_.c_str());

    for (auto p : dep_pages_)
        p->Clear();

    LogMsg("active Displays after Safedock_X destructor: %d", DynDisplay::Display::active_displays());
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

void Safedock_X::SetPos(const XPLMDrawInfo_t& drawinfo, float height) {
    height_ = height;
    pb_drawinfo_ = drawinfo;
    drawinfo_ = drawinfo;
    LogMsg("Safedock_X position updated for stand '%s', x: %0.1f, y: %0.1f, z: %0.1f", name_.c_str(), drawinfo_.x,
           drawinfo_.y, drawinfo_.z);
    drawinfo_.y += height_;
    UpdateInstance();
}

void Safedock_X::SetPos(const XPLMDrawInfo_t& drawinfo) {
    SetPos(drawinfo, height_);
}

void Safedock_X::SetMode(Mode mode) {
    if (mode_ == mode)
        return;

    // if switching from departure to another mode, create the generic display
    if (mode_ == kDeparture && mode != kDeparture) {
        for (auto p : dep_pages_)
            p->Clear();
        display_inst_ref_ = CreateInstance(display_obj, dgs_dlist_dr);
    }

    mode_ = mode;
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
        display_inst_ref_ = CreateInstance(idle_display_obj, dgs_dlist_dr);
    } else if (mode_ == kParked) {
        parked_ts_ = now;
        int zm = XPLMGetDatai(zulu_time_minutes_dr);
        int zh = XPLMGetDatai(zulu_time_hours_dr);
        scroll_txt_ =  std::make_unique<ScrollTxt>(std::format("{} AIBT {:02d}{:02d}   ", plane->callsign_, zh, zm));
    } else if (mode_ == kDeboarding) {
        // keep the scroll text from parked mode, but add "DEBOARDING" to it
    } else if (mode_ == kArrival) {
        if (display_name_.empty())
            scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + "   ");
        else
            scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + " STAND " + display_name_ + "   ");

        display_inst_ref_ = CreateInstance(display_obj, dgs_dlist_dr);
    } else if (mode_ == kDeparture) {
        display_inst_ref_ = nullptr;
        dep_page_seq_ = 0;
    }

    UpdateInstance();
}

void Safedock_X::SetPaxNo(int pax_no) {
    pax_no_ = pax_no;
}

void Safedock_X::NotifyOfpUpdate() {
#if 0
    std::string ofp_str = ofp.GenDepartureStr();
    LogMsg("NotifyOfpUpdate for stand '%s', OFP departure str: '%s'", name_.c_str(), ofp_str.c_str());
    if (display_name_.empty())
        scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + "   " + ofp_str + "   ");
    else
        scroll_txt_ = std::make_unique<ScrollTxt>(arpt_icao_ + " STAND " + display_name_ + "   " +
                                                    ofp_str + "   ");
#endif
    if (ofp.seqno > 0)
        logo::LoadAirlineLogo(ofp.icao_airline, kLogoSize);  // is async
}

float Safedock_X::Tick() {
    float delay = 1.0f;
    DynDisplay::Display::BgPostProcess();  // process any completed background tasks from the background thread
    logo::CheckAsyncDownload();

    memset(drefs_, 0, sizeof(drefs_));
    DGSFillUTCBrightness(drefs_);

    if (now < startup_ts_ + kStartupDelay) {
        // wait until the Safedock-X is fully initialized, simbrief loaded etc. before showing the first display
        if (mode_ == kIdle)
            UpdateInstance();
        return 1.0f;
    }

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
        // LogMsg("Tick for parked stand '%s', refreshing eq status", name_.c_str());

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

    if (mode_ == kDeparture) {
        // boarding detection
        if (pax_no_ > 0) {
            if (pax_no_ != pax_no_prev_) {
                pax_no_prev_ = pax_no_;
                pax_change_ts_ = now;
                boarding_ = true;
                boarding_completed_ = false;
            } else if (boarding_ && !boarding_completed_ && now > pax_change_ts_ + 20.0f) {
                LogMsg("Boarding completed, pax no: %d", pax_no_);
                boarding_completed_ = true;
                boarding_ = false;
                boarding_completed_ts_ = now;
            }

            if (boarding_completed_ && now > boarding_completed_ts_ + 120.0f) {
                LogMsg("Removing stand page");
                stand_page_.Clear();  // remove page
                dep_pages_.clear();
                dep_pages_.push_back(&cdm_page_);
                dep_pages_.push_back(&eq_page_);  // remove stand page from the departure sequence
                dep_page_seq_--;
                if (dep_page_seq_ < 0)
                    dep_page_seq_ = 0;
                boarding_completed_ts_ = 1E9;  // one shot only
            }
        }

        if (now > cycle_ts_ + kCycleTime) {
            LogMsg("Tick, cycling page %d", dep_page_seq_);
            dep_pages_[dep_page_seq_]->Hide();
            dep_page_seq_++;
            if (dep_page_seq_ >= (int)dep_pages_.size())
                dep_page_seq_ = 0;

            dep_pages_[dep_page_seq_]->Show();
            int next_seq = dep_page_seq_ + 1;
            if (next_seq >= (int)dep_pages_.size())
                next_seq = 0;
            dep_pages_[next_seq]->Update();  // update the page for the next cycle in bg
            cycle_ts_ = now;
        }
    }

    if (mode_ == kDeboarding && show_eq_status_small && pax_no_ > 0) {
        int blink = ((size_t)now) & 1;  // blink at 1Hz
        drefs_[DGS_DR_DEBOARDING] = blink;
        delay = std::min(delay, 0.2f);
        // LogMsg("Tick for deboarding stand '%s', pax no: %d, dref: %f", name_.c_str(), pax_no_,
        // drefs_[DGS_DR_DEBOARDING]);
    }

    UpdateInstance();
    return delay;
}

void Safedock_X::UpdateInstance() {
    if (display_inst_ref_)
        display_inst_ref_->SetPosition(&drawinfo_, drefs_);

    cdm_page_.SetPosition();

    if (box_inst_ref_)
        box_inst_ref_->SetPosition(&drawinfo_, nullptr);

    if (pole_base_inst_ref_)
        pole_base_inst_ref_->SetPosition(&pb_drawinfo_, nullptr);
}

//------------------------------------------------------------------------------------

// Convenience wrapper to create a DynDisplay::Display with the correct size and position for the Safedock-X display
static inline DynDisplay::DisplayPtr CreateDisplayX() {
    return DynDisplay::Display::Create(kDisplayWidth, kDisplayHeight, kDisplayOffsetX, kDisplayOffsetY,
                                       kDisplayOffsetZ);
}

CdmPage::CdmPage(Safedock_X* sdx) : Page(sdx->drawinfo_), sdx_(sdx) {}

// a - b in minutes, taking care of the day wrap around
static int TimeDiff(int a_h, int a_m, int b_h, int b_m) {
    int diff = (a_h * 60 + a_m) - (b_h * 60 + b_m);
    if (diff < -720)
        diff += 1440;
    else if (diff > 720)
        diff -= 1440;
    return diff;
}

void DisplayDiffToTOBT(DynDisplay::DisplayPtr& dd, int zulu_h, int zulu_m, const std::string& tobt) {
    int tobt_h = std::atoi(tobt.substr(0, 2).c_str());
    int tobt_m = std::atoi(tobt.substr(2, 2).c_str());
    int td = TimeDiff(tobt_h, tobt_m, zulu_h, zulu_m);
    LogMsg("DisplayDiffToTOBT: td: %d", td);
    bool expired = false;
    if (td < 0) {
        expired = true;
        td = -td;
    }
    int h = td / 60;
    int m = td % 60;
    if (td == 0)
        dd->TextLine(kTextYellow, *dd_font, "00", true);
    else if (expired) {
        if (h > 0)
            dd->TextLine(kTextRed, *dd_font, std::format("-{:02d}:{:02d}", h, m), true);
        else
            dd->TextLine(kTextRed, *dd_font, std::format("-{:02d}", m), true);
    } else {
        if (h > 0)
            dd->TextLine(kTextGreen, *dd_font, std::format("+{:02d}:{:02d}", h, m), true);
        else
            dd->TextLine(kTextGreen, *dd_font, std::format("+{:02d}", m), true);
    }
}

bool CdmPage::Update() {
    if (ofp.seqno <= 0)
        return false;

    int zulu_m = XPLMGetDatai(zulu_time_minutes_dr);
    int zulu_h = XPLMGetDatai(zulu_time_hours_dr);

    if (dd_ == nullptr || ofp.seqno > ofp_seqno_ || zulu_m != zulu_m_) {
        LogMsg("Updating CdmPage for stand '%s', ofp seqno: %d", sdx_->name_.c_str(), ofp.seqno);
        LogMsg("airline logo is available: %d", logo::airline_logo != nullptr);
        TimeCodeBlock tc("CdmPage::Update");

        ofp_seqno_ = ofp.seqno;
        zulu_m_ = zulu_m;

        dd_ = CreateDisplayX();

        static constexpr float kCol1 = 0.05;  // column for TOBT, TSAT, CTOT, RWY, SID
        static constexpr float kCol2 = 0.55;  // column value

        float y = kDisplayHeight - kLineSpacing + 0.01f;  // start from the very top

        dd_->SetTxtHeight(kTxtHeight);
        dd_->SetPos(kCol1, y);
        dd_->SetLineSpacing(kLineSpacing);

        auto Line = [&](const std::string& txt, bool centered = false) {
            dd_->TextLine(kTextColor, *dd_font, txt, centered);
        };

        auto Line2Col = [&](const std::string& txt1, const std::string& txt2) {
            dd_->SetPos(kCol1, -1);
            dd_->TextLine(kTextColor, *dd_font, txt1, false);
            dd_->SameLine();
            dd_->SetPos(kCol2, -1);  // move to next column
            dd_->TextLine(kTextColor, *dd_font, txt2, false);
        };

        if (logo::airline_logo) {
            dd_->TextAt(0.25f, y, kTextColor, *dd_font, ofp.callsign);
            dd_->Paste(*logo::airline_logo.get(), 0.05f, y - 0.02f);
        } else
            Line(ofp.callsign, true);

        Line(ofp.destination, true);

        time_t out_time = std::atol(ofp.est_out.c_str());
        time_t off_time = std::atol(ofp.est_off.c_str());
        std::string out;

        int eobt_h = 0;
        int eobt_m = 0;

        bool have_cdm{false};
        if (ofp.cdm_tobt.empty()) {
            auto out_tm = *std::gmtime(&out_time);
            eobt_h = out_tm.tm_hour;
            eobt_m = out_tm.tm_min;
            out = std::format("{:02d}{:02d}", eobt_h, eobt_m);
        } else {
            have_cdm = true;
            Line2Col("TOBT", ofp.cdm_tobt);
            DisplayDiffToTOBT(dd_, zulu_h, zulu_m, ofp.cdm_tobt);
        }

        if (!ofp.cdm_tsat.empty())
            Line2Col("TSAT", ofp.cdm_tsat);

        if (!ofp.cdm_ctot.empty())
            Line2Col("CTOT", ofp.cdm_ctot);

        if (!have_cdm) {
            Line2Col("OUT", out);
            auto off_tm = *std::gmtime(&off_time);
            Line2Col("OFF", std::format("{:02d}{:02d}", off_tm.tm_hour, off_tm.tm_min));
        }

        if (!ofp.cdm_runway.empty())
            Line2Col("RWY", ofp.cdm_runway);

        if (!ofp.cdm_sid.empty())
            Line(ofp.cdm_sid, true);

        dd_->Bake(dyn_display_obj_dir, "X-ofp");
    }

    return true;
}

//------------------------------------------------------------------------------------
StandPage::StandPage(Safedock_X* sdx) : Page(sdx->drawinfo_), sdx_(sdx) {}

bool StandPage::Update() {
    int zulu_m = XPLMGetDatai(zulu_time_minutes_dr);

    if (dd_ == nullptr || ofp.seqno > ofp_seqno_ || zulu_m != zulu_m_ || sdx_->pax_no_ != pax_no_disp_) {
        LogMsg("Updating StandPage for stand '%s', ofp seqno: %d, pax_no: %d", sdx_->name_.c_str(), ofp.seqno, sdx_->pax_no_);
        TimeCodeBlock tc("StandPage::Update");
        dd_ = CreateDisplayX();
        static constexpr float kCol1 = 0.05;  // label
        static constexpr float kCol2 = 0.55;  // value

        dd_->SetPos(0.0f, kDisplayHeight + 0.01f - kLineSpacing);  // start from the very top
        dd_->SetTxtHeight(kTxtHeight);
        dd_->SetLineSpacing(kLineSpacing);


        // if we have an ofp we start with callsign / destination
        if (ofp.seqno > 0) {
            if (logo::airline_logo) {
                auto [x, y] = dd_->GetPos();
                dd_->Paste(*logo::airline_logo.get(), 0.05f, y - 0.02f);
                dd_->TextAt(0.25f, y, kTextColor, *dd_font, ofp.callsign);
           } else
                dd_->TextLine(kTextColor, *dd_font, ofp.callsign, true);

            dd_->TextLine(kTextColor, *dd_font, ofp.destination, true);
        }

        // stand + clock
        dd_->SetPosDelta(0.0f, -0.02f);  // move down a bit
        if (!sdx_->display_name_.empty())
            dd_->TextLine(kTextColor, *dd_font, sdx_->display_name_, true);

        zulu_m_ = zulu_m;
        int zulu_h = XPLMGetDatai(zulu_time_hours_dr);
        dd_->TextLine(kTextColor, *dd_font, std::format("{:02d}:{:02d}", zulu_h, zulu_m), true);

        dd_->SetPosDelta(0.0f, -0.06f);  // move down a bit
        if (sdx_->boarding_) {
            pax_no_disp_ = sdx_->pax_no_;
            dd_->TextLine(kTextColor, *dd_font, "BOARDING", true);
            dd_->TextLine(kTextColor, *dd_font, std::format("Pax  {:3d}", pax_no_disp_), true);
        } else if (sdx_->boarding_completed_) {
            dd_->TextLine(kTextGreen, *dd_font, "BOARDING", true);
            dd_->TextLine(kTextGreen, *dd_font, "COMPLETE", true);
            dd_->TextLine(kTextColor, *dd_font, std::format("Pax   {:3d}", pax_no_disp_), true);
         } else if (ofp.seqno > 0) {
            ofp_seqno_ = ofp.seqno;

            dd_->SetPosDelta(0.0f, -0.02f);  // move down a bit
            dd_->TextAt(kCol1, -1, kTextColor, *dd_font, "Pax");
            dd_->SameLine();
            dd_->TextAt(kCol2, -1, kTextColor, *dd_font, ofp.pax_count);

            float cargo = std::atof(ofp.freight.c_str()) / 1000.0f;
            if (cargo > 0.0f) {
                dd_->TextAt(kCol1, -1, kTextColor, *dd_font, "Cargo");
                dd_->SameLine();
                dd_->TextAt(kCol2, -1, kTextColor, *dd_font, std::format("{:.1f}", cargo));
            }

            float fuel = std::atof(ofp.fuel_plan_ramp.c_str()) / 1000.0f;
            if (fuel > 0.0f) {
                dd_->TextAt(kCol1, -1, kTextColor, *dd_font, "Fuel");
                dd_->SameLine();
                dd_->TextAt(kCol2, -1, kTextColor, *dd_font, std::format("{:.1f}", fuel));
            }
        }

        dd_->Bake(dyn_display_obj_dir, "X-stand");
    }

    return true;
}

//------------------------------------------------------------------------------------
EqPage::EqPage(Safedock_X* sdx) : Page(sdx->drawinfo_), sdx_(sdx) {}

bool EqPage::Update() {
    EqStatus cur_eq_status{eq_status_};
    plane->GetEqStatus(eq_status_);

    int zulu_m = XPLMGetDatai(zulu_time_minutes_dr);
    int zulu_h = XPLMGetDatai(zulu_time_hours_dr);

    if (dd_ == nullptr || cur_eq_status != eq_status_ || ofp.seqno > ofp_seqno_ || zulu_m != zulu_m_) {
        LogMsg("Updating EqPage for stand '%s', ofp seqno: %d", sdx_->name_.c_str(), ofp.seqno);
        TimeCodeBlock tc("EqPage::Update");
        ofp_seqno_ = ofp.seqno;
        zulu_m_ = zulu_m;

        dd_ = CreateDisplayX();
        dd_->Paste(*eq_syms_img, 0.05f, 0.05f);  // paste the equipment symbols image

        static constexpr float kCol1 = 0.05;  // column for TOBT
        static constexpr float kCol2 = 0.55;  // column value

        dd_->SetPos(0.0f, kDisplayHeight + 0.01f - kLineSpacing);  // start the very top
        dd_->SetTxtHeight(kTxtHeight);
        dd_->SetLineSpacing(kLineSpacing);

        if (ofp.seqno > 0) {
            if (logo::airline_logo) {
                auto [_, y] = dd_->GetPos();
                dd_->TextAt(0.25f, y, kTextColor, *dd_font, ofp.callsign);
                dd_->Paste(*logo::airline_logo.get(), 0.05f, y - 0.02f);
            } else
                dd_->TextLine(kTextColor, *dd_font, ofp.callsign, true);

            dd_->SetPos(kCol1, -1);
            dd_->SetScale(0.8f);  // scale down the text to fit the eq status on the display

            dd_->TextAt(kCol1, -1, kTextColor, *dd_font, "TOBT");
            dd_->SameLine();
            dd_->TextAt(kCol2, -1, kTextColor, *dd_font, ofp.cdm_tobt);
            DisplayDiffToTOBT(dd_, zulu_h, zulu_m, ofp.cdm_tobt);
            dd_->SetScale(1.0f);  // reset the scale
        }

        float x = 0.22f;
        float col2 = 0.60f;  // column for On/Off status
        float y = 0.07f;
        float dy = 0.18f;

        auto OnOff = [&](int status) {
            if (status == kEqOn)
                dd_->TextAt(col2, y, kTextGreen, *dd_font, "On");
            else if (status == kEqOff)
                dd_->TextAt(col2, y, kTextRed, *dd_font, "Off");
            else
                dd_->TextAt(col2, y, kTextColor, *dd_font, "n/a");
        };

        dd_->SetTxtHeight(0.17f);

        // bottom up
        dd_->TextAt(x, y, kTextColor, *dd_font, "PCA");
        OnOff(eq_status_.pca);

        y += dy;
        dd_->TextAt(x, y, kTextColor, *dd_font, "PBB");
        OnOff(eq_status_.pbb);

        y += dy;
        dd_->TextAt(x, y, kTextColor, *dd_font, "GPU");
        OnOff(eq_status_.gpu);

        y += dy;
        dd_->TextAt(x, y, kTextColor, *dd_font, "CHK");
        OnOff(eq_status_.chocks);

        dd_->Bake(dyn_display_obj_dir, "X-eq");
    }

    return true;
}

}  // namespace dgs
