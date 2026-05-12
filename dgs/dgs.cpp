//
//    AutoDGS / openSAM: ManageVDGS
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

#include <cmath>
#include <algorithm>
#include "log_msg.h"
#include "dgs.h"
#include "dgs_impl.h"

#include "XPLMDataAccess.h"

namespace dgs {

static constexpr float kMinBrightness = 0.025;  // relative to 1
static constexpr float kMinEv100 = 6.0f;
static constexpr float kMaxEv100 = 11.0f;

XPLMDataRef ground_speed_dr;

#ifndef DREF_PREFIX
#error "DREF_PREFIX must be defined when compiling dgs.cpp"
#endif

// keep exactly the same order as in enum _DGS_DREF
const char *dgs_dlist_dr[] = {
    DREF_PREFIX "/dgs/status",
    DREF_PREFIX "/dgs/lr",
    DREF_PREFIX "/dgs/track",
    DREF_PREFIX "/dgs/xtrack",
    DREF_PREFIX "/dgs/distance",
    DREF_PREFIX "/dgs/distance_0",
    DREF_PREFIX "/dgs/distance_01",
    DREF_PREFIX "/dgs/icao_0",
    DREF_PREFIX "/dgs/icao_1",
    DREF_PREFIX "/dgs/icao_2",
    DREF_PREFIX "/dgs/icao_3",
    DREF_PREFIX "/dgs/r1_scroll",
    DREF_PREFIX "/dgs/r1c0",
    DREF_PREFIX "/dgs/r1c1",
    DREF_PREFIX "/dgs/r1c2",
    DREF_PREFIX "/dgs/r1c3",
    DREF_PREFIX "/dgs/r1c4",
    DREF_PREFIX "/dgs/r1c5",
    DREF_PREFIX "/dgs/boarding",
    DREF_PREFIX "/dgs/paxno_0",
    DREF_PREFIX "/dgs/paxno_1",
    DREF_PREFIX "/dgs/paxno_2",
    DREF_PREFIX "/dgs/deboarding",
    DREF_PREFIX "/dgs/time_utc_m0",
    DREF_PREFIX "/dgs/time_utc_m1",
    DREF_PREFIX "/dgs/time_utc_h0",
    DREF_PREFIX "/dgs/time_utc_h1",
    DREF_PREFIX "/dgs/vdgs_brightness",
    DREF_PREFIX "/dgs/blink",
    DREF_PREFIX "/dgs/move",
    DREF_PREFIX "/dgs/status_1",
    DREF_PREFIX "/dgs/chk",
    DREF_PREFIX "/dgs/gpu",
    DREF_PREFIX "/dgs/pca",
    DREF_PREFIX "/dgs/pbb",
    NULL
};

DGS::~DGS() {}; // required sytax for pure virtual destructor

//------------------------------------------------------------------------------------
ScrollTxt::ScrollTxt(const std::string& txt) {
    txt_ = txt;
    dr_scroll_ = 10;  // right most
    chars_[kR1Nchar - 1] = txt_[0];
    char_pos_ = 0;
}

void ScrollTxt::Tick(float* drefs) {
    if (txt_.size() == 0)
        return;

    dr_scroll_ -= 2;
    if (dr_scroll_ < 0) {
        dr_scroll_ = 10;
        char_pos_++;
        if (char_pos_ >= (int)txt_.size())
            char_pos_ = 0;

        for (int i = 1; i < kR1Nchar; i++)
            chars_[i - 1] = chars_[i];
        chars_[kR1Nchar - 1] = txt_[char_pos_];
    }

    drefs[DGS_DR_R1_SCROLL] = dr_scroll_;
    for (int i = 0; i < kR1Nchar; i++)
        drefs[DGS_DR_R1C0 + i] = chars_[i];
};

//------------------------------------------------------------------------------------

// Dummy dataref accessor
static float DummyDgsFloat([[maybe_unused]] void* ref) {
    return 0.0f;
}

static bool ev100_probed = false;
static XPLMDataRef ev100_dr, zulu_time_sec_dr, zulu_time_minutes_dr, zulu_time_hours_dr, percent_lights_dr;

bool InitDGS(const std::string& res_dir) {
    zulu_time_sec_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_seconds");
    zulu_time_minutes_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_minutes");
    zulu_time_hours_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_hours");
    percent_lights_dr = XPLMFindDataRef("sim/cockpit2/lights/percent_lights_on");
    ground_speed_dr = XPLMFindDataRef("sim/flightmodel/position/groundspeed");

    // register datarefs
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL, NULL, DummyDgsFloat, NULL, NULL, NULL, NULL,
                                NULL, NULL, NULL, NULL, NULL, NULL, 0);

    if (!InitMarshaller(res_dir) || !InitSafedock_T2_24(res_dir) || !InitSafedock_X(res_dir)) {
        LogMsg("init failure: Can't load DGS objects");
        return false;
    }

    return true;
}

void DGSFillUTCBrightness(float* drefs) {
    // a poor man's cache
    static int zm, zh, zs_prev = -1;
    static float vdgs_brightness;

    // private datarefs are usually intialized late, so we probe here when we actually need it for the first time
    if (!ev100_probed) {
        ev100_dr = XPLMFindDataRef("sim/private/controls/photometric/ev100");
        if (ev100_dr)
            LogMsg("ev100 dataref mapped");
        else
            LogMsg("ev100 dataref not found, using percent_lights_on as fallback for brightness");

        ev100_probed = true;
    }

    int zs = XPLMGetDatai(zulu_time_sec_dr);
    if (zs != zs_prev) {
        zs_prev = zs;
        zm = XPLMGetDatai(zulu_time_minutes_dr);
        zh = XPLMGetDatai(zulu_time_hours_dr);

        // brightness for VDGS

        if (ev100_dr) {
            // if ev100 is available, we use it to set brightness
            float ev100 = XPLMGetDataf(ev100_dr);
            ev100 = std::clamp(ev100, kMinEv100, kMaxEv100);
            const float f = (ev100 - kMinEv100) / (kMaxEv100 - kMinEv100);
            // ev100 is logarithmic and vdgs_brightness linear, so we use exp here
            const float exp_f = (std::exp(f) - 1.0f) / (std::exp(1.0f) - 1.0f);
            vdgs_brightness = kMinBrightness + (1.0f - kMinBrightness) * exp_f;
            // LogMsg("ev100: %0.2f, vdgs_brightness: %0.3f", ev100, vdgs_brightness);
        } else {
            // fallback: use percent_lights_on
            vdgs_brightness =
                kMinBrightness + (1.0f - kMinBrightness) * std::pow(1.0f - XPLMGetDataf(percent_lights_dr), 6.0f);
        }
    }

    drefs[DGS_DR_UTC_M0] = zm % 10;
    drefs[DGS_DR_UTC_M1] = zm / 10;
    drefs[DGS_DR_UTC_H0] = zh % 10;
    drefs[DGS_DR_UTC_H1] = zh / 10;
    drefs[DGS_DR_VDGS_BRIGHTNESS] = vdgs_brightness;
}

void DGSFillEqStatus(float* drefs, const EqStatus& eq_status) {
    drefs[DGS_DR_CHK] = eq_status.chocks;
    drefs[DGS_DR_GPU] = eq_status.gpu;
    drefs[DGS_DR_PCA] = eq_status.pca;
    drefs[DGS_DR_PBB] = eq_status.pbb;
}

} // namespace dgs
