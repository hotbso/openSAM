//
//    AutoDGS / openSAM: Manage DGS
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

#ifndef _DGS_IMPL_H_
#define _DGS_IMPL_H_

#include "dgs.h"

namespace dgs {

extern XPLMDataRef ground_speed_dr;


enum DgsDref {
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_XTRACK,
    DGS_DR_DISTANCE,
    DGS_DR_DISTANCE_0,   // if distance < 10: full meters digit
    DGS_DR_DISTANCE_01,  // first decimal digit
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,

    DGS_DR_R1_SCROLL,
    DGS_DR_R1C0,  // top row (=1), char #
    DGS_DR_R1C1,
    DGS_DR_R1C2,
    DGS_DR_R1C3,
    DGS_DR_R1C4,
    DGS_DR_R1C5,

    DGS_DR_BOARDING,    // boarding state 0/1
    DGS_DR_PAXNO_0,     // 3 digits
    DGS_DR_PAXNO_1,
    DGS_DR_PAXNO_2,

    DGS_DR_DEBOARDING,  // deboarding state 0/1

    DGS_DR_UTC_M0,           // time UTC in minutes, full minutes digit
    DGS_DR_UTC_M1,           // time UTC in minutes, first decimal digit
    DGS_DR_UTC_H0,           // time UTC in hours, full hours digit
    DGS_DR_UTC_H1,           // time UTC in hours, first decimal digit

    DGS_DR_VDGS_BRIGHTNESS,  // brightness for VDGS, 0-1
    DGS_DR_BLINK,            // for blinking (e.g. left/right), 0/1
    DGS_DR_MOVE,             // moves from 0 to 1, e.g for lead in
    DGS_DR_STATUS_1,         // implementation specific status, e.g. for Safedock_X parked with eq status
    DGS_DR_CHK,              // chocks status for parked plane, 0/1
    DGS_DR_GPU,              // gpu status for parked plane, 0/1
    DGS_DR_PCA,              // pca status for parked plane, 0/1
    DGS_DR_PBB,              // pbb status for parked plane, 0
    DGS_DR_NUM               // # of drefs
};

// the standard status values for guidance
enum DgsGuidanceStatus {
    kDgsGstIdle,        // idle, no plane, or plane far away
    kDgsGstIdentified,  // plane id, e.g. for VDGS, or "this way" for Marshaller
    kDgsGstStop,        // stop position
    kDgsGstOk,          // ok position
    kDgsGstTooFar       // too far
};

struct GuidanceParams {
    DgsGuidanceStatus status;
    int track, lr;
    float xtrack, distance;
    float ref_x, ref_z; // raw values of plane's ref point to stand
};

static constexpr int kR1Nchar = 6;

extern const char *dgs_dlist_dr[];

class ScrollTxt {
    std::string txt_;           // text to scroll
    int char_pos_;              // next char to enter on the right
    int dr_scroll_;             // dref value for scroll ctrl
    char chars_[kR1Nchar]{};    // chars currently visible

public:
    ScrollTxt(const std::string& txt);
    void Tick(float *drefs);
};

// Helper to trim whitespace
static inline void trim(std::string& s) {
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
}

// fill the UTC datarefs, and for VDGS also the brightness dataref
extern void DGSFillUTCBrightness(float *drefs);
extern void DGSFillEqStatus(float *drefs, const EqStatus& eq_status);

extern bool InitMarshaller(const std::string& obj_dir);
extern bool InitSafedock_T2_24(const std::string& obj_dir);
extern bool InitSafedock_X(const std::string& obj_dir);

} // namespace dgs

#endif
