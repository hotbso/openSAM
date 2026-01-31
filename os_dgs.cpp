//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
//              (C) Jonathan Harris 2006-2013
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

#include <cstddef>
#include <cmath>
#include <cstring>
#include <cassert>
#include <algorithm>

#include "openSAM.h"
#include "os_dgs.h"
#include "plane.h"
#include "simbrief.h"

#include "XPLMInstance.h"
#include "XPLMNavigation.h"

// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static constexpr float kCapA = 15;   // Capture
static constexpr float kCapZ = 140;  // (50-80 in Safedock2 flier)

static constexpr float kAziA = 15;      // provide azimuth guidance
static constexpr float kAziZ = 90;

static constexpr float kAziCrossover = 6;  // m, switch from azimuth to xtrack guidance

static constexpr float kGoodZ_p = 0.2;   // stop position for nw / to stop
static constexpr float kGoodZ_m = -0.5;  // stop position for nw / beyond stop

static constexpr float kGoodX = 2.0;  // for mw

static constexpr float kCrZ = 12;  // Closing Rate starts here VDGS, Marshaller uses 0.5 * kCrZ

static constexpr int kTurnRight = 1;  // arrow on left side
static constexpr int kTurnLeft = 2;   // arrow on right side

static constexpr float kMaxDgs2StandX = 10.0f;  // max offset/distance from DGS to stand
static constexpr float kMaxDgs2StandZ = 80.0f;

static constexpr float kDgsDist = 20.0f;  // distance from dgs to stand for azimuth computation

static constexpr int kR1Nchar = 6;  // chars in row 1 of the VDGS

class ScrollTxt {
    std::string txt_;         // text to scroll
    int char_pos_;            // next char to enter on the right
    int dr_scroll_;           // dref value for scroll ctrl
    char chars_[kR1Nchar]{};  // chars currently visible

   public:
    ScrollTxt(const std::string &txt);
    float Tick();
};

// types
typedef enum {
    DISABLED = 0,
    INACTIVE,
    DEPARTURE,
    BOARDING,
    ARRIVAL,
    ENGAGED,
    TRACK,
    GOOD,
    BAD,
    PARKED,
    CHOCKS,
    DONE
} state_t;

static const char *const state_str[] = {"DISABLED", "INACTIVE", "DEPARTURE", "BOARDING", "ARRIVAL", "ENGAGED",
                                        "TRACK",    "GOOD",     "BAD",       "PARKED",   "CHOCKS",  "DONE"};
static state_t state = DISABLED;
static float timestamp;  // for various states in the state_machine

// Datarefs
static XPLMDataRef percent_lights_dr, ev100_dr, sin_wave_dr, zulu_time_minutes_dr, zulu_time_hours_dr, ground_speed_dr;

// Published DataRef values
static int status, track, lr;
static float distance;

static Stand *active_stand;
static float active_stand_ts;  // timestamp of last FindNearestStand()
// track the max local z (= closest to stand) of dgs objs for active_stand
static float assoc_dgs_z_l, assoc_dgs_x_l, assoc_dgs_ts;
static bool dgs_assoc;  // stand is associated with a dgs

static std::string arpt_icao;                  // departure airport
static std::string display_name;               // departure stand's 'net' name <= kR1Nchar
static std::unique_ptr<ScrollTxt> scroll_txt;  // on departure stand

static int is_marshaller;
static float marshaller_x, marshaller_y, marshaller_z, marshaller_y_0, marshaller_psi;
static XPLMObjectRef marshaller_obj, stairs_obj;
static XPLMInstanceRef marshaller_inst, stairs_inst;

static int update_dgs_log_ts;  // throttling of logging
static float sin_wave_prev;

static float time_utc_m0, time_utc_m1, time_utc_h0, time_utc_h1, vdgs_brightness;

static std::unique_ptr<Ofp> ofp;
static int ofp_seqno;
static float ofp_ts;

enum _DGS_DREF {
    DGS_DR_IDENT,
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
    DGS_DR_BOARDING,  // boarding state 0/1
    DGS_DR_PAXNO_0,   // 3 digits
    DGS_DR_PAXNO_1,
    DGS_DR_PAXNO_2,
    DGS_DR_NUM  // # of drefs
};

// keep exactly the same order as list above
static const char *dgs_dlist_dr[] = {
    "opensam/dgs/ident",
    "opensam/dgs/status",
    "opensam/dgs/lr",
    "opensam/dgs/track",
    "opensam/dgs/xtrack",
    "opensam/dgs/distance",
    "opensam/dgs/distance_0",
    "opensam/dgs/distance_01",
    "opensam/dgs/icao_0",
    "opensam/dgs/icao_1",
    "opensam/dgs/icao_2",
    "opensam/dgs/icao_3",
    "opensam/dgs/r1_scroll",
    "opensam/dgs/r1c0",
    "opensam/dgs/r1c1",
    "opensam/dgs/r1c2",
    "opensam/dgs/r1c3",
    "opensam/dgs/r1c4",
    "opensam/dgs/r1c5",
    "opensam/dgs/boarding",
    "opensam/dgs/paxno_0",
    "opensam/dgs/paxno_1",
    "opensam/dgs/paxno_2",
    nullptr
};

static float drefs[DGS_DR_NUM];

// SAM1 drefs
enum _SAM1_DREF {
    SAM1_DR_STATUS,
    SAM1_DR_LATERAL,
    SAM1_DR_LONGITUDINAL,
    SAM1_DR_ICAO,
};

enum _SAM1_STATE { SAM1_TRACK = 1, SAM1_STOP_ZONE, SAM1_IDLE };
static constexpr float SAM1_LATERAL_OFF = 10.0f;  // switches off VDGS

// dref values
static float sam1_status, sam1_lateral, sam1_longitudinal;

//------------------------------------------------------------------------------------
ScrollTxt::ScrollTxt(const std::string &txt) {
    txt_ = txt;

    // don't scroll short texts
    if (txt_.size() > kR1Nchar) {
        dr_scroll_ = 10;  // right most
        chars_[kR1Nchar - 1] = txt_[0];
        char_pos_ = 0;
    } else {
        dr_scroll_ = 0;
        int ofs = (kR1Nchar - (int)txt_.size()) / 2;
        for (int i = 0; i < (int)txt_.size(); i++) {
            chars_[i + ofs] = txt_[i];
        }
    }
}

float ScrollTxt::Tick() {
    float delay = 4.0f;

    if (txt_.size() == 0)
        return delay;

    if (txt_.size() > kR1Nchar) {
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
        delay = 0.05f;  // scrolling, short delay
    }

    drefs[DGS_DR_R1_SCROLL] = dr_scroll_;
    for (int i = 0; i < kR1Nchar; i++)
        drefs[DGS_DR_R1C0 + i] = chars_[i];

    return delay;
}

void DgsSetInactive(void) {
    LogMsg("dgs set to INACTIVE");
    active_stand = nullptr;
    state = INACTIVE;

    if (marshaller_inst) {
        XPLMDestroyInstance(marshaller_inst);
        marshaller_inst = nullptr;
        if (stairs_inst) {
            XPLMDestroyInstance(stairs_inst);
            stairs_inst = nullptr;
        }
    }
}

// set mode to arrival
void DgsSetArrival(void) {
    if (!my_plane.on_ground()) {
        LogMsg("can't set active when not on ground");
        return;
    }

    // can be teleportation
    DgsSetInactive();
    my_plane.ResetBeacon();

    float lat = my_plane.lat();
    float lon = my_plane.lon();
    char airport_id[50];

    // find and load airport I'm on now
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, airport_id, NULL, NULL);
        LogMsg("now on airport: %s", airport_id);
    }

    state = ARRIVAL;
    LogMsg("dgs set to ARRIVAL");
}

// xform lat,lon into the active global frame
void Stand::Xform2RefFrame() {
    if (ref_gen_ < ::ref_gen) {
        XPLMWorldToLocal(lat, lon, my_plane.elevation(), &stand_x, &stand_y, &stand_z);
        ref_gen_ = ::ref_gen;
        dgs_assoc = false;  // association is lost
        assoc_dgs_z_l = -1.0E10;
        assoc_dgs_x_l = 1.0E10;
        assoc_dgs_ts = 1.0E10;
    }
}

// xform global coordinates into the stand frame
void Stand::Global2Stand(float x, float z, float &x_l, float &z_l) {
    float dx = x - stand_x;
    float dz = z - stand_z;

    x_l = dx * cos_hdgt + dz * sin_hdgt;
    z_l = -dx * sin_hdgt + dz * cos_hdgt;
}

//
// Check whether dgs obj is the (an) active one.
//
// We are looking for the DGS closest to the stand.
// First of all it must be in a box around the stand and reasonably aligned.
//
// Assume that we have an association with x/z values in assoc_dgs_[xz]...
//
// Primary criterion is max z value in the stand's local system
// but...
// if distance of a new candidate (z) is nearly the same but closer to stand's centerline (x)
//    consider it nearer even if z is slightly less
//
static inline bool IsDgsActive(float obj_x, float obj_z, float obj_psi) {
    if (nullptr == active_stand)
        return false;

    stat_dgs_acc++;

    float dgs_x_l, dgs_z_l;
    active_stand->Global2Stand(obj_x, obj_z, dgs_x_l, dgs_z_l);
    // LogMsg("dgs_x_l: %0.2f, dgs_z_l: %0.2f", dgs_x_l, dgs_z_l);

    if (dgs_assoc && (dgs_z_l < assoc_dgs_z_l - 2.0f || fabsf(dgs_x_l) > assoc_dgs_x_l))
        return false;  // already have a closer one

    // must be in a box +- kMaxDgs2StandX, kMaxDgs2StandZ
    // and reasonably aligned with stand (or for SAM1 anti aligned)
    if (fabsf(dgs_x_l) > kMaxDgs2StandX || dgs_z_l < -kMaxDgs2StandZ || dgs_z_l > -5.0f ||
        BETWEEN(fabsf(RA(active_stand->hdgt - obj_psi)), 10.0f, 170.0f))
        return false;

    // we found one
    if ((dgs_z_l > assoc_dgs_z_l - 2.0f && fabsf(dgs_x_l) < assoc_dgs_x_l - 1.0f) || (dgs_z_l > assoc_dgs_z_l)) {
        is_marshaller = 0;  // associated to a new dgs, don't know yet whether it's a marshaller
        assoc_dgs_z_l = dgs_z_l;
        assoc_dgs_x_l = fabsf(dgs_x_l);
        assoc_dgs_ts = now;
        LogMsg("associating DGS: dgs_x_l: %0.2f, dgs_z_l: %0.2f", dgs_x_l, dgs_z_l);
    }

    dgs_assoc = true;
    return true;
}

// Dataref accessor for the global datarefs, *_utc_* + brightness
static float DgsGlobalAcc(void *ref) {
    if (ref == nullptr)
        return -1.0f;

    return *(float *)ref;
}

//
// Accessor for the "opensam/dgs/..." datarefs
//
// This function is called from draw loops, efficient coding required.
//
//
static float DgsActiveAcc(void *ref) {
    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    if (obj_x == 0.0f && obj_z == 0.0f && obj_psi == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    int dr_index = (uint64_t)ref;

    if (!IsDgsActive(obj_x, obj_z, obj_psi))
        return 0.0f;

    if (DGS_DR_IDENT == dr_index) {
        if (fabsf(RA(active_stand->hdgt - obj_psi)) > 10.0f)  // no anti alignment for the Marshaller
            return 0.0;

        // if last nearest dgs was found 2 seconds ago
        // this should be the nearest one in this stand's bbox
        if (now > assoc_dgs_ts + 2.0f) {
            is_marshaller = 1;  // only marshaller queries ident
            marshaller_x = obj_x;
            marshaller_y = XPLMGetDataf(draw_object_y_dr);
            marshaller_z = obj_z;
            marshaller_psi = obj_psi;
            return 0.0f;
        }
    }

    return drefs[dr_index];
}

//
// Accessor for the "sam/..." docking related datarefs
//
// This function is called from draw loops, efficient coding required.
//
static float DgsSam1Acc(void *ref) {
    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    if (obj_x == 0.0f && obj_z == 0.0f && obj_psi == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    int dr_index = (uint64_t)ref;
    if (!IsDgsActive(obj_x, obj_z, obj_psi))
        switch (dr_index) {
            case SAM1_DR_STATUS:
                return SAM1_IDLE;
            case SAM1_DR_LATERAL:
                return SAM1_LATERAL_OFF;  // switch off VDGS
            case SAM1_DR_LONGITUDINAL:
                return 0.0f;
        }

    // LogMsg("DgsSam1Acc: %d", dr_index);
    switch (dr_index) {
        case SAM1_DR_STATUS:
            return sam1_status;
        case SAM1_DR_LATERAL:
            return sam1_lateral;
        case SAM1_DR_LONGITUDINAL:
            return sam1_longitudinal;
    }

    return 0.0f;
}

//
// Accessor for the "sam/...icao[0]" docking related datarefs
//
// This function is called from draw loops, efficient coding required.
//
static int DgsSam1IcaoAcc([[maybe_unused]] XPLMDataRef ref, int *values, int ofs, int n) {
    if (values == nullptr)
        return 4;

    if (n <= 0 || ofs < 0 || ofs >= 4)
        return 0;

    n = std::min(n, 4 - ofs);
    const char *icao = my_plane.icao().c_str();
    for (int i = 0; i < n; i++) {
        char c = icao[ofs + i];
        if (isalpha(c))
            values[i] = (c - 'A') + 1;
        else
            values[i] = (c - '0') + 27;
    }

    return n;
}

static void FindNearestStand() {
    double dist = 1.0E10;
    Stand *min_stand = nullptr;

    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    float plane_x = my_plane.x();
    float plane_z = my_plane.z();

    float plane_hdgt = my_plane.psi();

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (plane_lat < sc->bb_lat_min || plane_lat > sc->bb_lat_max || RA(plane_lon - sc->bb_lon_min) < 0 ||
            RA(plane_lon - sc->bb_lon_max) > 0) {
            continue;
        }

        for (auto stand : sc->stands) {
            // heading in local system
            float local_hdgt = RA(plane_hdgt - stand->hdgt);

            if (fabs(local_hdgt) > 90.0)
                continue;  // not looking to stand

            stand->Xform2RefFrame();

            float local_x, local_z;
            stand->Global2Stand(plane_x, plane_z, local_x, local_z);

            // nose wheel
            float nw_z = local_z - my_plane.nose_gear_z_;
            float nw_x = local_x + my_plane.nose_gear_z_ * sin(kD2R * local_hdgt);

            float d = len2f(nw_x, nw_z);
            if (d > kCapZ + 50)  // fast exit
                continue;

            // LogMsg("stand: %s, z: %2.1f, x: %2.1f", stand->id, nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                // LogMsg("behind: %s",stand->id);
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / kD2R;
                // LogMsg("angle to plane: %s, %3.1f",stand->id, angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = RA(-angle - local_hdgt);

                // LogMsg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //       stand->id, nw_x, local_hdgt, rel_to_stand);

                if ((nw_x > 10.0 && rel_to_stand < -60.0) || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    // LogMsg("drive by %s",stand->id);
                    continue;
                }
            }

            // for the final comparison give azimuth a higher weight
            static constexpr float azi_weight = 4.0;
            d = len2f(azi_weight * nw_x, nw_z);

            if (d < dist) {
                // LogMsg("new min: %s, z: %2.1f, x: %2.1f",stand->id, nw_z, nw_x);
                dist = d;
                min_stand = stand;
            }
        }
    }

    if (min_stand != nullptr && min_stand != active_stand) {
        is_marshaller = 0;

        if (marshaller_inst) {
            XPLMDestroyInstance(marshaller_inst);
            marshaller_inst = nullptr;
            if (stairs_inst) {
                XPLMDestroyInstance(stairs_inst);
                stairs_inst = nullptr;
            }
        }

        LogMsg("stand: %s, %f, %f, %f, dist: %f, kDgsDist: %0.2f", min_stand->id.c_str(), min_stand->lat, min_stand->lon,
               min_stand->hdgt, dist, kDgsDist);

        active_stand = min_stand;
        dgs_assoc = false;
        assoc_dgs_z_l = -1.0E10;
        assoc_dgs_ts = 1.0E10;
        assoc_dgs_x_l = 1.0E10;
        state = ENGAGED;
    }
}

// -> changed
static bool FindDepartureStand() {
    bool changed = false;

    float plane_lat = my_plane.lat();
    float plane_lon = my_plane.lon();

    float plane_x = my_plane.x();
    float plane_z = my_plane.z();
    float plane_hdgt = my_plane.psi();

    // nose wheel
    float nw_z = plane_z - my_plane.nose_gear_z_ * cosf(kD2R * plane_hdgt);
    ;
    float nw_x = plane_x + my_plane.nose_gear_z_ * sinf(kD2R * plane_hdgt);

    Stand *ds = nullptr;
    Scenery *min_sc = nullptr;

    for (auto sc : sceneries) {
        // cheap check against bounding box
        if (plane_lat < sc->bb_lat_min || plane_lat > sc->bb_lat_max || RA(plane_lon - sc->bb_lon_min) < 0 ||
            RA(plane_lon - sc->bb_lon_max) > 0) {
            continue;
        }

        for (auto s : sc->stands) {
            if (fabsf(RA(plane_hdgt - s->hdgt)) > 3.0f)
                continue;

            s->Xform2RefFrame();

            float dx = nw_x - s->stand_x;
            float dz = nw_z - s->stand_z;
            // LogMsg("stand: %s, z: %2.1f, x: %2.1f", s->id, dz, dx);
            if (fabsf(dx * dx + dz * dz) < 1.0f) {
                ds = s;
                min_sc = sc;
                break;
            }
        }
    }

    if (ds != active_stand) {
        if (ds) {
            // create display name
            // a stand name can be anything between "1" and "Gate A 40 (Class C, Terminal 3)"
            // we try to extract the net name "A 40" in the latter case
            const std::string &dsn = ds->id;

            if (dsn.starts_with("Stand"))
                display_name = dsn.substr(6);
            else if (dsn.starts_with("Gate"))
                display_name = dsn.substr(5);
            else
                display_name = dsn;

            // delete stuff following and including a "(,;"
            if (display_name.length() > kR1Nchar) {
                const auto i = display_name.find_first_of("(,;");
                if (i != std::string::npos) {
                    display_name.resize(i);
                    display_name.erase(display_name.find_last_not_of(" ") + 1);
                }
            }

            // trim whitespace
            display_name.erase(0, display_name.find_first_not_of(" "));

            if (display_name.length() > kR1Nchar)
                display_name.clear();  // give up
            arpt_icao = min_sc->arpt_icao;
            LogMsg("departure stand is: %s/%s, display_name: '%s'", arpt_icao.c_str(), ds->id.c_str(), display_name.c_str());
        } else {
            LogMsg("No departure stand found");
        }

        active_stand = ds;
        dgs_assoc = false;
        assoc_dgs_z_l = -1.0E10;
        assoc_dgs_ts = 1.0E10;
        assoc_dgs_x_l = 1.0E10;
        changed = true;
    }

    return changed;
}

int DgsInit() {
    percent_lights_dr = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
    sin_wave_dr = XPLMFindDataRef("sim/graphics/animation/sin_wave_2");
    zulu_time_minutes_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_minutes");
    zulu_time_hours_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_hours");
    ground_speed_dr = XPLMFindDataRef("sim/flightmodel/position/groundspeed");

    // create the dgs animation datarefs
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL, NULL, DgsActiveAcc, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, NULL, NULL, (void *)(uint64_t)i, NULL);

    // these are served globally
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_m0", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m0, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_m1", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m1, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_h0", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h0, 0);
    XPLMRegisterDataAccessor("opensam/dgs/time_utc_h1", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h1, 0);
    XPLMRegisterDataAccessor("opensam/dgs/vdgs_brightness", xplmType_Float, 0, NULL, NULL, DgsGlobalAcc, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, &vdgs_brightness, 0);

    XPLMRegisterDataAccessor("sam/vdgs/status", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_STATUS, NULL);

    XPLMRegisterDataAccessor("sam/docking/lateral", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_LATERAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/longitudinal", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_LONGITUDINAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/icao", xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                             DgsSam1IcaoAcc, NULL, NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_ICAO, NULL);

    // some custom VDGS use "sam/docking/status", e.g. Gaya LOWW
    XPLMRegisterDataAccessor("sam/docking/status", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, (void *)(uint64_t)SAM1_DR_STATUS, NULL);

    marshaller_obj = XPLMLoadObject("Resources/plugins/openSAM/objects/Marshaller.obj");
    if (nullptr == marshaller_obj) {
        LogMsg("Could not load Marshaller.obj");
        return 0;
    }

    stairs_obj = XPLMLoadObject("Resources/default scenery/airport scenery/Ramp_Equipment/Stair_Maint_1.obj");
    if (nullptr == stairs_obj) {
        LogMsg("Could not load Stair_Maint_1.obj");
        return 0;
    }

    DgsSetInactive();
    return 1;
}

float DgsStateMachine() {
    static bool first = true;
    if (first) {
        first = false;
        // private datarefs are usually defined late
        ev100_dr = XPLMFindDataRef("sim/private/controls/photometric/ev100");
        if (ev100_dr)
            LogMsg("ev100 dataref mapped");
    }

    // update global dataref values

    // brightness for VDGS
    static constexpr float min_brightness = 0.025;  // relative to 1

    if (ev100_dr) {
        // if ev100 is available, we use it to set brightness
        static constexpr float kMinEv100 = 6.0f;
        static constexpr float kMaxEv100 = 11.0f;
        float ev100 = XPLMGetDataf(ev100_dr);
        ev100 = std::clamp(ev100, kMinEv100, kMaxEv100);
        const float f = (ev100 - kMinEv100) / (kMaxEv100 - kMinEv100);
        // ev100 is logarithmic and vdgs_brightness linear, so we use exp here
        const float exp_f = (std::exp(f) - 1.0f) / (std::exp(1.0f) - 1.0f);
        vdgs_brightness = min_brightness + (1.0f - min_brightness) * exp_f;
        // LogMsg("ev100: %0.2f, vdgs_brightness: %0.3f", ev100, vdgs_brightness);
    } else {
        // fallback: use percent_lights_on
        vdgs_brightness =
            min_brightness + (1.0f - min_brightness) * std::pow(1.0f - XPLMGetDataf(percent_lights_dr), 6.0f);
    }

    // UTC time digits
    int zm = XPLMGetDatai(zulu_time_minutes_dr);
    int zh = XPLMGetDatai(zulu_time_hours_dr);
    time_utc_m0 = zm % 10;
    time_utc_m1 = zm / 10;
    time_utc_h0 = zh % 10;
    time_utc_h1 = zh / 10;

    state_t prev_state = state;

    // DEPARTURE and friends ...
    if (INACTIVE <= state && state <= BOARDING) {
        // on beacon or engine or teleportation -> INACTIVE
        if (my_plane.beacon_on() || my_plane.engines_on()) {
            active_stand = nullptr;
            state = INACTIVE;
            return 2.0f;
        }

        // check for stand
        bool changed = FindDepartureStand();
        if (active_stand == nullptr) {
            state = INACTIVE;
            return 4.0f;
        }

        if (changed) {
            if (display_name.empty())
                scroll_txt = std::make_unique<ScrollTxt>(arpt_icao);
            else
                scroll_txt = std::make_unique<ScrollTxt>(arpt_icao + " STAND " + display_name + "   ");
        }

        // FALLTHROUGH
        assert(active_stand != nullptr);

        if (my_plane.pax_no() <= 0) {
            state = DEPARTURE;
            if (state != prev_state)
                LogMsg("New state %s", state_str[state]);
            // FALLTHROUGH
        }

        if (state == INACTIVE)
            return std::min(4.0f, scroll_txt->Tick());

        // cdm data may come in late during boarding
        if (state == DEPARTURE || state == BOARDING) {
            // although LoadIfNewer is cheap throttling it is even cheaper
            if (now > ofp_ts + 5.0f) {
                ofp_ts = now;
                ofp = Ofp::LoadIfNewer(ofp_seqno);  // fetch ofp
                if (ofp) {
                    ofp_seqno = ofp->seqno;
                    std::string ofp_str = ofp->GenDepartureStr();
                    LogMsg("ofp_str: '%s'", ofp_str.c_str());
                    if (display_name.empty())
                        scroll_txt = make_unique<ScrollTxt>(arpt_icao + "   " + ofp_str + "   ");
                    else
                        scroll_txt =
                            make_unique<ScrollTxt>(arpt_icao + " STAND " + display_name + "   " + ofp_str + "   ");
                }
            }
        }

        if (state == DEPARTURE) {
            if (my_plane.pax_no() > 0) {
                state = BOARDING;
                LogMsg("New state %s", state_str[state]);
                // FALLTHROUGH
            } else
                return scroll_txt->Tick();  // just scroll the text
            // FALLTHROUGH
        }

        if (state == BOARDING) {
            int pax_no = my_plane.pax_no();
            // LogMsg("boarding pax_no: %d", pax_no);
            int pn[3]{-1, -1, -1};
            for (int i = 0; i < 3; i++) {
                pn[i] = pax_no % 10;
                pax_no /= 10;
                if (pax_no == 0)
                    break;
            }

            drefs[DGS_DR_BOARDING] = 1;
            for (int i = 0; i < 3; i++)
                drefs[DGS_DR_PAXNO_0 + i] = pn[i];

            return std::min(1.0f, scroll_txt->Tick());
        }
    }

    // ARRIVAL and friends ...
    // this can be high freq stuff

    // throttle costly search
    if (now > active_stand_ts + 2.0f) {
        FindNearestStand();
        active_stand_ts = now;
    }

    if (active_stand == nullptr) {
        state = ARRIVAL;
        return 1.0;
    }

    int lr_prev = lr;
    int track_prev = track;
    float distance_prev = distance;

    float loop_delay = 0.2;
    state_t new_state = state;

    // xform plane pos into stand local coordinate system

    float local_x, local_z;
    active_stand->Global2Stand(my_plane.x(), my_plane.z(), local_x, local_z);

    // relative reading to stand +/- 180
    float local_hdgt = RA(my_plane.psi() - active_stand->hdgt);

    // nose wheel
    float nw_z = local_z - my_plane.nose_gear_z_;
    float nw_x = local_x + my_plane.nose_gear_z_ * sinf(kD2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - my_plane.main_gear_z_;
    float mw_x = local_x + my_plane.main_gear_z_ * sinf(kD2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = std::clamp((nw_z - kAziCrossover) / 20.0, 0.0, 1.0);
    float plane_ref_z = (1.0f - a) * my_plane.nose_gear_z_ + a * my_plane.main_gear_z_;
    float ref_z = local_z - plane_ref_z;
    float ref_x = local_x + plane_ref_z * sin(kD2R * local_hdgt);

    float xtrack = 0.0;  // xtrack for VDGS, set later if needed

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 5.0f)) / kD2R;
    else
        azimuth_nw = 0.0;

    bool locgood = (fabsf(mw_x) <= kGoodX && kGoodZ_m <= nw_z && nw_z <= kGoodZ_p);
    bool beacon_on = my_plane.beacon_on();

    status = lr = track = 0;
    distance = nw_z;
    bool slow = false;

    // set drefs according to *current* state
    switch (state) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance <= kCapZ) && (fabsf(azimuth_nw) <= kCapA))
                    new_state = TRACK;
            } else {  // not beacon_on
                new_state = DONE;
            }
            break;

        case TRACK: {
            if (!beacon_on) {  // don't get stuck in TRACK
                new_state = DONE;
                break;
            }

            if (locgood) {
                new_state = GOOD;
                break;
            }

            if (nw_z < kGoodZ_m) {
                new_state = BAD;
                break;
            }

            if ((distance > kCapZ) || (fabsf(azimuth_nw) > kCapA)) {
                new_state = ENGAGED;  // moving away from current gate
                break;
            }

            status = 1;  // plane id
            if (distance > kAziZ || fabsf(azimuth_nw) > kAziA) {
                track = 1;  // lead-in only
                break;
            }

            // compute distance and guidance commands

            // xform xtrack distance to values required by the OBJ
            xtrack = std::clamp(ref_x, -4.0f, 4.0f);     // in m, 4 is hardcoded in the OBJ
            xtrack = std::roundf(xtrack * 2.0f) / 2.0f;  // round to 0.5 increments

            // compute left/right command
            if (ref_z > kAziCrossover) {
                // far, aim to an intermediate point between ref point and stand
                float req_hdgt = atanf(-ref_x / (0.3f * ref_z)) / kD2R;  // required hdgt
                float d_hdgt = req_hdgt - local_hdgt;                    // degrees to turn
                if (d_hdgt < -1.5f)
                    lr = kTurnLeft;
                else if (d_hdgt > 1.5f)
                    lr = kTurnRight;
                if (now > update_dgs_log_ts + 2.0f)
                    LogMsg(
                        "req_hdgt: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), "
                        "ref: (%0.1f, %0.1f), "
                        "x: %0.1f, ",
                        req_hdgt, local_hdgt, d_hdgt, mw_x, mw_z, nw_x, nw_z, ref_x, ref_z, local_x);

            } else {
                // close, use xtrack
                if (ref_x < -0.25f)
                    lr = kTurnRight;
                else if (ref_x > 0.25f)
                    lr = kTurnLeft;
            }

            // decide whether to show the SLOW indication
            // depends on distance and ground speed
            float gs = XPLMGetDataf(ground_speed_dr);
            slow = (distance > 20.0f && gs > 4.0f) || (10.0f < distance && distance <= 20.0f && gs > 3.0f) ||
                   (distance <= 10.0f && gs > 2.0f);

            if (distance <= kCrZ / 2) {
                track = 3;
                loop_delay = 0.03;
            } else  // azimuth only
                track = 2;

            // For the Marshaller sync change of straight ahead / turn commands with arm position
            if (is_marshaller) {
                // catch the phase ~180° point -> the Marshaller's arm is straight
                float sin_wave = XPLMGetDataf(sin_wave_dr);
                bool phase180 = (sin_wave_prev > 0.0) && (sin_wave <= 0.0);
                sin_wave_prev = sin_wave;

                if (!phase180) {
                    lr = lr_prev;
                    // sync transition with Marshaller's arm movement
                    if (track == 3 && track_prev == 2) {
                        track = track_prev;
                        distance = distance_prev;
                    }
                }
            }
        } break;

        case GOOD: {
            // @stop position*/
            status = 2;
            lr = 3;

            int parkbrake_set = my_plane.parkbrake_set();
            if (!locgood)
                new_state = TRACK;
            else if (parkbrake_set || !beacon_on)
                new_state = PARKED;
        } break;

        case BAD:
            if (!beacon_on && (now > timestamp + 5.0f)) {
                DgsSetInactive();
                return loop_delay;
            }

            if (nw_z >= kGoodZ_m)  // moving backwards
                new_state = TRACK;
            else {
                // Too far
                status = 4;
                lr = 3;
            }
            break;

        case PARKED:
            status = 3;
            lr = 0;
            // wait for beacon off
            if (!beacon_on) {
                new_state = DONE;
                if (!my_plane.dont_connect_jetway_) {  // check whether it's a ToLiss, then set chocks
                    XPLMDataRef tls_chocks = XPLMFindDataRef("AirbusFBW/Chocks");
                    if (tls_chocks) {
                        XPLMSetDatai(tls_chocks, 1);
                        if (!is_marshaller)
                            new_state = CHOCKS;
                    }
                }
            }
            break;

        case CHOCKS:
            status = 6;
            if (now > timestamp + 5.0)
                new_state = DONE;
            break;

        case DONE:
            if (now > timestamp + 3.0f) {
                if (!my_plane.dont_connect_jetway_)  // wait some seconds for the jw handler to catch up
                    my_plane.RequestDock();

                DgsSetInactive();
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state) {
        LogMsg("dgs state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        timestamp = now;
        return -1;  // see you on next frame
    }

    if (state > ARRIVAL) {
        // xform drefs into required constraints for the OBJs
        if (track == 0 || track == 1) {
            distance = 0;
            xtrack = 0.0;
        }

        distance = std::clamp(distance, kGoodZ_m, kCrZ);
        int d_0 = 0;
        int d_01 = 0;
        // according to Safegate_SDK_UG_Pilots_v1.10_s.pdf
        // > 3m: 1.0 m decrements, <= 3m 0.2m decrements
        if (0.0f <= distance && distance < 10.0f) {
            d_0 = distance;
            if (d_0 < 3) {
                int d = (distance - d_0) * 10.0f;
                d &= ~1;  // make it even = 0.2m increments
                d_01 = d;
            }
        }

        if (!is_marshaller)
            distance = ((float)((int)((distance) * 2))) / 2;  // multiple of 0.5m

        memset(drefs, 0, sizeof(drefs));
        drefs[DGS_DR_STATUS] = status;
        drefs[DGS_DR_TRACK] = track;
        drefs[DGS_DR_DISTANCE] = distance;
        drefs[DGS_DR_DISTANCE_0] = d_0;
        drefs[DGS_DR_DISTANCE_01] = d_01;
        drefs[DGS_DR_XTRACK] = xtrack;
        drefs[DGS_DR_LR] = lr;

        if (slow) {
            drefs[DGS_DR_ICAO_0] = 'S';
            drefs[DGS_DR_ICAO_1] = 'L';
            drefs[DGS_DR_ICAO_2] = 'O';
            drefs[DGS_DR_ICAO_3] = 'W';
        } else {
            const char *icao = my_plane.icao().c_str();
            for (int i = 0; i < 4; i++)
                drefs[DGS_DR_ICAO_0 + i] = icao[i];
        }

        // translate into compatible SAM1 values
        sam1_lateral = -ref_x;
        sam1_longitudinal = std::min(ref_z, 30.0f);

        switch (state) {
            case ENGAGED:
            case TRACK:
                sam1_status = SAM1_TRACK;
                break;

            case GOOD:
            case PARKED:
                if (sam1_longitudinal < 0.1f) {
                    sam1_status = SAM1_STOP_ZONE;
                    sam1_lateral = 0.0f;
                } else
                    sam1_status = SAM1_TRACK;
                break;

            case BAD:
                sam1_status = SAM1_TRACK;
                break;

            case CHOCKS:
            case DONE:
                sam1_status = SAM1_IDLE;
                sam1_longitudinal = 0.0;
                break;

            default:
                sam1_status = SAM1_IDLE;
                sam1_lateral = SAM1_LATERAL_OFF;
                sam1_longitudinal = 0.0f;
        }

        if (is_marshaller && BETWEEN(state, ENGAGED, PARKED)) {
            XPLMDrawInfo_t drawinfo = {};
            drawinfo.structSize = sizeof(XPLMDrawInfo_t);
            drawinfo.heading = marshaller_psi;
            drawinfo.pitch = drawinfo.roll = 0.0;

            if (nullptr == marshaller_inst) {
                LogMsg("place marshaller at %0.2f, %0.2f, %0.2f, hdg: %0.1f°", marshaller_x, marshaller_y, marshaller_z,
                       marshaller_psi);

                marshaller_inst = XPLMCreateInstance(marshaller_obj, dgs_dlist_dr);
                if (marshaller_inst == nullptr) {
                    LogMsg("error creating marshaller instance");
                    state = DISABLED;
                    return 0.0;
                }

                // now check whether it's Marshaller_high

                if (xplm_ProbeHitTerrain ==
                    XPLMProbeTerrainXYZ(probe_ref, marshaller_x, marshaller_y, marshaller_z, &probeinfo)) {
                    marshaller_y_0 = probeinfo.locationY;  // ground 0

                    if (marshaller_y - marshaller_y_0 > 2.0f) {
                        LogMsg("Marshaller_high detected, place stairs");
                        static const char *null[] = {nullptr};
                        stairs_inst = XPLMCreateInstance(stairs_obj, null);
                        if (stairs_inst == nullptr) {
                            LogMsg("error creating stairs instance");
                            state = DISABLED;
                            return 0.0;
                        }

                        // move slightly to the plane
                        static constexpr float delta_z = 1.0f;
                        drawinfo.x = marshaller_x - delta_z * active_stand->sin_hdgt;
                        drawinfo.y = marshaller_y_0;
                        drawinfo.z = marshaller_z + delta_z * active_stand->cos_hdgt;
                        XPLMInstanceSetPosition(stairs_inst, &drawinfo, NULL);
                    }
                }
            }

            // update datarefs
            drawinfo.x = marshaller_x;
            drawinfo.y = marshaller_y;
            drawinfo.z = marshaller_z;
            XPLMInstanceSetPosition(marshaller_inst, &drawinfo, drefs);
        }

        // don't flood the log
        if (now > update_dgs_log_ts + 2.0f) {
            update_dgs_log_ts = now;
            LogMsg(
                "stand: %s, state: %s, assoc: %d, is_marshaller: %d, track: %d, lr: %d, distance: %0.2f, xtrack: %0.1f",
                active_stand->id.c_str(), state_str[state], dgs_assoc, is_marshaller, track, lr, distance, xtrack);
            LogMsg("sam1: status %0.0f, lateral: %0.1f, longitudinal: %0.1f", sam1_status, sam1_lateral,
                   sam1_longitudinal);
        }
    }

    return loop_delay;
}
