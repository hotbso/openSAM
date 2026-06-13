//
//    openSAM: open source SAM emulator for X Plane
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

#include <cmath>
#include <unordered_map>

#include "XPLMGraphics.h"

#include "opensam.h"
#include "opensam_airport.h"
#include "my_plane.h"

#include "dgs/dgs.h"
#include "sam1_dgs.h"
#include "log_msg.h"

#include "dgs_variants_generated.h"

#include "flat_earth_math.h"
namespace fem = flat_earth_math;

static constexpr float kD2R = std::numbers::pi/180.0;

static constexpr float kMaxDgs2StandX = 10.0f;  // max offset/distance from DGS to stand
static constexpr float kMaxDgs2StandZ = 80.0f;
static constexpr float kMaxDgsHdgtDiff = 10.0f;  // max heading difference between DGS and stand
static constexpr float kExtraDz = 0.05f;  // extra pull forward of displays in z direction to account for precision loss

std::unique_ptr<OsAirport> os_arpt;

OsStand::OsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation)
    : dgs::Stand(as, arpt_icao, elevation) {

    // LogMsg("OsStand '%s', is_wet: %d, type: %d, dgs_dist: %0.1f constructed", cname(),
    //      is_wet_ ? 1 : 0, dgs_type_, dgs_dist_);
}

OsStand::~OsStand() {
    LogMsg("OsStand '%s' destructed", cname());
}

void OsStand::InstallDgs(int dgs_type, const DgsCtx& ctx) {
    float psi = (ctx.turn_180) ? fem::RA(ctx.obj_psi + 180.0f) : ctx.obj_psi;  // SAM1 legacy DGS can be turned 180°
    dgs_lat_ = ctx.lat;
    dgs_lon_ = ctx.lon;
    dgs_altitude_ = ctx.altitude;
    height_ = ctx.height;
    dgs_psi_ = psi;
    drawinfo_.x = ctx.obj_x;
    drawinfo_.y = ctx.obj_y;
    drawinfo_.z = ctx.obj_z;
    drawinfo_.heading = psi;

    // account for precision loss as obj_x/y/z is only float
    float dx = -kExtraDz * std::sin(psi * kD2R);
    float dz =  kExtraDz * std::cos(psi * kD2R);
    drawinfo_.x += dx;
    drawinfo_.z += dz;

    switch (dgs_type) {
        case kDgsType_Marshaller:
            dgs_ = dgs::CreateMarshaller(name());
            break;
        case kDgsType_Safedock_T2_24:
            dgs_ = dgs::CreateSafedock_T2_24(name(), arpt_icao_, height_, true /*display_only*/);
            break;
        case kDgsType_Safedock_X:
            dgs_ = dgs::CreateSafedock_X(name(), arpt_icao_, height_, true /*display_only*/);
            break;
        case kDgsType_SAM1_Legacy:
            dgs_ = dgs::CreateSam1Legacy(name());
            break;
        default:
            LogMsg("Unknown DGS type %d, not creating DGS instance", dgs_type);
    }

    if (dgs_ != nullptr) {
        dgs_->SetPos(drawinfo_);
        SetIdle();
        dgs_position_set_ = true;
    }
}

bool OsStand::has_jw() const {
    return true;  // if there is none nothing bad will happen
}

//--------------------- OsAirport --------------------------------------------------------------
OsAirport::OsAirport(const dgs::AptAirport& apt_airport) : dgs::Airport(apt_airport) {
   float arpt_elevation = XPLMGetDataf(plane_elevation_dr);  // best guess

    for (auto const& as : apt_airport.stands_) {
        stands_.emplace_back(std::make_unique<OsStand>(as, name_, arpt_elevation));
    }

    dgs_cache_.reserve(stands_.size() * 2);  // reserve some space for the DGS cache to avoid rehashing etc.
    pending_dgs_.reserve(stands_.size());
    LogMsg("OsAirport '%s' with %d stands constructed", name().c_str(), (int)stands_.size());
}

OsAirport::~OsAirport() {
    LogMsg("OsAirport '%s' destructed", name().c_str());
}

std::unique_ptr<OsAirport> OsAirport::LoadAirport(const dgs::AptAirport *arpt) {
    if (arpt == nullptr)
        return nullptr;

    return std::make_unique<OsAirport>(*arpt);
}

void OsAirport::ConnectJetway() {
    my_plane->RequestDock();
}

const OsStand* OsAirport::FindStandForJw(float jw_x, float jw_z) {
    const OsStand* min_stand = nullptr;
    float min_dist = std::numeric_limits<float>::max();

    for (auto& stand : stands_) {
        float x_sl, z_sl;
        stand->Local2Stand(jw_x, jw_z, x_sl, z_sl);
        if (x_sl > 2.0f)  // on the right
            continue;

        float dist = std::hypot(x_sl, z_sl);
        if (dist < min_dist) {
            min_dist = dist;
            min_stand = dynamic_cast<const OsStand*>(stand.get());
        }
    }

    return min_stand;
}

int OsAirport::FindStandForObj(const DgsCtx& ctx) {
    // find a stand for an DGS object with the given local coordinates and heading in the current ref frame,
    // return index in stands_ or -1 if not found

    int imin = -1;
    float min_x = 1.0e10;
    float max_z = -1.0e10;

    for (int i = 0; i < (int)stands_.size(); ++i) {
        OsStand* stand = dynamic_cast<OsStand*>(stands_[i].get());

        // to stand local
        float obj_x_sl, obj_z_sl;
        stand->Local2Stand(ctx.obj_x, ctx.obj_z, obj_x_sl, obj_z_sl);

        // must be in a box +- kMaxDgs2StandX, kMaxDgs2StandZ
        // and reasonably aligned with stand (or for SAM1 anti aligned)
        float dh = std::abs(fem::RA(stand->hdgt() - ctx.obj_psi));
        bool aligned;
        aligned = dh < kMaxDgsHdgtDiff;
        if (ctx.dgs_type == kDgsType_SAM1_Legacy)
            aligned = (dh < kMaxDgsHdgtDiff || dh > (180.0f - kMaxDgsHdgtDiff));  // SAM1 legacy DGS can be turned 180°
        else if (ctx.turn_180)  // we know it's anti aligned
            aligned = dh > (180.0f - kMaxDgsHdgtDiff);
        else
            aligned = dh < kMaxDgsHdgtDiff;

        if (obj_z_sl < -kMaxDgs2StandZ || obj_z_sl > -5.0f || std::abs(obj_x_sl) > kMaxDgs2StandX || !aligned) [[likely]]
            continue;

        // closest to center line, nearest to stand, closer to cl takes precedence
        if (((std::abs(obj_x_sl) < min_x - 2.0f) && (obj_z_sl > max_z - 8.0f)) ||
            ((std::abs(obj_x_sl) < min_x - 1.0f) && (obj_z_sl > max_z - 2.0f)) ||
            (((std::abs(obj_x_sl) < min_x) && (obj_z_sl > max_z)))) {
            min_x = std::abs(obj_x_sl);
            max_z = obj_z_sl;
            imin = i;
        }
    }

    return imin;
}

void OsAirport::ProcessPendingDgs(DgsCtx& ctx) {
    // process a newly identified DGS, e.g. by instancing it and associating it with a stand
    // LogMsg("Processing pending DGS: type %d, height %.1f, turn_180 %d, obj_x %.2f, obj_y %.2f, obj_z %.2f, obj_psi "
    //    "%.1f, lat %.6f, lon %.6f, altitude %.2f",
    //    ctx.dgs_type, ctx.height, ctx.turn_180 ? 1 : 0, ctx.obj_x, ctx.obj_y, ctx.obj_z, ctx.obj_psi, ctx.lat,
    //    ctx.lon, ctx.altitude);

    if (ctx.ref_gen != ref_gen_) {
        // we must retransform
        double x, y, z;
        XPLMWorldToLocal(ctx.lat, ctx.lon, ctx.altitude, &x, &y, &z);

        ctx.obj_x = (float)x;
        ctx.obj_y = (float)y;
        ctx.obj_z = (float)z;
        ctx.ref_gen = ref_gen_;
        LogMsg("retransformed DGS to new ref frame: obj_x %.2f, obj_y %.2f, obj_z %.2f", ctx.obj_x, ctx.obj_y, ctx.obj_z);
    }

    int istand = FindStandForObj(ctx);

    if (istand < 0) {
        LogMsg("No stand found for DGS at lat %.6f, lon %.6f, altitude %.2f", ctx.lat, ctx.lon, ctx.altitude);
        return;
    }

    // TODO: check whether it isw already associated, reidentify after ref frame shift etc.
    LogMsg("Associating DGS of type %d to stand '%s'", ctx.dgs_type, stands_[istand]->cname());
    OsStand* s = dynamic_cast<OsStand*>(stands_[istand].get());
    if (s->dgs_position_set_) {
        LogMsg("Stand '%s' already has a DGS associated, skipping", s->cname());
        return;
    }
    s->InstallDgs(ctx.dgs_type, ctx);
}

bool OsAirport::auto_post_parkbrake() const {
     return true; //operation_mode == MODE_AUTO && !dgs::plane->dont_connect_jetway;
}

// SAM1 drefs
enum _SAM1_DREF {
    SAM1_DR_STATUS,
    SAM1_DR_LATERAL,
    SAM1_DR_LONGITUDINAL,
    SAM1_DR_ICAO,
};

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

    // after tracking clear icao drefs
    if (dgs::sam1_drefs.is_active && ((dgs::sam1_drefs.status > dgs::SAM1_TRACK) || (dgs::sam1_drefs.longitudinal < 0.2f))) {
        for (int i = 0; i < n; i++)
            values[i] = 0;
        return n;
    }

    const char *icao = my_plane->icao().c_str();
    for (int i = 0; i < n; i++) {
        char c = icao[ofs + i];
        if (isalpha(c))
            values[i] = (c - 'A') + 1;
        else
            values[i] = (c - '0') + 27;
    }

    return n;
}

// Dataref accessor for the dgs/ident/* datarefs
// Check whether the DGS with the given local coordinates and heading in the current ref frame has already been
// identified, if not add it to the pending list to be processed in flight loop context for instancing etc. We need to
// defer this processing because we are not allowed to call XPLMInstance() in the draw loop context.
float OsAirport::DgsIdentAcc(void* ref) {
    kDgsVariant dgs_var = (kDgsVariant)(uint64_t)ref;

    if (os_arpt == nullptr || dgs_var >= kDgsVarCount)
        return 0.0f;

    float obj_x = XPLMGetDataf(draw_object_x_dr);
    float obj_y = XPLMGetDataf(draw_object_y_dr);
    float obj_z = XPLMGetDataf(draw_object_z_dr);
    float obj_psi = XPLMGetDataf(draw_object_psi_dr);

    if (obj_x == 0.0f || obj_z == 0.0f || obj_psi == 0.0f)
        return 0.0f;  // likely uninitialized, datareftool poll etc.

    //LogMsg("DgsIdentAcc called for variant %d, obj_x %.2f, obj_y %.2f, obj_z %.2f, obj_psi %.1f", dgs_var, obj_x, obj_y, obj_z, obj_psi);
    if (ref_gen != os_arpt->dgs_cache_ref_gen_) {
        os_arpt->dgs_cache_.clear();
        os_arpt->dgs_cache_ref_gen_ = ref_gen;
    }

    if (os_arpt->dgs_cache_.contains({obj_x, obj_z}))
        return 0.0f;  // already seen this one

    DgsCtx ctx;
    GetDgsVariantParams(dgs_var, ctx.dgs_type, ctx.height, ctx.turn_180);
    ctx.ref_gen = ref_gen;
    ctx.obj_x = obj_x;
    ctx.obj_y = obj_y;
    ctx.obj_z = obj_z;
    ctx.obj_psi = obj_psi;
    XPLMLocalToWorld(obj_x, obj_y, obj_z, &ctx.lat, &ctx.lon, &ctx.altitude);

    PositionCacheKey key{obj_x, obj_z};
    os_arpt->dgs_cache_[key] = true;
    os_arpt->pending_dgs_.push_back(ctx);
    //LogMsg("new DGS identified: type %d, height %.1f, turn_180 %d, obj_x %.2f, obj_y %.2f, obj_z %.2f, obj_psi %.1f", ctx.dgs_type,
    //       ctx.height, ctx.turn_180 ? 1 : 0, ctx.obj_x, ctx.obj_y, ctx.obj_z, ctx.obj_psi);
    return 0.0f;
}

//
// Accessor for the "sam/..." dgs related datarefs
//
// This function is called from draw loops, efficient coding required.
//
// Feed SAM1 legacy DGS
float OsAirport::DgsSam1Acc(void* ref) {
    int dr_index = (uint64_t)ref;

    if (os_arpt == nullptr)
        goto inactive;
    {
        float obj_x = XPLMGetDataf(draw_object_x_dr);
        float obj_z = XPLMGetDataf(draw_object_z_dr);
        float obj_y = XPLMGetDataf(draw_object_y_dr);
        float obj_psi = XPLMGetDataf(draw_object_psi_dr);

        if (obj_x == 0.0f || obj_z == 0.0f || obj_psi == 0.0f)
            return 0.0f;  // likely uninitialized, datareftool poll etc.

        // the active one
        if (dgs::sam1_drefs.is_active && std::fabs(obj_x - dgs::sam1_drefs.dgs_x) < 0.1f &&
            std::fabs(obj_z - dgs::sam1_drefs.dgs_z) < 0.1f) {
            switch (dr_index) {
                case SAM1_DR_STATUS:
                    return dgs::sam1_drefs.status;
                case SAM1_DR_LATERAL:
                    return dgs::sam1_drefs.lateral;
                case SAM1_DR_LONGITUDINAL:
                    return dgs::sam1_drefs.longitudinal;
                default:
                    // NOTREACHED, hopefully
                    return 0.0f;
            }
        }

        // not the active one, check if it's a new DGS to activate, only check status dref
        if (dr_index == SAM1_DR_STATUS) {
            if (ref_gen != os_arpt->dgs_cache_ref_gen_) {
                os_arpt->dgs_cache_.clear();
                os_arpt->dgs_cache_ref_gen_ = ref_gen;
            }

            if (os_arpt->dgs_cache_.contains({obj_x, obj_z}))
                goto inactive;  // already seen this one, but not active, so inactive

            DgsCtx ctx;
            ctx.dgs_type = kDgsType_SAM1_Legacy;
            ctx.height = 0.0f;     // SAM1 legacy DGS don't have height variants
            ctx.turn_180 = false;  // SAM1 legacy DGS don't have turn_180 variants
            ctx.ref_gen = ref_gen;
            ctx.obj_x = obj_x;
            ctx.obj_y = obj_y;
            ctx.obj_z = obj_z;
            ctx.obj_psi = obj_psi;
            XPLMLocalToWorld(obj_x, obj_y, obj_z, &ctx.lat, &ctx.lon, &ctx.altitude);

            PositionCacheKey key{obj_x, obj_z};
            os_arpt->dgs_cache_[key] = true;
            os_arpt->pending_dgs_.push_back(ctx);
            // see you created in the near future, until then, let's be inactive
            // FALLTHROUGH
        }
    }

inactive:
    switch (dr_index) {
        case SAM1_DR_STATUS:
            return dgs::SAM1_IDLE;
        case SAM1_DR_LATERAL:
            return dgs::SAM1_LATERAL_OFF;  // switch off VDGS
        case SAM1_DR_LONGITUDINAL:
            return 0.0f;
    }

    // NOTREACHED, hopefully
    return 0.0f;
}

float OsAirport::StateMachine() {
    float delay = 1.0f;

    // process pending DGS identified in draw loop context
    if (pending_dgs_.size() > 0) {
        LogMsg("Processing %d pending DGS", (int)pending_dgs_.size());

        for (auto& ctx : pending_dgs_)
            os_arpt->ProcessPendingDgs(ctx);

        pending_dgs_.clear();
    }

    try {
        delay = std::min(delay, dgs::Airport::StateMachine());
    } catch (const std::exception& e) {
        LogMsg("Exception in Airport::StateMachine: %s", e.what());
        error_disabled = true;
        return 0.0f;
    }

    return delay;
}

//------------------------------------------------------------------------------------
void OsAirport::Init() {
    for (int i = 0; i < kDgsVarCount; ++i) {
        XPLMRegisterDataAccessor(dgs_variant_drefs[i], xplmType_Float, 0, NULL, NULL, DgsIdentAcc, NULL, NULL, NULL,
                                 NULL, NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)i, NULL);
    }

    // SAM1 related datarefs
    XPLMRegisterDataAccessor("sam/vdgs/status", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, (void*)(uint64_t)SAM1_DR_STATUS, NULL);

    // some custom VDGS use "sam/docking/status", e.g. Gaya LOWW
    XPLMRegisterDataAccessor("sam/docking/status", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)SAM1_DR_STATUS, NULL);

    XPLMRegisterDataAccessor("sam/docking/lateral", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)SAM1_DR_LATERAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/longitudinal", xplmType_Float, 0, NULL, NULL, DgsSam1Acc, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)SAM1_DR_LONGITUDINAL, NULL);

    XPLMRegisterDataAccessor("sam/docking/icao", xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                             DgsSam1IcaoAcc, NULL, NULL, NULL, NULL, NULL, (void*)(uint64_t)SAM1_DR_ICAO, NULL);

    return;
}
