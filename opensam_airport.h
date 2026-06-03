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

#ifndef _OS_AIRPORT_H_
#define _OS_AIRPORT_H_

#include <memory>
#include <unordered_map>

#include "dgs/airport.h"

enum DgsType {
    kDgsType_Marshaller,
    kDgsType_Safedock_T2_24,
    kDgsType_Safedock_X,
    kDgsType_SAM1_Legacy
};

struct DgsCtx {
    unsigned int dgs_type;

    unsigned int ref_gen;                // validity of obj_* data
    float obj_x, obj_y, obj_z, obj_psi;  // local coordinates of the DGS

    float height;   // height of the DGS above ground
    bool turn_180;  // whether the DGS is turned 180° (for SAM1 compatibility)

    // world coordinates of the DGS, in case we ref frame shift and need to re-identify it
    double lat, lon, altitude;
};

class OsAirport;  // forward declaration for dgs::Stand

// dgs::Stand augmented for openSAM
class OsStand : public dgs::Stand {
   protected:
    friend class OsAirport;

    float dgs_psi_;  // orientation of DGS, from scenery object
    float height_;   // height of the DGS above ground, from the variant parameters

   public:
    OsStand(OsStand&&) = default;
    OsStand& operator=(OsStand&&) = delete;

    OsStand(const dgs::AptStand& as, const std::string& arpt_icao, float elevation);
    ~OsStand();

    bool has_jw() const override;  // whether the stand has a jetway, used for auto connect logic
    void InstallDgs(int dgs_type, const DgsCtx& ctx);
};

// dgs::Airport augmented for openSAM
class OsAirport : public dgs::Airport {
    // the cache is used to quickly check whether a DGS at a given position has already been seen
    // it is filled on the fly when we encounter a DGS for the first time in a draw loop and cleared when the ref
    // frame changes
    std::unordered_map<PositionCacheKey, bool, PositionCacheKeyHasher> dgs_cache_;
    unsigned int dgs_cache_ref_gen_ = 0;                                // to invalidate cache when ref frame changes

    // pending_dgs is filled in the draw loop with newly identified DGS to be processed in flight loop context for
    // instancing etc. We need to defer this processing because we are not allowed to call XPLMInstanceSetPosition() in
    // the draw loop context.
    std::vector<DgsCtx> pending_dgs_;

    void ProcessPendingDgs(DgsCtx& ctx);  // process a newly identified DGS, e.g. by instancing it and associating it with a stand
    int FindStandForObj(float obj_x, float obj_z, float obj_psi); // returns index in stands_ or -1 if not found

   public:
    static std::unique_ptr<OsAirport> LoadAirport(const dgs::AptAirport* arpt);

    static void Init();
    static float DgsIdentAcc(void* ref);  // dataref accessor for the dgs/ident/* datarefs
    static float DgsSam1Acc(void* ref);   // dataref accessor for the "sam/..." dgs related datarefs

    OsAirport() = delete;
    OsAirport(const dgs::AptAirport&);
    ~OsAirport();

    float StateMachine(); // augment dgs::Airport's state machine with openSAM specific logic

    void ResetState(state_t new_state);

    size_t n_stands() const { return stands_.size(); }
    const OsStand* FindStandForJw(float jw_x, float jw_z);

    // auto set chocks and connect jetway when parking ?
    bool auto_post_parkbrake() const override;
    void ConnectJetway() override;
};

extern std::unique_ptr<OsAirport> os_arpt;
#endif
