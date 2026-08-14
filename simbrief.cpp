//
//    openSAM: Manage DGS
//
//    Copyright (C) 2023, 2025, 2026 Holger Teutsch
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

#include "simbrief.h"

#include <cassert>
#include <ctime>
#include <cstring>
#include <print>

#include "XPLMDataAccess.h"

#include "log_msg.h"

Ofp Ofp::ofp;
bool Ofp::sbh_avail = false;

static bool init_done;

#define DEF_OFP_DR(f) static XPLMDataRef f ## _dr;
#define DEF_CDM_DR(f) static XPLMDataRef cdm_ ## f ## _dr;
DEF_OFP_DR(icao_airline);
DEF_OFP_DR(flight_number);
DEF_OFP_DR(aircraft_icao);
DEF_OFP_DR(destination);
DEF_OFP_DR(pax_count);
DEF_OFP_DR(freight);
DEF_OFP_DR(fuel_plan_ramp);
DEF_OFP_DR(est_out);
DEF_OFP_DR(est_off);
DEF_OFP_DR(est_on);
DEF_OFP_DR(est_in);
DEF_OFP_DR(dx_rmk);

DEF_CDM_DR(tobt);
DEF_CDM_DR(tsat);
DEF_CDM_DR(ctot);
DEF_CDM_DR(ttot);
DEF_CDM_DR(runway);
DEF_CDM_DR(sid);

#undef DEF_OFP_DR
#undef DEF_CDM_DR

static XPLMDataRef seqno_dr, cdm_seqno_dr, stale_dr;
static int sbh_ofp_seqno, sbh_cdm_seqno, my_seqno;

// fetch byte data into a string
static void FetchDref(std::string& str, XPLMDataRef dr) {
    str.clear();
    if (dr == nullptr)
        return;

    auto n = XPLMGetDatab(dr, nullptr, 0, 0);
    if (n == 0)
        return;

    str.resize(n + 1);  // ensure we always have a trailing 0
    [[maybe_unused]] auto n1 = XPLMGetDatab(dr, (void*)str.data(), 0, n);
    assert(n == n1);
    // in case a 0-terminated string was returned make it a compatible std::string
    str.resize(strlen(str.c_str()));
}

#define FIND_OFP_DREF(f) f##_dr = XPLMFindDataRef("sbh/" #f)
#define FIND_CDM_DREF(f)  cdm_ ## f ## _dr = XPLMFindDataRef("sbh/cdm/" #f)

#define GET_OFP_DREF(f) FetchDref(Ofp::ofp.f, f ## _dr)
#define GET_CDM_DREF(f) FetchDref(Ofp::ofp.cdm_ ## f, cdm_ ## f ## _dr)
#define LOG_FIELD(f) LogMsg(" " #f ": '%s'", Ofp::ofp.f.c_str())

void Ofp::Initialize() {
    sbh_avail = false;
    if (!init_done) {
        init_done = true;

        stale_dr = XPLMFindDataRef("sbh/stale");
        if (stale_dr == nullptr) {
            LogMsg("simbrief_hub plugin is not loaded, bye!");
            return;
        }

        seqno_dr = XPLMFindDataRef("sbh/seqno");

        FIND_OFP_DREF(icao_airline);
        FIND_OFP_DREF(flight_number);
        FIND_OFP_DREF(aircraft_icao);
        FIND_OFP_DREF(destination);
        FIND_OFP_DREF(pax_count);
        FIND_OFP_DREF(freight);
        FIND_OFP_DREF(fuel_plan_ramp);
        FIND_OFP_DREF(est_out);
        FIND_OFP_DREF(est_off);
        FIND_OFP_DREF(est_on);
        FIND_OFP_DREF(est_in);
        FIND_OFP_DREF(dx_rmk);

        FIND_CDM_DREF(seqno);
        FIND_CDM_DREF(tobt);
        FIND_CDM_DREF(tsat);
        FIND_CDM_DREF(ctot);
        FIND_CDM_DREF(runway);
        FIND_CDM_DREF(sid);
        sbh_avail = true;
    }
}

bool Ofp::LoadIfNewer() {
    if (!sbh_avail)
        return false;


    int ofp_seqno = XPLMGetDatai(seqno_dr);
    int cdm_seqno = XPLMGetDatai(cdm_seqno_dr);
    if (ofp_seqno == sbh_ofp_seqno && cdm_seqno == sbh_cdm_seqno)
        return false;

    sbh_ofp_seqno = ofp_seqno;
    sbh_cdm_seqno = cdm_seqno;
    my_seqno++;

    int stale = XPLMGetDatai(stale_dr);
    if (stale)
        LogMsg("simbrief_hub data may be stale");

    ofp.stale = stale != 0;
    ofp.seqno = my_seqno;
    GET_OFP_DREF(icao_airline);
    GET_OFP_DREF(flight_number);
    GET_OFP_DREF(aircraft_icao);
    GET_OFP_DREF(destination);
    GET_OFP_DREF(pax_count);
    GET_OFP_DREF(freight);
    GET_OFP_DREF(fuel_plan_ramp);
    GET_OFP_DREF(est_out);
    GET_OFP_DREF(est_off);
    GET_OFP_DREF(est_on);
    GET_OFP_DREF(est_in);
    GET_OFP_DREF(dx_rmk);
    GET_CDM_DREF(tobt);
    GET_CDM_DREF(tsat);
    GET_CDM_DREF(ctot);
    GET_CDM_DREF(ttot);
    GET_CDM_DREF(runway);
    GET_CDM_DREF(sid);

    ofp.callsign = ofp.icao_airline + ofp.flight_number;

    auto cnv_hhmm = [](const std::string& t) -> std::string {
        if (t.empty())
            return "";
        time_t tt = std::atol(t.c_str());
        auto tm = *std::gmtime(&tt);
        return std::format("{:02d}{:02d}", tm.tm_hour, tm.tm_min);
    };

    ofp.est_out_str = cnv_hhmm(ofp.est_out);
    ofp.est_off_str = cnv_hhmm(ofp.est_off);

    LogMsg("From simbrief_hub: Seqno: %d, Cdm: %d", ofp_seqno, cdm_seqno);
    LOG_FIELD(icao_airline);
    LOG_FIELD(flight_number);
    LOG_FIELD(aircraft_icao);
    LOG_FIELD(destination);
    LOG_FIELD(pax_count);
    LOG_FIELD(freight);
    LOG_FIELD(fuel_plan_ramp);
    LOG_FIELD(est_out);
    LOG_FIELD(est_off);
    LOG_FIELD(est_on);
    LOG_FIELD(est_in);
    LOG_FIELD(dx_rmk);
    LOG_FIELD(callsign);
    LOG_FIELD(est_out_str);
    LOG_FIELD(est_off_str);
    LOG_FIELD(cdm_tobt);
    LOG_FIELD(cdm_tsat);
    LOG_FIELD(cdm_ctot);
    LOG_FIELD(cdm_ttot);
    LOG_FIELD(cdm_runway);
    LOG_FIELD(cdm_sid);

    return true;
}

const std::string Ofp::GenDepartureStr() const {
    std::string str = callsign + " " + aircraft_icao + " TO " + destination;

    bool have_cdm = !cdm_tobt.empty();
    if (!have_cdm) {
       str.append(" OUT " + est_out_str);
    } else {
        if (cdm_tsat != cdm_tobt)
            str.append(" TOBT " + cdm_tobt);

        if (!cdm_tsat.empty())
            str.append(" TSAT " + cdm_tsat);

        if (!cdm_ttot.empty())
            str.append(" TTOT " + cdm_ttot);

        else if (!cdm_ctot.empty())
            str.append(" CTOT " + cdm_ctot);
    }

    if (!have_cdm)
       str.append(" OFF " + est_off_str);

    if (!cdm_runway.empty())
        str.append(" RWY " + cdm_runway);

    if (!cdm_sid.empty())
        str.append(" SID " + cdm_sid);

    return str;
}
