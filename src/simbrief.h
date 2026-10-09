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

#pragma once

#include <string>

struct Ofp {
   private:
    static bool sbh_avail;

   public:
    int seqno{0};       // incremented after each successfull fetch, 0 == empty / no data
    bool stale{false};  // true if the data may be stale (e.g. was valid but now simbrief not responding)

    std::string icao_airline;
    std::string flight_number;
    std::string aircraft_icao;
    std::string origin;
    std::string destination;
    std::string pax_count;
    std::string freight;
    std::string fuel_plan_ramp;
    std::string est_out;
    std::string est_off;
    std::string est_on;
    std::string est_in;
    std::string dx_rmk;

    // cdm fields
    std::string cdm_tobt;
    std::string cdm_tsat;
    std::string cdm_ctot;
    std::string cdm_ttot;
    std::string cdm_runway;
    std::string cdm_sid;

    // for convenience, not part of the simbrief_hub data
    std::string callsign;
    std::string est_out_str;  // hhmm representation of est_out
    std::string est_off_str;  // hhmm representation of est_off

    // update the ofp data from the simbrief_hub datarefs, if they have changed since the last call
    // returns true if the ofp data has changed
    static bool LoadIfNewer();

    // generate a string to be displayed in a VDGS
    const std::string GenDepartureStr() const;

    // singleton for the current flight
    static Ofp ofp;

    static void Initialize();   // delayed initialization, called from the openSAM flight loop
    static bool SbhAvailable() { return sbh_avail; }
};
