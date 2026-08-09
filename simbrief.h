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

#define F(f) std::string f

struct Ofp
{
    int seqno{0};          // incremented after each successfull fetch, 0 == empty / no data
    bool stale{false};       // true if the data may be stale (e.g. was valid but now simbrief not responding)
    F(icao_airline);
    F(flight_number);
    F(aircraft_icao);
    F(destination);
    F(pax_count);
    F(freight);
    F(fuel_plan_ramp);
    F(est_out);
    F(est_off);
    F(est_on);
    F(est_in);
    F(dx_rmk);

    // cdm fields
    F(cdm_tobt);
    F(cdm_tsat);
    F(cdm_ctot);
    F(cdm_runway);
    F(cdm_sid);

    std::string callsign;   // for convenience, not part of the simbrief_hub data

    // return ptr to an OFP if a newer version is available or nullptr
    static bool LoadIfNewer(); // -> updated

    // generate a string to be displayed in a VDGS
    const std::string GenDepartureStr() const;
};

#undef F

// global instance, there is only 'the' ofp for the current flight
extern Ofp ofp;
