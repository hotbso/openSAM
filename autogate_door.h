//
// Minimal stub — full AutoGate-style door logic may be added locally.
//

#ifndef AUTOGATE_DOOR_H
#define AUTOGATE_DOOR_H

#include <string>

#include "XPLMDataAccess.h"

/// Adjust door position (m, aircraft coordinates) before writing door_info_; stub leaves values unchanged.
void ApplyAutogateStyleDoor1(float& dx, float& dy, float& dz, const std::string& icao, XPLMDataRef acf_descrip_dr,
                             XPLMDataRef acf_cg_y_dr, XPLMDataRef acf_cg_z_dr);

#endif
