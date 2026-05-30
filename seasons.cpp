//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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

#include "seasons.h"
#include "opensam.h"
#include "XPLMDataAccess.h"

// support for legacy SAM seasons
namespace Seasons {

int auto_season;
int nh;      // on northern hemisphere
int season;  // 0-3

static const char* dr_name[] = {"sam/season/winter", "sam/season/spring", "sam/season/summer", "sam/season/autumn"};
static XPLMDataRef date_day_dr;

// Accessor for the "sam/season/*" datarefs
static int ReadSeasonAcc(void* ref) {
    int s = (long long)ref;
    int val = (s == season);
    return val;
}

void InitDataRefs() {
    date_day_dr = XPLMFindDataRef("sim/time/local_date_days");

    for (int i = 0; i < 4; i++)
        XPLMRegisterDataAccessor(dr_name[i], xplmType_Int, 0, ReadSeasonAcc, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL, NULL, (void*)(long long)i, NULL);
}

void SetAuto() {
    if (!auto_season)
        return;

    int day = XPLMGetDatai(date_day_dr);
    if (nh) {
        if (day <= 80) {
            season = 0;
        } else if (day <= 172) {
            season = 1;
        } else if (day <= 264) {
            season = 2;
        } else if (day <= 355) {
            season = 3;
        } else if (day) {
            season = 0;
        }
    } else {
        if (day <= 80) {
            season = 2;
        } else if (day <= 172) {
            season = 3;
        } else if (day <= 264) {
            season = 0;
        } else if (day <= 355) {
            season = 1;
        } else if (day) {
            season = 2;
        }
    }

    LogMsg("nh: %d, day: %d, season: %d", nh, day, season);
}

} // namespace Seasons
