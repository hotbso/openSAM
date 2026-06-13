//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023, 2025 Holger Teutsch
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
//

#include "opensam.h"
#include "autodgs_airport.h"
#include "my_plane.h"
#include "log_msg.h"

enum {
    API_OPERATION_MODE,
    API_ON_GROUND,
};

// API accessor routines
static int
api_getint(XPLMDataRef ref)
{
    switch ((long long)ref) {
        case API_OPERATION_MODE:
            return operation_mode;
        case API_ON_GROUND:
            return my_plane->on_ground() ? 1 : 0;
    }

    return 0;
}

static void
api_setint(XPLMDataRef ref, int val)
{
    switch ((long long)ref) {
        case API_OPERATION_MODE:
            ; // required by some gcc versions
            opmode_t mode = (opmode_t)val;
            if (mode != MODE_AUTO && mode != MODE_MANUAL) {
                LogMsg("API: trying to set invalid operation_mode %d, ignored", val);
                return;
            }

            if (mode == operation_mode) // Lua hammers writeable drefs in a frame loop
                return;

            LogMsg("API: operation_mode set to %s", opmode_str[mode]);
            operation_mode = mode;
            break;
    }
}


void
create_api_drefs()
{
        // API datarefs
    XPLMRegisterDataAccessor("AutoDGS/operation_mode", xplmType_Int, 1, api_getint, api_setint, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_OPERATION_MODE, (void *)API_OPERATION_MODE);

    XPLMRegisterDataAccessor("AutoDGS/on_ground", xplmType_Int, 0, api_getint, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_ON_GROUND, NULL);
}
