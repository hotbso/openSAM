/*
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

*/

#include <cstddef>
#include <vector>

#define DRF_MAX_ANIM 10
struct SamDrf {
    char name[60];

    int n_tv;
    float t[DRF_MAX_ANIM];
    float v[DRF_MAX_ANIM];
    float s[DRF_MAX_ANIM];  // s[i] = slope for (i-1, i)

    bool autoplay, randomize_phase, augment_wind_speed;
};

struct SamObj {
    char id[30];
    float latitude, longitude, elevation, heading;

    // local x,y,z computed from the xml's lat/lon
    float xml_x, xml_y, xml_z;
    unsigned int xml_ref_gen;   // only valid if this matches the generation of the ref frame
};

typedef enum _ANIM_STATE  {
    ANIM_OFF = 0,
    ANIM_OFF_2_ON,
    ANIM_ON_2_OFF,
    ANIM_ON
} anim_state_t;

class SamAnim {
  public:
    char label[40];
    char title[40];

    int drf_idx;        // index into sam_drfs
    int obj_idx;        // index into sc->sam_objs

    anim_state_t state;
    float start_ts;

    int menu_item;
};

extern std::vector<SamDrf*> sam_drfs;

extern int anim_init(void);
extern float anim_state_machine(void);
extern void anim_menu_cb(void *menu_ref, void *item_ref);

