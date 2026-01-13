//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
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
#include <vector>

#define DRF_MAX_ANIM 10
struct SamDrf {
    std::string name;

    int n_tv;
    std::vector<float> t;
    std::vector<float> v;
    std::vector<float> s;  // s[i] = slope for (i-1, i)

    bool autoplay, randomize_phase, augment_wind_speed;
};

struct SamObj {
    std::string id;
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
    std::string label;
    std::string title;

    int drf_idx;        // index into sam_drfs
    int obj_idx;        // index into sc->sam_objs

    anim_state_t state;
    float start_ts;

    int menu_item;
};

extern std::vector<SamDrf*> sam_drfs;

extern bool AnimInit(void);
extern float AnimStateMachine(void);
extern void AnimMenuCb(void *menu_ref, void *item_ref);

