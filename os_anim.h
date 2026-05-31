//
//    openSAM: manage DGS and jetways for X Plane
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

#include <cstddef>
#include <vector>

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
    unsigned int xml_ref_gen;   // only valid if this matches the generation of the ref frame
    float xml_x, xml_y, xml_z;
};

class SamAnim {
   public:
    enum AnimState { kOff = 0, kOff2On, kOn2Off, kOn };

    std::string label, title;
    std::string ui_line;

    int drf_idx;  // index into sam_drfs
    int obj_idx;  // index into sc->sam_objs

    AnimState state;
    float start_ts;

    int menu_item;

    bool is_on() const { return state == kOn || state == kOff2On; }
    void SetState(bool on);

    static float AnimAcc(void* ref);    // dref accessor
};

extern std::vector<SamDrf*> sam_drfs;
extern Scenery* anim_sc;                // the scenery currently selected for animation

extern bool AnimInit(void);
extern float AnimStateMachine(void);
extern void AnimMenuCb(void *menu_ref, void *item_ref);

