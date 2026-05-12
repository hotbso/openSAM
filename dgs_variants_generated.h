
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


// Generated code for DGS variants - do not edit manually, edit gen_height_variants.py instead

enum kDgsVariant {
    kDgsVar_marshaller,
    kDgsVar_sam1_legacy,
    kDgsVar_safedock_x,
    kDgsVar_safedock_x_pole,
    kDgsVar_safedock_t2_24_0_0m,
    kDgsVar_safedock_t2_24_2_5m,
    kDgsVar_safedock_t2_24_3_0m,
    kDgsVar_safedock_t2_24_3_5m,
    kDgsVar_safedock_t2_24_4_0m,
    kDgsVar_safedock_t2_24_4_5m,
    kDgsVar_safedock_t2_24_5_0m,
    kDgsVar_safedock_t2_24_5_5m,
    kDgsVar_safedock_t2_24_6_0m,
    kDgsVar_safedock_t2_24_6_5m,
    kDgsVar_safedock_t2_24_7_0m,
    kDgsVar_safedock_t2_24_2_5m_180,
    kDgsVar_safedock_t2_24_3_0m_180,
    kDgsVar_safedock_t2_24_3_5m_180,
    kDgsVar_safedock_t2_24_4_0m_180,
    kDgsVar_safedock_t2_24_4_5m_180,
    kDgsVar_safedock_t2_24_5_0m_180,
    kDgsVar_safedock_t2_24_5_5m_180,
    kDgsVar_safedock_t2_24_6_0m_180,
    kDgsVar_safedock_t2_24_6_5m_180,
    kDgsVar_safedock_t2_24_7_0m_180,
    kDgsVar_safedock_t2_24_0_0m_pole,
    kDgsVar_safedock_t2_24_2_5m_pole,
    kDgsVar_safedock_t2_24_3_0m_pole,
    kDgsVar_safedock_t2_24_3_5m_pole,
    kDgsVar_safedock_t2_24_4_0m_pole,
    kDgsVar_safedock_t2_24_4_5m_pole,
    kDgsVar_safedock_t2_24_5_0m_pole,
    kDgsVar_safedock_t2_24_6_0m_pole,
    kDgsVar_safedock_t2_24_2_5m_pole_180,
    kDgsVar_safedock_t2_24_3_0m_pole_180,
    kDgsVar_safedock_t2_24_3_5m_pole_180,
    kDgsVar_safedock_t2_24_4_0m_pole_180,
    kDgsVar_safedock_t2_24_4_5m_pole_180,
    kDgsVar_safedock_t2_24_5_0m_pole_180,
    kDgsVar_safedock_t2_24_6_0m_pole_180,
    kDgsVarCount};

const char* dgs_variant_drefs[] = {
"opensam/dgs/ident/marshaller",
"opensam/dgs/ident/sam1_legacy",
"opensam/dgs/ident/safedock_x",
"opensam/dgs/ident/safedock_x_pole",
"opensam/dgs/ident/safedock_t2_24_0_0m",
"opensam/dgs/ident/safedock_t2_24_2_5m",
"opensam/dgs/ident/safedock_t2_24_3_0m",
"opensam/dgs/ident/safedock_t2_24_3_5m",
"opensam/dgs/ident/safedock_t2_24_4_0m",
"opensam/dgs/ident/safedock_t2_24_4_5m",
"opensam/dgs/ident/safedock_t2_24_5_0m",
"opensam/dgs/ident/safedock_t2_24_5_5m",
"opensam/dgs/ident/safedock_t2_24_6_0m",
"opensam/dgs/ident/safedock_t2_24_6_5m",
"opensam/dgs/ident/safedock_t2_24_7_0m",
"opensam/dgs/ident/safedock_t2_24_2_5m_180",
"opensam/dgs/ident/safedock_t2_24_3_0m_180",
"opensam/dgs/ident/safedock_t2_24_3_5m_180",
"opensam/dgs/ident/safedock_t2_24_4_0m_180",
"opensam/dgs/ident/safedock_t2_24_4_5m_180",
"opensam/dgs/ident/safedock_t2_24_5_0m_180",
"opensam/dgs/ident/safedock_t2_24_5_5m_180",
"opensam/dgs/ident/safedock_t2_24_6_0m_180",
"opensam/dgs/ident/safedock_t2_24_6_5m_180",
"opensam/dgs/ident/safedock_t2_24_7_0m_180",
"opensam/dgs/ident/safedock_t2_24_0_0m_pole",
"opensam/dgs/ident/safedock_t2_24_2_5m_pole",
"opensam/dgs/ident/safedock_t2_24_3_0m_pole",
"opensam/dgs/ident/safedock_t2_24_3_5m_pole",
"opensam/dgs/ident/safedock_t2_24_4_0m_pole",
"opensam/dgs/ident/safedock_t2_24_4_5m_pole",
"opensam/dgs/ident/safedock_t2_24_5_0m_pole",
"opensam/dgs/ident/safedock_t2_24_6_0m_pole",
"opensam/dgs/ident/safedock_t2_24_2_5m_pole_180",
"opensam/dgs/ident/safedock_t2_24_3_0m_pole_180",
"opensam/dgs/ident/safedock_t2_24_3_5m_pole_180",
"opensam/dgs/ident/safedock_t2_24_4_0m_pole_180",
"opensam/dgs/ident/safedock_t2_24_4_5m_pole_180",
"opensam/dgs/ident/safedock_t2_24_5_0m_pole_180",
"opensam/dgs/ident/safedock_t2_24_6_0m_pole_180",
};

static inline void GetDgsVariantParams(kDgsVariant variant, unsigned int& dgs_type, float& height, bool& turn_180) {
    switch (variant) {
        case kDgsVar_marshaller:
            dgs_type = kDgsType_Marshaller;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_sam1_legacy:
            dgs_type = kDgsType_SAM1_Legacy;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_x:
            dgs_type = kDgsType_Safedock_X;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_x_pole:
            dgs_type = kDgsType_Safedock_X;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_0_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 2.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_5m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_5m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_5m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_5m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_7_0m:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 7.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 2.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_0m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_5m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_0m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_5m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_0m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_5m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_0m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_5m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_7_0m_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 7.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_0_0m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 0.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 2.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_0m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_5m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_0m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_5m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.5f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_0m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_0m_pole:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.0f;
            turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 2.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_0m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_5m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 3.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_0m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_5m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 4.5f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_0m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 5.0f;
            turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_0m_pole_180:
            dgs_type = kDgsType_Safedock_T2_24;
            height = 6.0f;
            turn_180 = true;
            break;
        default:
            break;
    }
}

// end of generated code
