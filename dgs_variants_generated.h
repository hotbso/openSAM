
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

static inline void GetDgsVariantParams(kDgsVariant variant, OsDgsCtx& ctx) {
    switch (variant) {
        case kDgsVar_marshaller:
            ctx.dgs_type = kDgsType_Marshaller;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_sam1_legacy:
            ctx.dgs_type = kDgsType_SAM1_Legacy;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_x:
            ctx.dgs_type = kDgsType_Safedock_X;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_x_pole:
            ctx.dgs_type = kDgsType_Safedock_X;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_0_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 2.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_5m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_5m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_5m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_5m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_7_0m:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 7.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 2.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_0m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_5m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_0m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_5m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_0m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_5m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_0m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_5m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_7_0m_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 7.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_0_0m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 0.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 2.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_0m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_3_5m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_0m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_4_5m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.5f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_5_0m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_6_0m_pole:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.0f;
            ctx.turn_180 = false;
            break;
        case kDgsVar_safedock_t2_24_2_5m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 2.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_0m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_3_5m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 3.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_0m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_4_5m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 4.5f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_5_0m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 5.0f;
            ctx.turn_180 = true;
            break;
        case kDgsVar_safedock_t2_24_6_0m_pole_180:
            ctx.dgs_type = kDgsType_Safedock_T2_24;
            ctx.height = 6.0f;
            ctx.turn_180 = true;
            break;
        default:
            break;
    }
}

// end of generated code
