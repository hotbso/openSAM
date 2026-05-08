
import re
import shutil, sys, os

# X-Planes starts to query drefs *somewhere* in the LOD range and not immediately when the camera
# comes nearer than the upper bound of a LOD range. As a consequence openSAM lights up VDGS much later
# than AutoDGS. To mimic the behavior of AutoDGS we increase the near LOD to 350m.
# In addition VDGS in openSAM are often much farther away from the stand than in AutoDGS.

target_dir_lib = "../openSAM-pkg/openSAM_Library"
target_dir_res = "../openSAM-pkg/openSAM/resources"

tex_files = ["../../AutoDGS/DGS-Safedock-T2-24/Safedock-T2-24.png",
             "../../AutoDGS/DGS-Safedock-X/Safedock-X.png"]
t2_tmpl_obj = "../../AutoDGS/DGS-Safedock-T2-24/Safedock-T2-24.obj"
t2_tmpl_obj_pole = "../../AutoDGS/DGS-Safedock-T2-24/Safedock-T2-24-pole.obj"

resource_files = ["../../AutoDGS/DGS-Safedock-T2-24/Safedock-T2-24-display.obj", "../../AutoDGS/DGS-Safedock-T2-24/Safedock-T2-24-display_LIT.png",
                  "../../AutoDGS/DGS-Safedock-X/Safedock-X-display.obj", "../../AutoDGS/DGS-Safedock-X/Safedock-X-idle.obj",
                  "../../AutoDGS/DGS-Safedock-X/Safedock-X-display_LIT.png"]

new_near_lod = "350"

AGP = """A
1000
AG_POINT

TEXTURE ../blank/blank.png
TEXTURE_SCALE 5 5
TEXTURE_WIDTH 10
HIDE_TILES

OBJECT {}

TILE -5 -5 5 5
ROTATION 0
ANCHOR_PT 0.0 0.0
OBJ_DRAPED 0 0 0 0
"""

AGP_180 = """A
1000
AG_POINT

TEXTURE ../blank/blank.png
TEXTURE_SCALE 5 5
TEXTURE_WIDTH 10
HIDE_TILES

OBJECT {}

TILE -5 -5 5 5
ROTATION 2
ANCHOR_PT 0.0 0.0
OBJ_DRAPED 0 0 0 0
"""

tags = ["marshaller", "sam1_legacy"]
heights = [0, 0]
turns = ["false", "false"]
gen_files = []

def gen_variant(master, v_name, height, turn_180 = False):
    tag = v_name.split("/")[-1]
    tag = tag.replace(".obj", "")
    tag = re.sub(r'[-.]', '_', tag).lower()
    tags.append(tag)

    if "_180" in tag:
        turns.append("true")
    else:
        turns.append("false")
    heights.append(height)

    print(f"Generating variant '{v_name}' with tag '{tag}' and height {height}m (turn_180={turn_180})")
    lines = open(master, "r").readlines()
    dh = height

    vlines = []

    n_lod = 0
    n_no_opensam = 0

    for l in lines:
        if False:
            if "# NO-openSAM_end" in l:
                n_no_opensam -= 1
                continue
            elif "# NO-openSAM_begin" in l:
                n_no_opensam += 1

            if n_no_opensam > 0:
                continue

        l = l.replace("AutoDGS", "opensam")
        if l.startswith("ATTR_LOD"):
            words = l.split()
            n_lod += 1
            if n_lod == 1:
                prev_far_lod = new_near_lod
                vlines.append(f"ATTR_LOD 0 {new_near_lod}\n")
                vlines.append("ANIM_begin\n")
                vlines.append(f"ANIM_hide 1 1 opensam/dgs/ident/{tag}\n")
            else:
                vlines.append("ANIM_end\n")
                vlines.append(f"ATTR_LOD {prev_far_lod} {words[2]}\n")
                prev_far_lod = words[2]

                vlines.append("ANIM_begin\n")
                vlines.append(f"ANIM_hide 1 1 opensam/dgs/ident/{tag}\n")

            if turn_180 and not l.startswith("#"):
                vlines.append("    ANIM_rotate 0 1 0 180 180 0 1 no_ref\n")

            if abs(dh) <= 0.03:
                continue
            else:
                vlines.append(f"    ANIM_trans	   0.0000    {dh:0.3f}    0.0000	   0.0000    {dh:0.3f}    0.0000	0 0	no_ref\n")
        else:
            vlines.append(l)

    vlines.append("\nANIM_end\n")   # close last LOD
    open(v_name, "w", newline='\n').writelines(vlines)

def check():
    """Check whether all variants in library.txt exist"""
    res = True
    files = []
    for l in open(target_dir_lib + "/library.txt", "r").readlines():
        if not l.startswith("EXPORT"):
            continue
        w =l.split()
        files.append(w[2])

    # make unique
    files = list(set(files))
    for f in files:
        ff = f"{target_dir_lib}/{f}"
        if not os.path.exists(ff):
            print(f"Does not exist: '{ff}'")
            res = False
    if res:
        print("All files in library.txt exist")

# Safedock-X is modern with height 0 only
fn = f"{target_dir_lib}/dgs/Safedock-X.obj"
gen_variant("../../AutoDGS/DGS-Safedock-X/Safedock-X.obj", fn, 0)
gen_files.append(fn)

fn = f"{target_dir_lib}/dgs/Safedock-X-pole.obj"
gen_variant("../../AutoDGS/DGS-Safedock-X/Safedock-X-pole.obj", fn, 0)
gen_files.append(fn)

for h in [0, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}.obj"
    gen_variant(t2_tmpl_obj, f"{target_dir_lib}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}_180.obj"
    gen_variant(t2_tmpl_obj, f"{target_dir_lib}/{fn}", h, True)
    gen_files.append(fn)


for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir_lib}/{fn}", "w", newline='\n').write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir_lib}/{fn}", "w", newline='\n').write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [0, 2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}.obj"
    gen_variant(t2_tmpl_obj_pole, f"{target_dir_lib}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.obj"
    gen_variant(t2_tmpl_obj_pole, f"{target_dir_lib}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir_lib}/{fn}", "w", newline='\n').write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir_lib}/{fn}", "w", newline='\n').write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

print("\n".join(gen_files))

for f in tex_files:
    ft = shutil.copy(f, f"{target_dir_lib}/dgs/", follow_symlinks=True)
    print(f"Copied: '{ft}'")

for f in resource_files:
    if ".obj" in f:
        with open(f, "r") as fr:
            lines = fr.readlines()
        tgtname = f"{target_dir_res}/" + f.split("/")[-1]
        with open(tgtname, "w", newline='\n') as fw:
            for l in lines:
                l = l.replace("AutoDGS/", "opensam/")
                fw.write(l)
        print(f"Processed: '{tgtname}'")
    else:
        ft = shutil.copy(f, f"{target_dir_res}/", follow_symlinks=True)
        print(f"Copied: '{ft}'")

check()

copyright = """
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
"""
with open(f"../dgs_variants_generated.h", "w", newline='\n') as f:
    f.write(copyright)
    f.write("\n\n")
    f.write("// Generated code for DGS variants - do not edit manually, edit gen_height_variants.py instead\n\n")
    f.write("enum kDgsVariant {\n")
    for t in tags:
        f.write(f"    kDgsVar_{t},\n")
    f.write("    kDgsVarCount};\n\n")

    f.write("const char* dgs_variant_drefs[] = {\n")
    for t in tags:
        f.write(f'"opensam/dgs/ident/{t}",\n')
    f.write("};\n\n")

    f.write("static inline void GetDgsVariantParams(kDgsVariant variant, unsigned int& dgs_type, float& height, bool& turn_180) {\n")
    f.write("    switch (variant) {\n")
    for t in tags:
        if t == "marshaller":
            dgs_type = "kDgsType_Marshaller"
        elif "safedock_t2_24" in t:
            dgs_type = "kDgsType_Safedock_T2_24"
        elif "safedock_x" in t:
            dgs_type = "kDgsType_Safedock_X"
        elif "sam1_legacy" in t:
            dgs_type = "kDgsType_SAM1_Legacy"
        else:
            raise ValueError(f"Unknown DGS type for tag '{t}'")

        f.write(f"        case kDgsVar_{t}:\n")
        f.write(f'            dgs_type = {dgs_type};\n')
        f.write(f'            height = {heights[tags.index(t)]:0.1f}f;\n')
        f.write(f'            turn_180 = {turns[tags.index(t)]};\n')
        f.write(f'            break;\n')

    f.write("        default:\n")
    f.write('            break;\n')
    f.write("    }\n")
    f.write("}\n")
    f.write("\n// end of generated code\n")

print("\nGenerated '../dgs_variants_generated.h' with enum and GetDgsVariantParams()")
