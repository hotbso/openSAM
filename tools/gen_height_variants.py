
import shutil, sys, os

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

def gen_variant(master, v_name, height, turn_180 = False):
    lines = open(master, "r").readlines()
    dh = height

    vlines = []

    first_lod = True
    n_no_opensam = 0
    for l in lines:
        if "# NO-openSAM_end" in l:
            n_no_opensam -= 1
            continue
        elif "# NO-openSAM_begin" in l:
            n_no_opensam += 1

        if n_no_opensam > 0:
            continue

        l = l.replace("AutoDGS", "opensam")
        if "LOD" in l and not first_lod:
            vlines.append("ANIM_end\n")

        vlines.append(l)

        if "LOD" in l:
            first_lod = False
            vlines.append("ANIM_begin\n")
            if turn_180 and not l.startswith("#"):
                vlines.append("    ANIM_rotate 0 1 0 180 180 0 1 no_ref\n")

            if abs(dh) <= 0.03:
                continue
            else:
                vlines.append(f"    ANIM_trans	   0.0000    {dh:0.3f}    0.0000	   0.0000    {dh:0.3f}    0.0000	0 0	no_ref\n")

    vlines.append("\nANIM_end\n")   # close last LOD
    open(v_name, "w").writelines(vlines)

def check():
    """Check whether all variants in library.txt exist"""
    res = True
    files = []
    for l in open(target_dir + "/library.txt", "r").readlines():
        if not l.startswith("EXPORT"):
            continue
        w =l.split()
        files.append(w[2])

    # make unique
    files = list(set(files))
    for f in files:
        ff = f"{target_dir}/{f}"
        if not os.path.exists(ff):
            print(f"Does not exist: '{ff}'")
            res = False
    if res:
        print("All files in library.txt exist")

target_dir = "../openSAM-pkg/openSAM_Library"
gen_files = []

tex_files = ["../../AutoDGS/DGSs-Safedock-T2-24/Safedock-T2-24.png", "../../AutoDGS/DGSs-Safedock-T2-24/Safedock-T2-24_LIT.png"]
tmpl_obj = "../../AutoDGS/DGSs-Safedock-T2-24/Safedock-T2-24.obj"
tmpl_obj_pole = "../../AutoDGS/DGSs-Safedock-T2-24/Safedock-T2-24-pole.obj"

for h in [0, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}.obj"
    gen_variant(tmpl_obj, f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}_180.obj"
    gen_variant(tmpl_obj, f"{target_dir}/{fn}", h, True)
    gen_files.append(fn)


for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"Safedock-T2-24_{h:0.1f}m"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [0, 2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}.obj"
    gen_variant(tmpl_obj_pole, f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.obj"
    gen_variant(tmpl_obj_pole, f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 6]:
    name = f"Safedock-T2-24_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

print("\n".join(gen_files))

for f in tex_files:
    ft = shutil.copy(f, f"{target_dir}/dgs/", follow_symlinks=True)
    print(f"Copied: '{ft}'")

check()
