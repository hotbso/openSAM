
magic = "SHIFT"

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
    dh = height - 5.0
    magic_repl = f"{dh:0.4f}"

    vlines = []

    for l in lines:
        if magic in l:
            if turn_180 and not l.startswith("#"):
                vlines.append("    ANIM_rotate 0 1 0 180 180 0 1 no_ref\n")

            if abs(dh) <= 0.03:
                continue
            else:
               l = l.replace(magic, magic_repl)

        vlines.append(l)

    open(v_name, "w").writelines(vlines)

target_dir = "../openSAM-pkg/openSAM_Library"
gen_files = []

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"SafedockT2_{h:0.1f}m"
    fn = f"dgs/{name}.obj"
    gen_variant("SafedockT2-6m-SHIFT.obj", f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"SafedockT2_{h:0.1f}m"
    fn = f"dgs/{name}_180.obj"
    gen_variant("SafedockT2-6m-SHIFT.obj", f"{target_dir}/{fn}", h, True)
    gen_files.append(fn)


for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"SafedockT2_{h:0.1f}m"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7]:
    name = f"SafedockT2_{h:0.1f}m"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5]:
    name = f"SafedockT2_{h:0.1f}m_pole"
    fn = f"dgs/{name}.obj"
    gen_variant("SafedockT2-6m-pole-SHIFT.obj", f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5]:
    name = f"SafedockT2_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.obj"
    gen_variant("SafedockT2-6m-pole-SHIFT.obj", f"{target_dir}/{fn}", h)
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5]:
    name = f"SafedockT2_{h:0.1f}m_pole"
    fn = f"dgs/{name}.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP.format(f"{name}.obj"))
    gen_files.append(fn)

for h in [2.5, 3, 3.5, 4, 4.5, 5]:
    name = f"SafedockT2_{h:0.1f}m_pole"
    fn = f"dgs/{name}_180.agp"
    open(f"{target_dir}/{fn}", "w").write(AGP_180.format(f"{name}.obj"))
    gen_files.append(fn)

print("\n".join(gen_files))

