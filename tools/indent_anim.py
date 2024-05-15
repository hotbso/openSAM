import sys

lines = open(sys.argv[1], "r").readlines()

f = sys.stdout

indent = 0
for l in lines:
    if "ANIM_begin" in l:
        l = " " * indent + l
        f.write(l)
        indent += 4
    elif "ANIM_end" in l:
        indent -= 4
        l = " " * indent + l
        f.write(l)
    else:
        l = " " * indent + l
        f.write(l)

       