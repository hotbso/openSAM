"""
    AutoDGS: Show Marshaller or VDGS at default airports

    Copyright (C) 2025  Holger Teutsch

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
"""
import os
import sys
from vdgstools.display import AnimBlock, TexQuad # keep vscode happy
from vdgstools.display import *

#################
if len(sys.argv) < 2:
    sys.exit("no base obj given")

# idle display only ?
idle = False
if len(sys.argv) > 2 and sys.argv[2] == "-idle":
    idle = True

xpo = XPObj(sys.argv[1], -0.465, -0.095, 1.043, 0.92, 1.15)

# we work in cm, but XPObj is in m, so set scale to 0.01
xpo.set_scale(0.01)

status_dr = "opensam/dgs/status"
status_1_dr = "opensam/dgs/status_1"
track_dr = "opensam/dgs/track"
distance_dr = "opensam/dgs/distance"
distance_0_dr = "opensam/dgs/distance_0"
distance_01_dr = "opensam/dgs/distance_01"
xtrack_dr = "opensam/dgs/xtrack"
lr_dr = "opensam/dgs/lr"
icao_0_dr = "opensam/dgs/icao_0"
icao_1_dr = "opensam/dgs/icao_1"
icao_2_dr = "opensam/dgs/icao_2"
icao_3_dr = "opensam/dgs/icao_3"
move_dr = "opensam/dgs/move"
blink_dr = "opensam/dgs/blink"

chk_dr = "opensam/dgs/chk"
gpu_dr = "opensam/dgs/gpu"
pca_dr = "opensam/dgs/pca"
pbb_dr = "opensam/dgs/pbb"

h = 80  # char height in tex pixels, char width is 48, but we use 48.15 to get the correct spacing for the font
w = 48

# Display is 92 x 115 cm and we adress directly in cm
dh = 115
dw = 92
# 8 tex pixels correspond to 0.01m (1 LED cell), so 1 tex pixel is 0.00125m, or 0.125cm
tex  = Texture(xpo, 2048, 1024,  0.01 / 8)

# get all chars, font: "Roboto Mono Medium",
# char height in tex pixels is 80, char width is 48, but we use 48.15 to get the correct spacing for the font
char_tex = Texture(xpo, 2048, 1024,  0.15 / h)
char_txq = []

x = 9
y = 26
dx = 48.15
for i in range(0, 20):
    char_txq.append(TexQuad(char_tex, x, y, w, h))
    x += dx

x = 9
y = y + 106
for i in range(0, 19):
    char_txq.append(TexQuad(char_tex, x, y, w, h))
    x += dx

c_m = 36
c_colon = 37
decimal_txq = char_txq[38]

# symbols
hbar_txq = TexQuad(tex, 25, 233, 384, 4 * 8)
vbar_txq = TexQuad(tex, 25, 233, 6 * 8 , 2 * 8)

plane2_txq = TexQuad(tex, 20, 646, 350, 345) # the plane
plane4_txq = TexQuad(tex, 20, 646, 350, 345)
red_block_txq = TexQuad(tex, 25, 361, 128, 128) # the red block

lr_right_txq = TexQuad(tex, 175, 360, 74, 128)
lr_left_txq = TexQuad(tex, 175, 360, 74, 128, mirror_x = True)
lead_in_arrow_txq = TexQuad(tex, 209, 507, 120, 100)
lead_in_wave_txq = TexQuad(tex, 1411, 22, 171, 97)

stop_txq = TexQuad(char_tex, 606, 269, 239,88)
too_far_txq = TexQuad(char_tex, 579, 386, 333, 64)
chock_txq = TexQuad(char_tex, 579, 457, 281, 60)
off_txq = TexQuad(char_tex, 870, 536, 138, 60)
on_txq = TexQuad(char_tex, 915, 457, 90, 60)
aibt_txq = TexQuad(char_tex, 750, 622, 190, 60)
deboarding_txq = TexQuad(char_tex, 737, 909, 474, 60)
chock_on_sym_txq = TexQuad(tex, 452, 248, 82, 82)

chock_on_lg_frame_txq = TexQuad(tex, 1014, 14, 375, 375)
chock_on_lg_txq = TexQuad(tex, 1014, 414, 375, 375)

chk_sym_txq = TexQuad(tex, 455, 248, 82, 82)
gpu_sym_txq = TexQuad(tex, 455, 339, 82, 82)
pca_sym_txq = TexQuad(tex, 455, 547, 82, 82)
pbb_sym_txq = TexQuad(tex, 455, 443, 82, 82)

chk_txt_txq = TexQuad(char_tex, 735, 826, 142, 62)
gpu_txt_txq = TexQuad(char_tex, 879, 826, 142, 62)
pca_txt_txq = TexQuad(char_tex, 1026, 826, 142, 62)
pbb_txt_txq = TexQuad(char_tex, 1171, 826, 142, 62)

pax_txq = TexQuad(char_tex, 584, 530, 135, 65)
ok_txq = TexQuad(char_tex, 776, 530, 87, 65)

def icao_large():
    """helper ICAO in large font"""
    xpo.line("# distance > 10: ICAO large font")
    y = 82
    x = 24
    dx = 12
    xpo.char_stack(char_txq, x, y, icao_0_dr, first = 10, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_1_dr, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_2_dr, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_3_dr, last = 36, h2w = 2.0)


# augment the .obj
xpo.line("# ---- status == 0, display stand + display UTC time")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 0)

    y = 98
    x = 20      # align with ICAO field to save some TRIS
    dx = 12
    with AnimBlock(xpo):
        xpo.trans_x(-16, 0, 0, 16, "opensam/dgs/r1_scroll") # lateral
        for i in range(0, 6):
            xpo.char_stack(char_txq, x, y, f"opensam/dgs/r1c{i}")
            x += dx

    y = 82
    x = 20
    dx = 12
    xpo.char_stack(char_txq, x, y, "opensam/dgs/time_utc_h1", last = 3, ascii = False)
    x = x + dx
    xpo.char_stack(char_txq, x, y, "opensam/dgs/time_utc_h0", last = 10, ascii = False)
    x = x + dx
    xpo.quad(char_txq[c_colon], x, y)
    x = x + dx * 0.8
    xpo.char_stack(char_txq, x, y, "opensam/dgs/time_utc_m1", last = 6, ascii = False)
    x = x + dx
    xpo.char_stack(char_txq, x, y, "opensam/dgs/time_utc_m0", last = 10, ascii = False)

# that's it for idle display, dump and exit
if idle:
    xpo.dump("Safedock-X-idle.obj")
    sys.exit()

###
### other modes than idle
###
xpo.line("# ---- boarding == 1, -> pax no")
with AnimBlock(xpo):
    xpo.show_if_eq("opensam/dgs/boarding", 1)
    xpo.quad(pax_txq, 15, 69)
    x = 50
    y = 66
    dx = 12
    x += 2 * dx # 3 digits = 2 spaces
    for i in range(3):
        xpo.char_stack(char_txq, x, y, f"opensam/dgs/paxno_{i}", last = 10, ascii = False)
        x -= dx

xpo.line("#---- status = 1 && track = 1 -> lead in")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 1)
    xpo.show_if_eq(track_dr, 1)
    icao_large()

    y = 80
    steps = 12
    total_scale = 2
    dy = 3.5
    scale_per_step = pow(total_scale, 1/(steps -1))
    scale = pow(scale_per_step, -2) # start with shrink
    for i in range(0, steps):
        with AnimBlock(xpo):
            xpo.show_if_in_range(move_dr, i / steps - 0.02, (i + 1) / steps + 0.02)
            xpo.quad(lead_in_wave_txq, dw / 2, y, anchor = 'ct', scale = scale)
        scale = scale * scale_per_step
        y -= dy
        dy *= scale_per_step

xpo.line("#---- status = 1 && track = 2,3 -> show guidance info")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 1)
    xpo.show_if_in_range(track_dr, 2, 3)

    xpo.line("# distance > 10: ICAO large font")
    with AnimBlock(xpo):
        xpo.hide_if_in_range(distance_dr, -100, 9.99)
        icao_large()

    xpo.line("# distance < 10: ICAO small font + distance < 10 readout")
    with AnimBlock(xpo):
        xpo.show_if_in_range(distance_dr, -100, 9.99)

        # ICAO normal font
        y = 98
        x = 24
        dx = 11
        xpo.char_stack(char_txq, x, y, icao_0_dr, first = 10, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_1_dr, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_2_dr, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_3_dr, last = 36)

        # field "9.9m"
        y = 84
        x = 26
        xpo.char_stack(char_txq, x, y, distance_0_dr, last = 9, ascii = False)
        x = x + dx
        xpo.quad(decimal_txq, x, y)
        x = x + dx * 0.5
        xpo.char_stack(char_txq, x, y, distance_01_dr, last = 9, ascii = False)
        x = x + dx
        xpo.quad(char_txq[c_m], x, y)

    xpo.line("# show hbar")
    vbar_top = 80
    vbar_length = 34
    xpo.quad(hbar_txq, dw/2, vbar_top, anchor  = 'cb')

    xpo.line("# show vbar + plane azimuth marker")

    for i in range(0, 24):  # 12m in 0.5 increments
        with AnimBlock(xpo):
            xpo.hide_if_in_range(distance_dr, 0, i * 0.5 + 0.01)
            xpo.quad(vbar_txq, dw/2, vbar_top - i * vbar_length / 24, anchor = 'ct')

    # plane azimuth marker
    yazi = vbar_top - 2

    xpo.line("# Plane azimuth marker")
    with AnimBlock(xpo):
        xpo.trans_y(0, -vbar_length, 0.0, 12.0, distance_dr) # below vbar
        with AnimBlock(xpo):
            xpo.trans_x(-16, 16, -4.0, 4.0, xtrack_dr) # lateral
            xpo.quad(plane2_txq, dw/2, yazi, anchor = 'ct')

    xpo.line("# LR arrow right side")
    with AnimBlock(xpo):
        xpo.show_if_eq(lr_dr, 2)
        xpo.hide_if_eq(blink_dr, 0)
        xpo.quad(lr_right_txq, dw, yazi, anchor = 'rb')

    xpo.line("# LR arrow left side")
    with AnimBlock(xpo):
        xpo.show_if_eq(lr_dr, 1)
        xpo.hide_if_eq(blink_dr, 0)
        xpo.quad(lr_left_txq, 0, yazi, anchor = 'lb')

xpo.line("#---- status = 2 -> red blocks + STOP")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 2)
    xpo.quad(red_block_txq, 0, yazi)
    xpo.quad(red_block_txq, dw, yazi, anchor = 'rb')
    xpo.quad(stop_txq, dw/2, dh - 4, h2w = 2.0, anchor = 'ct')

xpo.line("#---- status = 3 -> OK")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 3)
    xpo.quad(ok_txq, dw/2, 90, h2w = 2.0, anchor = 'cb')

    # chocks off
    with AnimBlock(xpo):
        xpo.hide_if_eq(chk_dr, 0)   # status is n/a

        with AnimBlock(xpo):
            xpo.show_if_eq(chk_dr, 1)   # status is off
            xpo.quad(chock_on_lg_frame_txq, dw/2, 30, anchor = 'cb')
            xpo.quad(chock_txq, 7, 10)
            xpo.quad(off_txq, 65, 10)

xpo.line("#---- status = 4 -> TOO FAR")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 4)
    xpo.quad(too_far_txq, dw/2, 86, anchor = 'cb')

xpo.line("#---- status_1 = 1 -> CHK status large")
with AnimBlock(xpo):
    xpo.hide_if_eq(chk_dr, 0)   # status is n/a
    xpo.show_if_eq(status_1_dr, 1)   # parked

    with AnimBlock(xpo):
        xpo.show_if_eq(chk_dr, 1)   # status is off
        xpo.quad(chock_on_lg_frame_txq, dw/2, 30, anchor = 'cb')
        xpo.quad(chock_txq, 7, 10)
        xpo.quad(off_txq, 65, 10)

    with AnimBlock(xpo):
        xpo.show_if_eq(chk_dr, 2)   # status is on
        xpo.quad(chock_on_lg_txq, dw/2, 30, anchor = 'cb')
        xpo.quad(chock_txq, 7, 10)
        xpo.quad(on_txq, 65, 10)

xpo.line("# status_1 = 2 -> equipment status small, e.g. GPU, PCA, PBB")
with AnimBlock(xpo):
    xpo.show_if_eq(status_1_dr, 2)   # show equipment status

    x = 5
    y = 52
    dy_sym = 2
    dx_txt = 16
    dx_on_off = 50

    with AnimBlock(xpo):
        xpo.quad(chk_sym_txq, x, y + dy_sym, anchor = 'lb')
        xpo.quad(chk_txt_txq, x + dx_txt, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(chk_dr, 1)   # status is off
            xpo.quad(off_txq, x + dx_on_off, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(chk_dr, 2)   # status is on
            xpo.quad(on_txq, x + dx_on_off, y, anchor = 'lb')
    y -= 16

    with AnimBlock(xpo):
        xpo.quad(gpu_sym_txq, x, y + dy_sym, anchor = 'lb')
        xpo.quad(gpu_txt_txq, x + dx_txt, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(gpu_dr, 1)   # status is off
            xpo.quad(off_txq, x + dx_on_off, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(gpu_dr, 2)   # status is on
            xpo.quad(on_txq, x + dx_on_off, y, anchor = 'lb')

    y -= 16
    with AnimBlock(xpo):
        xpo.quad(pca_sym_txq, x, y + dy_sym, anchor = 'lb')
        xpo.quad(pca_txt_txq, x + dx_txt, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(pca_dr, 1)   # status is off
            xpo.quad(off_txq, x + dx_on_off, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(pca_dr, 2)   # status is on
            xpo.quad(on_txq, x + dx_on_off, y, anchor = 'lb')

    y -= 16
    with AnimBlock(xpo):
        xpo.hide_if_eq(pbb_dr, 0)   # status is n/a
        xpo.quad(pbb_sym_txq, x, y + dy_sym, anchor = 'lb')
        xpo.quad(pbb_txt_txq, x + dx_txt, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(pbb_dr, 1)   # status is off
            xpo.quad(off_txq, x + dx_on_off, y, anchor = 'lb')
        with AnimBlock(xpo):
            xpo.show_if_eq(pbb_dr, 2)   # status is on
            xpo.quad(on_txq, x + dx_on_off, y, anchor = 'lb')

    with AnimBlock(xpo):
        xpo.show_if_eq("opensam/dgs/deboarding", 1)   # parked
        xpo.quad(deboarding_txq, dw/2, 69, anchor = 'cb')

xpo.dump()
