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
import sys
from vdgstools.display import *

#################
if len(sys.argv) < 2:
    sys.exit("no base obj given")

xpo = XPObj(sys.argv[1], -0.4, -0.26, 0.51, 0.8, 0.9)

# LED tile is 16x16 leds 0.16m x 0.16m
xpo.set_scale(0.01) # we use LED coordinates, 1 LED cell is 0.01m

# convenience function LED, tile + led_no in tile
def LED(tile, led_no = 0):
    return tile * 16 + led_no

status_dr = "AutoDGS/dgs/status"
track_dr = "AutoDGS/dgs/track"
distance_dr = "AutoDGS/dgs/distance"
distance_0_dr = "AutoDGS/dgs/distance_0"
distance_01_dr = "AutoDGS/dgs/distance_01"
xtrack_dr = "AutoDGS/dgs/xtrack"
lr_dr = "AutoDGS/dgs/lr"
icao_0_dr = "AutoDGS/dgs/icao_0"
icao_1_dr = "AutoDGS/dgs/icao_1"
icao_2_dr = "AutoDGS/dgs/icao_2"
icao_3_dr = "AutoDGS/dgs/icao_3"
beacon_dr = "sim/graphics/animation/airport_beacon_rotation"
sine_wave_dr = "sim/graphics/animation/sin_wave_2"

tex  = Texture(xpo, 1024, 1024,  0.16 / (16 * 8))
char_tex = Texture(xpo, 1024, 1024,  0.16 / (9 * 8))

# get all chars, font: "Repetition Scrolling Regular"
# capitals are a 7x5 raster, with some weird spacing, a led dot is 8x8 tex pix
char_txq = []
h = 7 * 8
w = 5 * 8

x = 8
y = 146
dx = 44.25
for i in range(0, 20):
    char_txq.append(TexQuad(char_tex, x, y, w, h))
    x += dx

x = 8
y = 226
for i in range(0, 19):
    char_txq.append(TexQuad(char_tex, x, y, w, h))
    x += dx

c_m = 36
c_colon = 37
decimal_txq = char_txq[38]

# symbols
hbar_txq = TexQuad(tex, 184, 312, 384, 4 * 8)
vbar_txq = TexQuad(tex, 184, 312, 6 * 8 , 2 * 8)

azimuth_led_w = 16
azimuth_led_h = 12
azimuth_txq = TexQuad(tex, 192, 592, 8 * azimuth_led_w, 8 * azimuth_led_h) # the up arrow

red_block_led_w = 16
red_block_led_h = 16
red_block_txq = TexQuad(tex, 184, 440, 8 * red_block_led_w, 8 * red_block_led_h) # the red block

lr_right_txq = TexQuad(tex, 344, 440, 64, 128)
lr_left_txq = TexQuad(tex, 344, 440, 64, 128, mirror_x = True)
lead_in_txq = TexQuad(tex, 368, 592, 120, 96)

stop_txq = TexQuad(char_tex, 652, 350, 172, 56)
too_txq = TexQuad(char_tex, 652, 450, 128, 56)
far_txq = TexQuad(char_tex, 784, 450, 128, 56)
ok_txq = TexQuad(char_tex, 917, 450, 84, 56)
chock_txq = TexQuad(char_tex, 652, 525, 216, 56)
on_txq = TexQuad(char_tex, 874, 525, 82, 56)
pax_txq = TexQuad(char_tex, 652, 598, 128, 56)

# augment the .obj
xpo.line("ATTR_light_level 0.0 1.0 AutoDGS/dgs/vdgs_brightness	7000")

xpo.line("# ---- status == 0, display stand + display UTC time")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 0)

    y = LED(5, 2)
    x = LED(1)      # align with ICAO field to save some TRIS
    dx = 12
    with AnimBlock(xpo):
        xpo.trans_x(-16, 0, 0, 16, "AutoDGS/dgs/r1_scroll") # lateral
        for i in range(0, 6):
            xpo.char_stack(char_txq, x, y, f"AutoDGS/dgs/r1c{i}")
            x += dx

    y = LED(4, 2)
    x = LED(0, 12)
    dx = 12
    xpo.char_stack(char_txq, x, y, "AutoDGS/dgs/time_utc_h1", last = 3, ascii = False)
    x = x + dx
    xpo.char_stack(char_txq, x, y, "AutoDGS/dgs/time_utc_h0", last = 10, ascii = False)
    x = x + dx
    xpo.quad(char_txq[c_colon], x, y)
    x = x + dx * 0.8
    xpo.char_stack(char_txq, x, y, "AutoDGS/dgs/time_utc_m1", last = 6, ascii = False)
    x = x + dx
    xpo.char_stack(char_txq, x, y, "AutoDGS/dgs/time_utc_m0", last = 10, ascii = False)

    with AnimBlock(xpo):
        xpo.show_if_eq("AutoDGS/dgs/boarding", 1)
        xpo.line("# PAX no")
        xpo.quad(pax_txq, LED(1, 8), LED(2, 2))
        x = LED(1, 8)
        y = LED(1, 2)
        dx = 12
        x += 2 * dx
        for i in range(3):
            xpo.char_stack(char_txq, x, y, f"AutoDGS/dgs/paxno_{i}", last = 10, ascii = False)
            x -= dx


def icao_large():
    """helper ICAO in large font"""
    xpo.line("# distance > 10: ICAO large font")
    y = LED(4, 2)
    x = LED(1, 4)
    dx = 12
    xpo.char_stack(char_txq, x, y, icao_0_dr, first = 10, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_1_dr, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_2_dr, last = 36, h2w = 2.0)
    x = x + dx
    xpo.char_stack(char_txq, x, y, icao_3_dr, last = 36, h2w = 2.0)

xpo.line("#---- status = 1 && track = 1 -> lead in")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 1)
    xpo.show_if_eq(track_dr, 1)
    icao_large()

    x = LED(2)
    dy = 10
    y = 2
    degrees = 180 / 5
    for i in range(5):
        with AnimBlock(xpo):
            xpo.show_if_in_range(beacon_dr, i * degrees, (i + 1) * degrees - 0.1)
            xpo.quad(lead_in_txq, x, y)
        y += dy

    y = 2
    for i in range(5, 10):
        with AnimBlock(xpo):
            xpo.show_if_in_range(beacon_dr, i * degrees, (i + 1) * degrees - 0.1)
            xpo.quad(lead_in_txq, x, y)
        y += dy

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
        y = LED(5, 2)
        x = LED(1)
        dx = 12
        xpo.char_stack(char_txq, x, y, icao_0_dr, first = 10, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_1_dr, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_2_dr, last = 36)
        x = x + dx
        xpo.char_stack(char_txq, x, y, icao_3_dr, last = 36)

        # field "9.9m"
        y = LED(4, 1)
        x = LED(1, 8)
        xpo.char_stack(char_txq, x, y, distance_0_dr, last = 9, ascii = False)
        x = x + dx
        xpo.quad(decimal_txq, x, y)
        x = x + dx * 0.5
        xpo.char_stack(char_txq, x, y, distance_01_dr, last = 9, ascii = False)
        x = x + dx
        xpo.quad(char_txq[c_m], x, y)

    xpo.line("# show hbar")
    xpo.quad(hbar_txq, LED(1), LED(3, 12))

    xpo.line("# show vbar + azimuth marker")
    x = LED(2, 5)
    vbar_top = LED(3, 12)

    for i in range(0, 24):  # 12m in 0.5 increments
        with AnimBlock(xpo):
            xpo.hide_if_in_range(distance_dr, 0, i * 0.5 + 0.01)
            xpo.quad(vbar_txq, x, vbar_top - i * 2)

    # azimuth marker
    xazi = LED(2, 8) - azimuth_led_w / 2       # lower left pixel
    yazi = vbar_top - (1 + azimuth_led_h)

    xpo.line("# Azimuth marker")
    with AnimBlock(xpo):
        xpo.trans_y(0, -LED(3), 0.0, 12.0, distance_dr) # below vbar
        with AnimBlock(xpo):
            xpo.trans_x(-16, 16, -4.0, 4.0, xtrack_dr) # lateral
            xpo.quad(azimuth_txq, xazi, yazi)

    xpo.line("# LR arrow right side")
    with AnimBlock(xpo):
        xpo.show_if_eq(lr_dr, 2)
        xpo.hide_if_in_range(sine_wave_dr, -0.91, -0.37)
        xpo.hide_if_in_range(sine_wave_dr, 0.37, 0.91)
        xpo.quad(lr_right_txq, LED(4, 8), LED(3))

    xpo.line("# LR arrow left side")
    with AnimBlock(xpo):
        xpo.show_if_eq(lr_dr, 1)
        xpo.hide_if_in_range(sine_wave_dr, -0.75, -0.25)
        xpo.hide_if_in_range(sine_wave_dr, 0.25, 0.75)
        xpo.quad(lr_left_txq, 0, LED(3))

xpo.line("#---- status = 2 -> red blocks + STOP")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 2)
    xpo.quad(red_block_txq, 0, LED(3))
    xpo.quad(red_block_txq, LED(4), LED(3))
    xpo.quad(stop_txq, LED(1, 8), LED(4), h2w = 2.0)

xpo.line("#---- status = 3 -> OK")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 3)
    xpo.quad(ok_txq, LED(2), LED(4), h2w = 2.0)

xpo.line("#---- status = 4 -> TOO FAR")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 4)
    xpo.quad(too_txq, LED(1, 8), LED(5))
    xpo.quad(far_txq, LED(1, 8), LED(4))

xpo.line("#---- status = 6 -> CHOCK ON")
with AnimBlock(xpo):
    xpo.show_if_eq(status_dr, 6)
    xpo.quad(chock_txq, LED(1, 2), LED(5))
    xpo.quad(on_txq, LED(1, 14), LED(4))

xpo.dump()
