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

class XPObj:
    class VT:
        def __init__(self, x, y, z, s, t):
            self.x = x
            self.y = y
            self.z = z
            self.s = s
            self.t = t

        def __str__(self):
            return f"VT {self.x:8.5f} {self.y:8.5f} {self.z:8.5f} 0 0 1 {self.s:6.5f} {self.t:6.5f}\n"

    def __init__(self, base_obj_fn, b_x, b_y, b_z, w, h):
        """
            b_(x,y,z) blender coordinates of lower left corner of display
            w, h width and height of display (all in m)
        """
        self.scale = 1
        self.x0 = b_x           # xform to local coordinate system
        self.y0 = b_z
        self.z0 = -b_y + 0.003   # in front of the display

        self.new_obj_fn = base_obj_fn.replace(".obj-base", ".obj")
        self.vt_table = []
        self.idx_table = []
        self.line_table = []

        # base object
        self.bo_preamble = []
        self.bo_vt_lines = []
        self.bo_n_vt = 0
        self.bo_idx_lines = []
        self.bo_n_idx = 0
        self.bo_body = []

        # indentation control
        self.indent = 0
        self.indent_str = ""

        # statistics
        self.optim_vts = 0
        self.optim_tris = 0
        self.anim_blocks = 0
        self.hides = 0

        bo_lines = open(base_obj_fn, "r").readlines()

        i = 0
        n = len(bo_lines)
        while i < n:
            l = bo_lines[i]
            if l.startswith("VT"):
                break
            if not l.startswith("POINT_COUNTS"):
                self.bo_preamble.append(l)
            i = i + 1

        while i < n:
            l = bo_lines[i]
            l = l.strip()
            if l == "":
                i = i + 1
                continue

            if not l.startswith("VT"):
                break

            self.bo_vt_lines.append(l)
            i = i + 1

        self.bo_n_vt = len(self.bo_vt_lines)

        while i < n:
            l = bo_lines[i]
            l = l.strip()
            if l == "":
                i = i + 1
                continue

            if not l.startswith("IDX"):
                break

            self.bo_idx_lines.append(l)
            i = i + 1
            if l.startswith("IDX10"):
                self.bo_n_idx += 10
            else:
                self.bo_n_idx += 1

        self.bo_body = bo_lines[i:]
        # print(f"base_obj VT: {self.bo_n_vt}, IDX: {self.bo_n_idx}")
        # print(f"{self.bo_body}")

    def _indent_inc(self):
        self.indent += 4
        self.indent_str = ' ' * self.indent

    def _indent_dec(self):
        self.indent -= 4
        self.indent_str = ' ' * self.indent

    def _add_vt(self, x, y, s, t) -> int:
        x = x + self.x0
        y = y + self.y0
        z = self.z0
        i = 0
        for v in self.vt_table:
            if v.x == x and v.y == y and v.z == z and v.s == s and v.t == t:
                self.optim_vts += 1
                return self.bo_n_vt + i
            i += 1

        idx = self.bo_n_vt + len(self.vt_table)
        self.vt_table.append(self.VT(x, y, z, s, t))
        return idx

    def set_scale(self, scale):
        """factor user units -> m"""
        self.scale = scale

    def line(self, line):
        self.line_table.append(self.indent_str + line)

    def quad(self, txq, x, y, h2w = 1.0):
        x = x * self.scale
        y = y * self.scale
        w = txq.w * txq.tex.pix2m
        h = txq.h * txq.tex.pix2m * h2w

        vt0 = self._add_vt(x, y, txq.s1, txq.t1)
        vt1 = self._add_vt(x, y + h, txq.s1, txq.t2)
        vt2 = self._add_vt(x + w, y, txq.s2, txq.t1)
        vt3 = self._add_vt(x + w, y + h, txq.s2, txq.t2)

        # tris are clockwise
        vts = [vt0, vt1, vt2, vt1, vt3, vt2]
        for i in range(0, len(self.idx_table), 6):
            if vts == self.idx_table[i : i + 6]:
                idx = i + self.bo_n_idx
                self.line(f"TRIS {idx} 6")
                self.optim_tris += 1
                return

        idx = self.bo_n_idx + len(self.idx_table)
        self.idx_table.extend(vts)
        self.line(f"TRIS {idx} 6")

    def char_stack(self, char_txq, x, y, dref, first = 0, last = 999, ascii = True, h2w = 1.0):
        self._indent_inc()
        self.line(f"# char stack {dref} @{x, y}")
        self.line("\n")

        n = min(last + 1, len(char_txq))
        for i in range(first, n):
            v = i
            if ascii:
                # dref is in ascii code
                if 0 <= i and i <= 9:
                    v += 48
                elif 10 <= i and i <= 35:
                    v += -10 + 65
                elif i == 37:
                    v = ord(':')
                elif i == 38:
                    v = ord('.')

            tx = char_txq[i]
            self.line(f"# char {i}")
            self.line("ANIM_begin")
            self._indent_inc()
            self.line(f"ANIM_hide -2 {v - 1} {dref}")
            self.line(f"ANIM_hide {v + 1} 1000 {dref}")
            self.quad(tx, x, y, h2w = h2w)
            self._indent_dec()
            self.line("ANIM_end\n")
            self.anim_blocks += 1
            self.hides += 2
            i += 1

        self._indent_dec()

    def hide_if_in_range(self, dref, low, high):
        self.line(f"ANIM_hide {low:0.4f} {high:0.4f} {dref}")
        self.hides += 1

    def show_if_in_range(self, dref, low, high):
        self.hide_if_in_range(dref, - 10000, low - 0.001)
        self.hide_if_in_range(dref, high + 0.001, 10000)
        self.hides += 2

    def show_if_eq(self, dref, val):
        self.show_if_in_range(dref, val, val)

    def trans_x(self, dx_left, dx_right, v1, v2, dref):
        """move lateral from dx_left to dx_right while dref varies between v1 and v2"""
        dx_left = dx_left * self.scale
        dx_right = dx_right * self.scale
        self.line(f"ANIM_trans {dx_left:0.4f} 0 0 {dx_right:0.4f} 0 0 {v1} {v2} {dref}")

    def trans_y(self, dy_bottom, dy_top, v1, v2, dref):
        """move verical from dy_bottom to dy_top while dref varies between v1 and v2"""
        dy_bottom = dy_bottom * self.scale
        dy_top = dy_top * self.scale
        self.line(f"ANIM_trans 0 {dy_bottom:0.4f} 0 0 {dy_top:0.4f} 0 {v1} {v2} {dref}")

    def dump(self):
        with open(self.new_obj_fn, "w", newline = '\n') as f:
            for l in self.bo_preamble:
                f.write(l)

            nvt = self.bo_n_vt + len(self.vt_table)
            nidx = self.bo_n_idx + len(self.idx_table)
            f.write(f"POINT_COUNTS {nvt} 0 0 {nidx}\n")

            f.write("\n# base obj\n")
            for l in self.bo_vt_lines:
                f.write(l + "\n")

            f.write("\n# display\n")
            for v in self.vt_table:
                f.write(str(v))

            f.write("\n# base obj\n")
            for l in self.bo_idx_lines:
                f.write(l + "\n")

            f.write("\n# display\n")
            n = len(self.idx_table)
            i = 0
            while n > 10:
                f.write("IDX10")
                for j in range(i, i + 10):
                    f.write(f" {self.idx_table[j]}")
                f.write("\n")
                i += 10
                n -= 10

            for idx in self.idx_table[i:]:
                f.write(f"IDX {idx}\n")

            # place display at the end of first LOD section or the very end
            f.write("\n# base obj\n")
            n_LOD = 0
            display_added = False
            for l in self.bo_body:
                if "LOD" in l:
                    n_LOD += 1
                    if n_LOD == 2:
                        display_added = True
                        f.write("\n# display\n")
                        for ld in self.line_table:
                            f.write(ld + "\n")
                        f.write("\n# continue base obj\n")

                f.write(l)

            if not display_added:
                f.write("\n# display\n")
                for ld in self.line_table:
                    f.write(ld + "\n")
                f.write("\n# continue base obj\n")

        print(f"obj written to '{self.new_obj_fn}'\n")
        print(f"VTs  created:        {len(self.vt_table):4}")
        print(f"TRIs created:        {int(len(self.idx_table)/3):4}")
        print(f"VTs  optimized:      {self.optim_vts:4}")
        print(f"TRIs optimized:      {self.optim_tris:4}")
        print(f"Anim blocks created: {self.anim_blocks:4}")
        print(f"ANIM_hide   created: {self.hides:4}")

class Texture:
    def __init__(self, xpo, w, h, pix2m):
        self.xpo = xpo
        self.w = w
        self.h = h
        self.pix2m = pix2m

class TexQuad:
    def __init__(self, tex, x, y, w, h, mirror_x = False):
        """in pixels with (0,0) = upper left corner = rectangle selection in GIMP"""
        self.tex = tex
        self.w = w
        self.h = h
        self.s1 = x / tex.w
        self.t1 = (tex.h - (y + h)) / tex.h
        self.s2 = (x + w) / tex.w
        self.t2 = (tex.h - y) / tex.h
        if mirror_x:
            s = self.s2
            self.s2 = self.s1
            self.s1 = s

        #print(f"{self.s1:0.2f} {self.t1:0.2f}")

class AnimBlock:
    """manage a "ANIM_begin / ANIM_end" block"""

    def __init__(self, xpo):
        self.xpo = xpo

    def __enter__(self):
        self.xpo.anim_blocks += 1
        self.xpo.line("ANIM_begin")
        self.xpo._indent_inc()

    def __exit__(self, type, value, traceback):
        self.xpo._indent_dec()
        self.xpo.line("ANIM_end")
