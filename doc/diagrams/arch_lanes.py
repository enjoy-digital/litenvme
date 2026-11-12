#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Shared helper of the Part II figures (no build(): not a figure).
#
# Lanes (sequence) diagrams: one vertical lane per block (a titled box on top and a dashed line
# below), horizontal arrows between lanes, labels above the arrow next to the source lane, so a
# label never sits on the dashed line of a lane the arrow crosses.

from svgdiag import *


class Lanes:
    """Multi-lane exchange diagram: names[i] centered at xs[i]."""
    def __init__(self, d, names, xs, fills=None, top=15, bottom=None, w=120, step=32):
        self.d, self.xs, self.step = d, xs, step
        fills  = fills or [COLOR_FILL]*len(names)
        bottom = bottom or d.height - 8
        for x, name, fill in zip(xs, names, fills):
            h = 44 if "\n" in name else 34
            d.add(Box(x - w/2, top, w, h, label=name, fill=fill, size=14))
            d.add(Bus([(x, top + h), (x, bottom)], dash="4,3"))
        self.y = top + 44 + 30

    def msg(self, a, b, label, kind="control", step=None, size=13):
        """Arrow from lane a to lane b at the current row; label above, in the gap next to a."""
        y      = self.y
        xa, xb = self.xs[a], self.xs[b]
        nxt    = self.xs[a + (1 if b > a else -1)]
        lx     = (xa + nxt)/2
        self.d.add(Arrow((xa, y), (xb, y), kind=kind, label=label, label_pos=(lx, y - 10),
            label_size=size))
        self.y += step or self.step
        return y

    def note(self, lane, text, side="right", size=12):
        """Free note next to a lane at the current row (no arrow)."""
        y = self.y - 12
        x = self.xs[lane]
        if side == "right":
            self.d.add(Text(x + 8, y, text, size=size, anchor="start", color="#444444"))
        else:
            self.d.add(Text(x - 8, y, text, size=size, anchor="end", color="#444444"))
        self.y += 18

    def gap(self, dy=12):
        self.y += dy
