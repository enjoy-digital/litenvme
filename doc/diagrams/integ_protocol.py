#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Block interface of the standalone core: one write then one read, as seen from user logic.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 420)
    xl, xr, top, bottom = 200, 520, 20, 410
    for x, name, fill in ((xl, "User logic", COLOR_FILL), (xr, "litenvme_core", COLOR_ACCENT)):
        d.add(Box(x - 70, top, 140, 36, label=name, fill=fill))
        d.add(Bus([(x, top + 36), (x, bottom)], dash="4,3"))
    y = [92, 132, 172, 212, 262, 302, 342, 382]
    def msg(i, label, to, kind="control", note=None):
        a, b = ((xl, y[i]), (xr, y[i])) if to == "right" else ((xr, y[i]), (xl, y[i]))
        d.add(Arrow(a, b, kind=kind, label=label, label_side="above", label_size=14))
        if note:
            d.add(Text(xr + 12, y[i], note, size=13, anchor="start", color="#444444"))
    msg(0, "status_init_done = 1", "left", note="after bring-up")
    msg(1, "start, write = 1, sector, count", "right")
    msg(2, "write data, count x 16 beats", "right", kind="accent", note="fills staging")
    msg(3, "done = 1, error", "left", note="after the WRITE CQE")
    msg(4, "start, write = 0, sector, count", "right")
    msg(5, "busy = 1", "left", note="READ command")
    msg(6, "read data, count x 16 beats", "left", kind="accent", note="drains staging")
    msg(7, "done = 1, error", "left")
    d.add(Text(xl - 14, (y[1] + y[3])/2, "write", size=15, anchor="end", italic=True))
    d.add(Text(xl - 14, (y[4] + y[7])/2, "read", size=15, anchor="end", italic=True))
    return d
