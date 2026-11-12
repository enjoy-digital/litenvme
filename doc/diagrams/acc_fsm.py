#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# State machine of the shared accessor (configuration and MMIO accessors).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    w, h, y0, y1 = 130, 44, 40, 150
    idle  = d.add(Box(30,  y0, w, h, label="IDLE"))
    send  = d.add(Box(220, y0, w, h, label="SEND", fill=COLOR_ACCENT))
    wait  = d.add(Box(410, y0, w, h, label="WAIT", fill=COLOR_ACCENT))
    latch = d.add(Box(600, y0, w, h, label="LATCH"))
    bad   = d.add(Box(30,  y1, w, h, label="BAD-WSEL"))
    wlat  = d.add(Box(220, y1, w, h, label="WRITE-LATCH"))
    tmo   = d.add(Box(410, y1, w, h, label="TIMEOUT"))
    d.add(Arrow((idle, "r"), (send, "l"), kind="data", label="start"))
    d.add(Arrow((send, "r"), (wait, "l"), kind="data", label="accepted"))
    d.add(Arrow((wait, "r"), (latch, "l"), kind="data", label="end beat"))
    d.add(Arrow((idle, "b"), (bad, "t"), kind="control", label="write, wsel not 0xf", label_side="right"))
    d.add(Arrow((send, "b"), (wlat, "t"), kind="control", label="posted write", label_side="right"))
    d.add(Arrow((wait, "b"), (tmo, "t"), kind="control", label="2^20 cycles", label_side="right"))
    # Every final state sets done (and err for the errors) and returns to IDLE (dotted trunk).
    trunk = 250
    for b in (bad, wlat, tmo):
        d.add(Bus([(b.cx, b.y + h), (b.cx, trunk)], dash=DASH_FEEDBACK))
    d.add(Bus([(latch.cx, latch.y + h), (latch.cx, trunk)], dash=DASH_FEEDBACK))
    d.add(Bus([(bad.cx, trunk), (latch.cx, trunk)], dash=DASH_FEEDBACK))
    d.add(Arrow((bad.cx, trunk), (idle.x - 15, idle.cy), via=[(idle.x - 15, trunk)], kind="feedback"))
    d.add(Arrow((idle.x - 15, idle.cy), (idle, "l"), kind="feedback"))
    d.add(Text(380, trunk + 18, "done = 1, back to IDLE", size=13, color="#444444"))
    return d
