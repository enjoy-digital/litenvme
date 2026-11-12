#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Control plane (accessors, set-up) versus data plane (host-memory window, I/O engine).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(10, 10, 390, 125, title="Control plane", title_pos="tl"))
    d.add(Container(10, 155, 390, 135, title="Data plane", title_pos="bl"))
    sw   = d.add(Box(30, 60, 160, 50, label="Firmware or\ninit sequencer"))
    acc  = d.add(Box(230, 60, 145, 50, label="Accessors", fill=COLOR_ACCENT_LIGHT))
    eng  = d.add(Box(30, 175, 160, 50, label="I/O engine", fill=COLOR_ACCENT))
    win  = d.add(Box(230, 175, 145, 50, label="Host-memory\nwindow", fill=COLOR_ACCENT))
    rp   = d.add(Box(430, 30, 90, 240, label="Root\nport", fill=COLOR_HARDWARE))
    ssd  = d.add(Box(640, 30, 100, 240, label="SSD", fill=COLOR_HARDWARE))
    d.add(Arrow((sw, "r"), (acc, "l"), kind="control"))
    d.add(Arrow((acc, "r"), (rp.x, acc.cy), kind="data", heads="both"))
    d.add(Arrow((rp.x + rp.w, acc.cy), (ssd.x, acc.cy), kind="data", label="Cfg, BAR0", label_pos=(580, acc.cy - 12)))
    d.add(Arrow((eng, "r"), (win, "l"), kind="accent", heads="both"))
    d.add(Arrow((win, "r"), (rp.x, win.cy), kind="data", heads="both"))
    d.add(Arrow((ssd.x, win.cy), (rp.x + rp.w, win.cy), kind="data", label="SSD DMA", label_pos=(580, win.cy - 12)))
    d.add(Arrow((eng.x + 80, eng.y), (acc.x + 30, acc.y + acc.h), via=[(110, 146), (260, 146)],
        kind="control", label="doorbells", label_pos=(302, 146)))
    return d
