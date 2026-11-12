#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Host-memory window: PCIe-side responder and other masters on an AXI arbiter, BRAM backend.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 340)
    rp   = d.add(Box(20, 30, 110, 44, label="Root port", fill=COLOR_HARDWARE))
    resp = d.add(Box(175, 30, 150, 44, label="PCIe-side responder", fill=COLOR_ACCENT, size=14))
    dbg  = d.add(Box(175, 95, 150, 44, label="Register debug port", size=14))
    eng  = d.add(Box(175, 160, 150, 44, label="I/O engine bridge", size=14))
    stg  = d.add(Box(175, 225, 150, 44, label="Staging DMA", size=14))
    ini  = d.add(Box(175, 290, 150, 44, label="Init bridge", size=14))
    arb  = d.add(Box(380, 30, 110, 304, label="AXI\narbiter", fill=COLOR_ACCENT_LIGHT))
    ram  = d.add(Box(560, 130, 170, 104, label="BRAM backend\nbeat wide,\nbyte enables", fill=COLOR_ACCENT))
    d.add(Arrow((rp, "r"), (resp, "l"), kind="data", heads="both"))
    for b in (resp, dbg, eng, stg, ini):
        d.add(Arrow((b, "r"), (arb.x, b.cy), kind="accent" if b is resp else "data"))
    d.add(Arrow((arb.x + arb.w, ram.cy), (ram, "l"), kind="data", heads="both", label="single beats"))
    return d
