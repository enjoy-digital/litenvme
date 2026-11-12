#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Who does what in NVMe over PCIe: the host writes registers and doorbells, the SSD reads and writes
# the host memory (queues and data buffers).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 270)
    d.add(Container(20, 20, 330, 235, title="Host", title_pos="bl"))
    d.add(Container(450, 20, 290, 235, title="SSD", title_pos="br"))
    sw   = d.add(Box(40,  40,  140, 50, label="Driver\nCPU or RTL", fill=COLOR_ACCENT_LIGHT))
    rp   = d.add(Box(230, 95,  100, 70, label="Root port", fill=COLOR_ACCENT_LIGHT))
    mem  = d.add(Box(40,  150, 140, 60, label="Host memory\nqueues, data", fill=COLOR_ACCENT))
    regs = d.add(Box(470, 40,  150, 50, label="Controller\nregisters (BAR0)", fill=COLOR_ACCENT))
    ctl  = d.add(Box(470, 150, 150, 60, label="Controller\nlogic, flash", fill=COLOR_HARDWARE))
    d.add(Arrow((sw, "b"), (mem, "t"), kind="data", label="SQE, CQE", label_side="right"))
    d.add(Arrow((sw.x + sw.w, 65), (rp.x + 50, rp.y), via=[(rp.x + 50, 65)], kind="control"))
    d.add(Arrow((mem.x + mem.w, 180), (rp.x + 50, rp.y + rp.h), via=[(rp.x + 50, 180)], kind="data", heads="both"))
    d.add(Arrow((rp.x + rp.w, 115), (regs, "l"), via=[(400, 115), (400, regs.cy)], kind="control", label="MemWr, MemRd", label_pos=(400, 50)))
    d.add(Arrow((ctl, "l"), (rp.x + rp.w, 145), via=[(400, ctl.cy), (400, 145)], kind="accent", heads="both", label="DMA", label_pos=(372, 195)))
    d.add(Arrow((regs, "b"), (ctl, "t"), kind="control", label="doorbell", label_side="right"))
    return d
