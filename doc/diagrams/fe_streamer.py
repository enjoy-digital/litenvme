#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Block streamer: control to the I/O engine, data through the staging region of the window.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    user = d.add(Box(10, 40, 80, 230, label="User\nlogic"))
    d.add(Container(205, 15, 245, 275, title="Block streamer", title_pos="br"))
    fsm  = d.add(Box(225, 40, 200, 60, label="Command FSM", fill=COLOR_ACCENT))
    dma  = d.add(Box(225, 150, 200, 60, label="Staging DMA", fill=COLOR_ACCENT_LIGHT))
    eng  = d.add(Box(530, 40, 130, 60, label="I/O engine"))
    win  = d.add(Box(530, 150, 210, 60, label="Host-memory window\n(staging region)", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((user.x + user.w, 60), (fsm.x, 60), kind="control", label="start", label_pos=(147, 49)))
    d.add(Arrow((fsm.x, 85), (user.x + user.w, 85), kind="control", label="done, error", label_pos=(147, 74)))
    d.add(Arrow((user.x + user.w, 165), (dma.x, 165), kind="data", label="wr_sink", label_pos=(147, 154)))
    d.add(Arrow((dma.x, 195), (user.x + user.w, 195), kind="data", label="rd_source", label_pos=(147, 184)))
    d.add(Arrow((fsm.x + fsm.w, 60), (eng.x, 60), kind="control", label="request", label_pos=(490, 49)))
    d.add(Arrow((eng.x, 85), (fsm.x + fsm.w, 85), kind="control", label="completion", label_pos=(490, 74)))
    d.add(Arrow((dma, "r"), (win, "l"), kind="data", heads="both", label="AXI", label_pos=(490, 169)))
    d.add(Arrow((fsm, "b"), (dma, "t"), kind="control", label="fill, drain", label_side="right"))
    d.add(Arrow((eng.x + 70, eng.y + eng.h), (eng.x + 70, win.y), kind="control", heads="both",
        label="SQE, CQE", label_side="right"))
    return d
