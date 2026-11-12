#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LiteNVMe core blocks: accessors, I/O engine, host-memory window and front-end on the root port.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 360)
    rp   = d.add(Box(20, 30, 110, 300, label="LitePCIe\nroot port", fill=COLOR_HARDWARE))
    d.add(Container(150, 15, 465, 330, title="LiteNVMe", title_pos="br"))
    cfg  = d.add(Box(170, 35,  135, 46, label="Configuration\naccessor"))
    mmio = d.add(Box(170, 100, 135, 46, label="MMIO\naccessor"))
    db   = d.add(Box(170, 165, 135, 46, label="Doorbell\naccessor", fill=COLOR_ACCENT_LIGHT))
    win  = d.add(Box(170, 240, 135, 60, label="Host-memory\nwindow", fill=COLOR_ACCENT))
    eng  = d.add(Box(380, 150, 100, 76, label="I/O engine", fill=COLOR_ACCENT))
    fe   = d.add(Box(505, 150, 95, 76, label="Front-end\n(optional)"))
    user = d.add(Box(640, 150, 100, 76, label="User\nlogic"))
    # Accessors send requests towards the SSD; the window receives the requests of the SSD.
    for b in (cfg, mmio, db):
        d.add(Arrow((b, "l"), (rp.x + rp.w, b.cy), kind="data", heads="end"))
    d.add(Arrow((rp.x + rp.w, win.cy), (win, "l"), kind="accent", heads="end"))
    d.add(Arrow((eng.x, db.cy), (db, "r"), kind="control", label="doorbells", label_pos=(342, db.cy - 10)))
    d.add(Arrow((eng.x + 30, eng.y + eng.h), (win.x + win.w, win.y + 20), via=[(410, 260)],
        kind="accent", heads="both", label="SQE, CQE", label_pos=(357, 249)))
    d.add(Arrow((fe, "l"), (eng, "r"), kind="data", heads="both"))
    d.add(Arrow((fe.x + 47, fe.y + fe.h), (win.x + win.w, win.y + 45), via=[(552, 285)],
        kind="data", heads="both", label="staging", label_pos=(480, 274)))
    d.add(Arrow((user, "l"), (fe, "r"), kind="data", heads="both"))
    return d
