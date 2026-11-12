#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LiteNVMe at a glance: the SSD, the LitePCIe root port, the core and the logic that uses it.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(130, 20, 610, 255, title="FPGA", title_pos="br"))
    d.add(Container(275, 35, 285, 200, title="LiteNVMe", title_pos="bl"))
    ssd = d.add(Box(20,  110, 90,  60, label="NVMe SSD", fill=COLOR_HARDWARE))
    rp  = d.add(Box(150, 105, 100, 70, label="LitePCIe\nroot port", fill=COLOR_ACCENT_LIGHT))
    acc = d.add(Box(295, 50,  120, 45, label="Accessors", fill=COLOR_ACCENT))
    win = d.add(Box(295, 135, 120, 50, label="Host-memory\nwindow", fill=COLOR_ACCENT))
    eng = d.add(Box(435, 90,  110, 50, label="I/O engine", fill=COLOR_ACCENT))
    cpu = d.add(Box(585, 45,  140, 50, label="CPU or init\nsequencer"))
    usr = d.add(Box(585, 150, 140, 50, label="User logic"))
    d.add(Arrow((ssd, "r"), (rp, "l"), kind="data", heads="both", label="PCIe", label_pos=(75, 95)))
    d.add(Arrow((rp.x + rp.w, 125), (acc, "l"), via=[(265, 125), (265, acc.cy)], kind="data", heads="both"))
    d.add(Arrow((rp.x + rp.w, 155), (win, "l"), via=[(265, 155), (265, win.cy)], kind="data", heads="both"))
    d.add(Arrow((eng.x, 105), (acc, "r"), via=[(425, 105), (425, acc.cy)], kind="control"))
    d.add(Arrow((eng.x, 125), (win, "r"), via=[(425, 125), (425, win.cy)], kind="data", heads="both"))
    d.add(Arrow((cpu.x, 60), (acc.x + acc.w, 60), kind="control", label="registers", label_side="above"))
    d.add(Arrow((usr, "l"), (eng.x + 60, eng.y + eng.h), via=[(eng.x + 60, usr.cy)], kind="data", heads="both", label="blocks", label_pos=(530, usr.cy - 10)))
    return d
