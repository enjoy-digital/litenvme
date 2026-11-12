#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Integration in a LiteX SoC: PCIe PHY as a root port, LitePCIe root port, PERST# timer, LiteNVMe,
# register master and front-end user logic.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(150, 20, 590, 290, title="LiteX SoC", title_pos="br"))
    ssd   = d.add(Box(20, 200, 100, 70, label="NVMe SSD", fill=COLOR_HARDWARE))
    perst = d.add(Box(175, 60, 130, 60, label="PERST#\ntimer"))
    regs  = d.add(Box(345, 60, 150, 60, label="CPU or\nEtherbone"))
    user  = d.add(Box(600, 60, 115, 60, label="User logic"))
    phy   = d.add(Box(175, 200, 130, 70, label="USPPCIEPHY\nroot port", fill=COLOR_ACCENT_LIGHT))
    rport = d.add(Box(345, 200, 150, 70, label="LitePCIeRootPort", fill=COLOR_ACCENT_LIGHT))
    nvme  = d.add(Box(535, 200, 180, 70, label="LiteNVMe", fill=COLOR_ACCENT))
    d.add(Arrow((ssd, "r"), (phy, "l"), kind="data", heads="both"))
    d.add(Arrow((perst, "b"), (phy, "t"), kind="control", label="reset", label_side="right"))
    d.add(Arrow((175, 90), (70, 200), via=[(70, 90)], kind="control"))
    d.add(Arrow((phy, "r"), (rport, "l"), kind="data", heads="both"))
    d.add(Arrow((rport, "r"), (nvme, "l"), kind="accent", heads="both", label="TLPs", label_side="above"))
    d.add(Arrow((420, 120), (560, 200), via=[(420, 160), (560, 160)], kind="control"))
    d.add(Text(430, 150, "registers", size=FONT_SIZE_SM, anchor="start"))
    d.add(Arrow((user, "b"), (657.5, 200), kind="data", heads="both", label="streams",
        label_side="right"))
    return d
