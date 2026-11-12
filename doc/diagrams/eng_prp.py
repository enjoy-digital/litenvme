#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PRP list of a 16 KiB transfer: PRP1 to page 0, PRP2 to the list page of the slot, list to pages 1-3.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    ch = 40
    d.add(Text(110, 22, "SQE", size=14, italic=True))
    d.add(Text(370, 22, "PRP-list page of the slot", size=14, italic=True))
    d.add(Text(650, 22, "Data buffer", size=14, italic=True))
    prp1 = d.add(Box(20, 40, 180, ch, label="PRP1 = buf", radius=0, fill=COLOR_ACCENT_LIGHT, size=14))
    prp2 = d.add(Box(20, 80, 180, ch, label="PRP2 = list page", radius=0, fill=COLOR_ACCENT_LIGHT, size=14))
    ents, pages = [], []
    for i in range(4):
        pages.append(d.add(Box(570, 40 + i*ch, 160, ch, label=f"Page {i}", radius=0,
            fill=COLOR_ACCENT if i == 0 else COLOR_FILL, size=14)))
    for i in range(3):
        ents.append(d.add(Box(290, 80 + i*ch, 160, ch, label=f"buf + {4*(i + 1)} KiB", radius=0,
            fill=COLOR_SHADE, size=14)))
    d.add(Arrow((prp1, "r"), (pages[0], "l"), kind="accent"))
    d.add(Arrow((prp2, "r"), (ents[0], "l"), kind="data"))
    for e, p in zip(ents, pages[1:]):
        d.add(Arrow((e, "r"), (p, "l"), kind="data"))
    return d
