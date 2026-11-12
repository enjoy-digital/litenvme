#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PRP entries: PRP1 points at the first data page; PRP2 at the second page, or at a list page that
# points at the following pages.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 280)
    A, L, S, F = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE, COLOR_FILL
    d.add(Container(20, 20, 160, 245, title="SQE", title_pos="bl"))
    p1 = d.add(Box(40, 45,  120, 40, label="PRP1", fill=A))
    p2 = d.add(Box(40, 125, 120, 40, label="PRP2", fill=A))
    d.add(Container(250, 20, 160, 245, title="PRP list", title_pos="bl"))
    ents = [d.add(Box(270, 85 + i*40, 120, 40, label=f"entry {i}", radius=0, fill=L)) for i in range(3)]
    pages = [d.add(Box(500, 45 + i*40, 200, 40, label=f"data page {i}", radius=0, fill=F)) for i in range(4)]
    d.add(Arrow((p1, "r"), (pages[0], "l"), kind="accent"))
    d.add(Arrow((p2, "r"), (ents[0], "l"), via=[(220, p2.cy), (220, ents[0].cy)], kind="accent"))
    for e, p in zip(ents, pages[1:]):
        d.add(Arrow((e, "r"), (p, "l"), kind="control"))
    d.add(Text(600, 228, "4 KiB each", size=13, color="#444444"))
    return d
