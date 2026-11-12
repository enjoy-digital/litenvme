#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A 16-byte CQE written at ACQ+16 on a 256-bit window: shift to dword 4, strobe only its dwords.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    x0, cw, ch = 200, 64, 36
    for i in range(8):
        d.add(Text(x0 + i*cw + cw/2, 22, f"dword {i}", size=12, color="#444444"))
    rows = [
        (40,  "TLP data",      ["D0", "D1", "D2", "D3", "", "", "", ""],  None),
        (130, "Written beat",  ["", "", "", "", "D0", "D1", "D2", "D3"], COLOR_ACCENT),
        (186, "Byte enables",  ["0x0"]*4 + ["0xf"]*4,                      None),
    ]
    for y, name, cells, accent in rows:
        d.add(Text(x0 - 12, y + ch/2, name, size=14, anchor="end"))
        for i, c in enumerate(cells):
            fill = COLOR_SHADE if not c else (accent if (accent and c) else COLOR_FILL)
            if name == "Byte enables":
                fill = COLOR_ACCENT_LIGHT if c == "0xf" else COLOR_FILL
            d.add(Box(x0 + i*cw, y, cw, ch, label=c or " ", radius=0, fill=fill, size=14))
    d.add(Arrow((x0 + 2*cw, 76), (x0 + 6*cw, 130), via=[(x0 + 2*cw, 103), (x0 + 6*cw, 103)],
        kind="accent", label="shift by 4 dwords", label_pos=(x0 + 4*cw, 93)))
    return d
