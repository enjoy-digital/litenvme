#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The 16-byte completion queue entry and its status field.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 220)
    A, L, S, F = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE, COLOR_FILL
    x0, w, h = 150, 440, 28
    rows = [
        ("dword 0", [(440, "command specific", F)]),
        ("dword 1", [(440, "reserved", S)]),
        ("dword 2", [(220, "SQ ID", F), (220, "SQHD: SQ head", L)]),
        ("dword 3", [(220, "status, phase", A), (220, "CID", A)]),
    ]
    d.add(Text(x0, 12, "bit 31", size=12, anchor="start"))
    d.add(Text(x0 + w, 12, "bit 0", size=12, anchor="end"))
    y = 26
    for label, cells in rows:
        d.add(Text(x0 - 12, y + h/2, label, size=13, anchor="end"))
        bitfield(d, x0, y, cells, h=h, size=13)
        y += h
    y += 40
    d.add(Text(x0 - 12, y + h/2, "dword 3 [31:16]", size=13, anchor="end"))
    cells = [(60, "DNR 31", L), (50, "M 30", F), (80, "CRD 29:28", S), (85, "SCT 27:25", A), (120, "SC 24:17", A), (45, "P 16", A)]
    bitfield(d, x0, y, cells, h=h, size=13)
    d.add(Bus([(x0 + 110, 26 + 4*h), (x0 + 110, y)], dash="4,3"))
    return d
