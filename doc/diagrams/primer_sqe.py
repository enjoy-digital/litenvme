#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The 64-byte submission queue entry of a Read or a Write, as LiteNVMe fills it.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    A, L, S, F = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE, COLOR_FILL
    x0, w, h = 150, 400, 26
    rows = [
        ("dword 0",     [(160, "CID", A), (120, "flags", S), (120, "opcode", A)]),
        ("dword 1",     [(400, "NSID: namespace", L)]),
        ("dwords 2-5",  [(400, "reserved, metadata (zero)", S)]),
        ("dwords 6-7",  [(400, "PRP1: first data page", A)]),
        ("dwords 8-9",  [(400, "PRP2: second page or list", A)]),
        ("dwords 10-11", [(400, "SLBA: first sector", L)]),
        ("dword 12",    [(160, "flags (zero)", S), (240, "NLB: sectors - 1", L)]),
        ("dwords 13-15", [(400, "command specific (zero)", S)]),
    ]
    d.add(Text(x0, 12, "bit 31", size=12, anchor="start"))
    d.add(Text(x0 + w, 12, "bit 0", size=12, anchor="end"))
    y = 28
    for label, cells in rows:
        d.add(Text(x0 - 12, y + h/2, label, size=13, anchor="end"))
        bitfield(d, x0, y, cells, h=h, size=13)
        y += h + 4
    d.add(Text(x0 + w + 14, 28 + h/2, "0x02 Read, 0x01 Write", size=13, anchor="start", color="#444444"))
    return d
