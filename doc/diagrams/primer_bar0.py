#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The registers of an NVMe controller in BAR0: the fixed registers from 0x00, the doorbells from
# 0x1000, spaced by the doorbell stride.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    x0, w, h = 190, 300, 26
    A, L, S, F = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE, COLOR_FILL
    rows = [
        ("0x00", "CAP: capabilities (64 bits)", L, "MQES, DSTRD, TO"),
        ("0x08", "VS: version", F, None),
        ("0x14", "CC: configuration", A, "EN, page size, entry sizes"),
        ("0x1c", "CSTS: status", A, "RDY"),
        ("0x24", "AQA: admin queue sizes", L, None),
        ("0x28", "ASQ: admin SQ base (64 bits)", L, None),
        ("0x30", "ACQ: admin CQ base (64 bits)", L, None),
        (None, "other registers", S, None),
        ("0x1000", "SQ 0 tail doorbell", A, "admin queue"),
        ("+ stride", "CQ 0 head doorbell", A, None),
        ("+ 2 strides", "SQ 1 tail doorbell", A, "I/O queue 1"),
        ("+ 3 strides", "CQ 1 head doorbell", A, None),
    ]
    y = 20
    for offset, label, fill, note in rows:
        if offset:
            d.add(Text(x0 - 12, y + h/2, offset, size=13, anchor="end"))
        bitfield(d, x0, y, [(w, label, fill)], h=h, size=13)
        if note:
            d.add(Text(x0 + w + 14, y + h/2, note, size=13, anchor="start", color="#444444"))
        y += h + (8 if offset is None else 0)
    d.add(Text(x0 + w/2, y + 22, "stride = 4 << CAP.DSTRD bytes", size=13))
    return d
