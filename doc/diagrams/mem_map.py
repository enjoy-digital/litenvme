#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Layout of the host-memory window shared by the firmware and the init sequencer (offsets from the base).

from svgdiag import *


def build():
    rows = [
        ("+0x00000", "Admin SQ (ASQ)",             COLOR_ACCENT_LIGHT),
        ("+0x01000", "Admin CQ (ACQ)",             COLOR_ACCENT_LIGHT),
        ("+0x02000", "Identify buffer (firmware)", COLOR_FILL),
        ("+0x03000", "I/O CQ",                     COLOR_ACCENT),
        ("+0x04000", "I/O SQ",                     COLOR_ACCENT),
        ("+0x05000", "Read, write buffers (firmware)", COLOR_FILL),
        ("+0x10000", "Data region: slot buffers or staging", COLOR_ACCENT_LIGHT),
        ("+0x10000 + qsize x 4 KiB", "PRP-list region", COLOR_SHADE),
    ]
    h, x, w = 36, 300, 300
    d = Diagram(CANVAS_WIDTH, 30 + h*len(rows))
    for i, (off, name, fill) in enumerate(rows):
        y = 15 + i*h
        d.add(Box(x, y, w, h, label=name, radius=0, fill=fill, size=14))
        d.add(Text(x - 12, y + h/2, off, size=13, anchor="end"))
    return d
