#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Bring-up steps shared by the firmware and the init sequencer, with the path each one uses.

from svgdiag import *


def build():
    steps = [
        ("1. Configuration: BAR0, MEM, BME",                 "configuration accessor"),
        ("2. Root port: memory window (and MPS)",           "root configuration accessor"),
        ("3. Controller: CAP, AQA, ASQ, ACQ, CC.EN, CSTS.RDY", "MMIO accessor"),
        ("4. Admin: Set Features, Create I/O CQ and SQ",     "window and doorbells"),
        ("5. Engine: queue bases, doorbells, enable",        "engine registers"),
        ("6. init_done",                                     ""),
    ]
    h, step = 46, 62
    d = Diagram(CANVAS_WIDTH, 20 + step*(len(steps) - 1) + h + 15)
    boxes = []
    for i, (name, path) in enumerate(steps):
        y = 15 + i*step
        fill = COLOR_ACCENT if i == len(steps) - 1 else COLOR_FILL
        label = name + ("\n" + path if path else "")
        boxes.append(d.add(Box(170, y, 420, h, label=label, fill=fill, size=14)))
    for a, b in zip(boxes, boxes[1:]):
        d.add(Arrow((a, "b"), (b, "t"), kind="data"))
    return d
