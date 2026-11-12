#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# I/O engine state machine: IDLE picks one of four paths in priority order, each returns to IDLE.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 350)
    idle = d.add(Box(30, 140, 110, 60, label="IDLE", fill=COLOR_ACCENT))
    paths = [
        ("1. request, room, CID free", "Write one SQE\nLATCH-REQ to SUBMIT-ADVANCE", COLOR_ACCENT_LIGHT),
        ("2. SQ tail moved",           "Ring SQ doorbell\nSUBMIT-DOORBELL(-WAIT)",    COLOR_FILL),
        ("3. commands in flight",      "Reap a CQE batch\nREAP-READ to REAP-EMIT",    COLOR_ACCENT_LIGHT),
        ("4. CQ head moved",           "Ring CQ doorbell\nREAP-DOORBELL(-WAIT)",      COLOR_FILL),
    ]
    trunk, bx, bw, bh = 175, 400, 300, 52
    boxes = []
    for i, (cond, label, fill) in enumerate(paths):
        y = 20 + i*75
        b = d.add(Box(bx, y, bw, bh, label=label, fill=fill, size=14))
        boxes.append(b)
        d.add(Arrow((trunk, b.cy), (b, "l"), kind="data", label=cond, label_pos=(287, b.cy - 11)))
    d.add(Bus([(idle.x + idle.w, idle.cy), (trunk, idle.cy)], kind="data"))
    d.add(Bus([(trunk, boxes[0].cy), (trunk, boxes[-1].cy)], kind="data"))
    # Return to IDLE (dotted): every path ends in IDLE (the reap path after its last CQE).
    rx = 730
    for b in boxes:
        d.add(Bus([(b.x + bw, b.cy), (rx, b.cy)], dash=DASH_FEEDBACK))
    d.add(Bus([(rx, boxes[0].cy), (rx, 325)], dash=DASH_FEEDBACK))
    d.add(Arrow((rx, 325), (idle.cx, idle.y + idle.h), via=[(idle.cx, 325)], kind="feedback",
        label="back to IDLE", label_pos=(560, 313)))
    return d
