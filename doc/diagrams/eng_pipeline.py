#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Doorbell coalescing: a burst of SQEs and one SQ doorbell, a batch of CQEs and one CQ doorbell.

from arch_lanes import *


def build():
    d = Diagram(CANVAS_WIDTH, 500)
    l = Lanes(d, ["I/O engine", "Host-memory\nwindow", "SSD"], [130, 390, 640],
        [COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_HARDWARE], w=140, step=30)
    l.msg(0, 1, "SQE, slot 0")
    l.msg(0, 1, "SQE, slot 1")
    l.msg(0, 1, "SQE, slot 2")
    l.msg(0, 2, "SQ doorbell, tail 3", kind="accent")
    l.msg(2, 1, "fetch 3 SQEs")
    l.msg(2, 1, "data (read or write)", kind="data")
    l.msg(2, 1, "CQE, slot 0")
    l.msg(2, 1, "CQE, slot 1")
    l.msg(0, 1, "read CQE 0, 1: phase ok")
    l.msg(0, 1, "read CQE 2: no change")
    l.msg(0, 2, "CQ doorbell, head 2", kind="accent")
    l.msg(2, 1, "CQE, slot 2")
    l.msg(0, 1, "read CQE 2: phase ok")
    l.msg(0, 2, "CQ doorbell, head 3", kind="accent")
    return d
