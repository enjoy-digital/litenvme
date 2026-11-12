#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A block write end to end: user data, staging, SQE, doorbell, SSD fetch and data read, CQE, done.

from arch_lanes import *


def build():
    d = Diagram(CANVAS_WIDTH, 470)
    l = Lanes(d, ["User logic", "Block\nstreamer", "I/O engine", "Host-memory\nwindow", "SSD"],
        [80, 230, 380, 530, 680],
        [COLOR_FILL, COLOR_ACCENT_LIGHT, COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_HARDWARE])
    l.msg(0, 1, "start, data beats", kind="data")
    l.msg(1, 3, "fill staging", kind="data")
    l.msg(1, 2, "WRITE request")
    l.msg(2, 3, "SQE")
    l.msg(2, 4, "SQ doorbell", kind="accent")
    l.msg(4, 3, "fetch SQE")
    l.msg(4, 3, "read data", kind="data")
    l.msg(4, 3, "CQE")
    l.msg(2, 3, "read CQE")
    l.msg(2, 4, "CQ doorbell", kind="accent")
    l.msg(2, 1, "completion")
    l.msg(1, 0, "done")
    return d
