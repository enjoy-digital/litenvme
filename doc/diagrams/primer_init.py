#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Controller initialization as seen on the link: configuration, registers, admin commands.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 440)
    e = Exchange(d, "Host", "SSD", x_left=250, x_right=560)
    e.msg("CfgWr BAR0, Command", to="right", note="MEM, BME")
    e.msg("MemRd CAP", to="right", note="limits, stride")
    e.msg("MemWr AQA, ASQ, ACQ", to="right", note="admin queue")
    e.msg("MemWr CC.EN = 1", to="right", note="enable")
    e.msg("MemRd CSTS", to="right", note="until RDY = 1")
    e.gap()
    e.msg("Identify (optional)", to="right", kind="accent", note="admin command")
    e.msg("Set Features", to="right", kind="accent", note="number of queues")
    e.msg("Create I/O CQ", to="right", kind="accent")
    e.msg("Create I/O SQ", to="right", kind="accent")
    e.note("I/O queue ready", side="left", step=0)
    return d
