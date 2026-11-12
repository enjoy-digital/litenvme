#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# One Read command as PCIe transactions between the host and the SSD.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 400)
    e = Exchange(d, "Host", "SSD", x_left=250, x_right=560)
    e.note("SQE into SQ slot", side="left", step=30)
    e.msg("MemWr SQ tail doorbell", to="right", note="new command")
    e.msg("MemRd SQE, 64 bytes", to="left", note="fetch")
    e.msg("CplD SQE", to="right")
    e.gap()
    e.msg("MemWr data", to="left", kind="accent", note="into PRP pages")
    e.msg("MemWr data", to="left", kind="accent")
    e.msg("MemWr CQE, 16 bytes", to="left", note="phase flipped")
    e.gap()
    e.note("poll CQE phase", side="left", step=30)
    e.msg("MemWr CQ head doorbell", to="right", note="entry consumed")
    return d
