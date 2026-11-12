#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Standalone Verilog core (litenvme_gen): port groups of the generated module.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 370)
    d.add(Container(240, 20, 270, 335, title="litenvme_core", title_pos="bl"))
    phy  = d.add(Box(270, 50, 210, 80, label="PCIe PHY, root port\nPERST# timer", fill=COLOR_ACCENT_LIGHT))
    nvme = d.add(Box(270, 160, 210, 90, label="LiteNVMe", fill=COLOR_ACCENT))
    init = d.add(Box(270, 275, 210, 40, label="CPU or init sequencer", fill=COLOR_ACCENT_LIGHT))
    ref  = d.add(Box(20, 50, 160, 24, label="pcie_clk_p/n", size=13, fill=COLOR_HARDWARE))
    lane = d.add(Box(20, 78, 160, 24, label="pcie_rx/tx_p/n", size=13, fill=COLOR_HARDWARE))
    rstn = d.add(Box(20, 106, 160, 24, label="rst_n (PERST#)", size=13, fill=COLOR_HARDWARE))
    uart = d.add(Box(20, 283, 160, 24, label="serial_tx/rx", size=13))
    clk  = d.add(Box(570, 78, 170, 24, label="clk, rst", size=13))
    ctrl = d.add(Box(570, 165, 170, 24, label="block_ctrl_*", size=13))
    wr   = d.add(Box(570, 193, 170, 24, label="block_wr_axis_*", size=13))
    rd   = d.add(Box(570, 221, 170, 24, label="block_rd_axis_*", size=13))
    stat = d.add(Box(570, 283, 170, 24, label="status_init_*", size=13))
    d.add(Arrow((ref, "r"), (270, 62), kind="control"))
    d.add(Arrow((lane, "r"), (270, 90), kind="data", heads="both"))
    d.add(Arrow((270, 118), (rstn, "r"), kind="control"))
    d.add(Arrow((uart, "r"), (init, "l"), kind="control", heads="both"))
    d.add(Arrow((480, 90), (clk, "l"), kind="control"))
    d.add(Arrow((phy, "b"), (nvme, "t"), kind="accent", heads="both"))
    d.add(Arrow((init, "t"), (nvme, "b"), kind="control"))
    d.add(Arrow((ctrl, "l"), (480, 177), kind="control", heads="both"))
    d.add(Arrow((wr, "l"), (480, 205), kind="data"))
    d.add(Arrow((480, 233), (rd, "l"), kind="data"))
    d.add(Arrow((init, "r"), (stat, "l"), kind="control"))
    return d
