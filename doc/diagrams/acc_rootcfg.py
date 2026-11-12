#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Root configuration accessor: sys-domain request, pulse synchronizers, pcie-domain FSM, cfg_mgmt.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 270)
    regs  = d.add(Box(20, 40, 150, 180, label="Registers or\ninit sequencer"))
    s_go  = d.add(Box(225, 40, 140, 44, label="Start sync", fill=COLOR_ACCENT_LIGHT))
    s_dn  = d.add(Box(225, 116, 140, 44, label="Done sync, rdata", fill=COLOR_ACCENT_LIGHT))
    fsm   = d.add(Box(420, 40, 110, 120, label="Transaction\nFSM", fill=COLOR_ACCENT))
    hip   = d.add(Box(625, 40, 115, 180, label="Hard IP\ncfg_mgmt", fill=COLOR_HARDWARE))
    d.add(Bus([(295, 15), (295, 250)], dash=DASH_FEEDBACK))
    d.add(Text(250, 245, "sys", size=14, italic=True, color="#444444"))
    d.add(Text(340, 245, "pcie", size=14, italic=True, color="#444444"))
    d.add(Arrow((regs.x + regs.w, s_go.cy), (s_go, "l"), kind="control", label="start"))
    d.add(Arrow((s_go, "r"), (fsm.x, s_go.cy), kind="control"))
    d.add(Arrow((fsm.x + fsm.w, s_go.cy), (hip.x, s_go.cy), kind="control", label="read, write"))
    d.add(Arrow((hip.x, s_dn.cy), (fsm.x + fsm.w, s_dn.cy), kind="control", label="done, data"))
    d.add(Arrow((fsm.x, s_dn.cy), (s_dn, "r"), kind="control"))
    d.add(Arrow((s_dn, "l"), (regs.x + regs.w, s_dn.cy), kind="control", label="done"))
    d.add(Arrow((regs.x + regs.w, 195), (hip.x, 195), kind="data", label="address, data, byte enables",
        label_pos=(452, 183)))
    return d
