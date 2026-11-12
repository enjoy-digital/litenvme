#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Init sequencer: the step chain from the configuration probe to READY, failures to FAILED with a code.

from svgdiag import *


def build():
    names = ["CFG-PROBE (retries)", "CFG writes: BAR0, BAR1, Command", "ROOT: memory window",
             "CAP poll (non-zero)", "MMIO writes: AQA, ASQ, ACQ, CC", "CSTS.RDY poll",
             "3 admin commands, CQE poll", "ENGINE: program, enable", "READY: init_done"]
    fails = {0: "1", 3: "2", 5: "3", 6: "4, 5"}
    step, h, x, w = 46, 32, 60, 300
    d = Diagram(CANVAS_WIDTH, 20 + step*(len(names) - 1) + h + 15)
    boxes = []
    for i, n in enumerate(names):
        fill = COLOR_ACCENT if i == len(names) - 1 else COLOR_FILL
        boxes.append(d.add(Box(x, 15 + i*step, w, h, label=n, fill=fill, size=14)))
    for a, b in zip(boxes, boxes[1:]):
        d.add(Arrow((a, "b"), (b, "t"), kind="data"))
    trunk = 520
    rows  = [boxes[i] for i in fails]
    for i, b in zip(fails, rows):
        d.add(Bus([(x + w, b.cy), (trunk, b.cy)], dash=DASH_FEEDBACK))
        d.add(Text(x + w + 50, b.cy - 10, "code " + fails[i], size=12, color="#444444"))
    d.add(Bus([(trunk, rows[0].cy), (trunk, rows[-1].cy)], dash=DASH_FEEDBACK))
    fail = d.add(Box(580, rows[2].cy - 24, 150, 48, label="FAILED:\ninit_error"))
    d.add(Arrow((trunk, fail.cy), (fail, "l"), kind="feedback"))
    return d
