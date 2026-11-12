#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A queue is a ring of entries in host memory: the producer advances the tail, the consumer the
# head; the phase bit tells a new completion from an old one.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    A, L, S, F = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE, COLOR_FILL
    x0, w, h = 110, 68, 36
    d.add(Text(x0 - 12, 70 + h/2, "SQ", size=15, anchor="end"))
    cells = [(w, f"{i}", A if 2 <= i <= 4 else F) for i in range(8)]
    bitfield(d, x0, 70, cells, h=h, size=13)
    d.add(Arrow((x0 + 2*w + w/2, 40), (x0 + 2*w + w/2, 70), kind="control", label="head", label_pos=(x0 + 2*w + w/2, 30)))
    d.add(Arrow((x0 + 5*w + w/2, 40), (x0 + 5*w + w/2, 70), kind="control", label="tail", label_pos=(x0 + 5*w + w/2, 30)))
    d.add(Text(x0 + 8*w + 10, 70 + h/2, "commands", size=13, anchor="start", color="#444444"))
    d.add(Text(x0 - 12, 160 + h/2, "CQ", size=15, anchor="end"))
    cells = [(w, f"P={1 if i < 3 else 0}", L if i == 0 else A if i < 3 else S) for i in range(8)]
    bitfield(d, x0, 160, cells, h=h, size=13)
    d.add(Arrow((x0 + 1*w + w/2, 230), (x0 + 1*w + w/2, 196), kind="control", label="head", label_pos=(x0 + 1*w + w/2, 240)))
    d.add(Text(x0 + 8*w + 10, 160 + h/2, "completions", size=13, anchor="start", color="#444444"))
    d.add(Arrow((x0 + 8*w - 4, 106), (x0 + 4, 106), via=[(x0 + 8*w - 4, 128), (x0 + 4, 128)], kind="feedback", label="wraps", label_pos=(x0 + 4*w, 140)))
    return d
