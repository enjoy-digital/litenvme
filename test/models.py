#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Shared simulation models for the LiteNVMe engine tests.

These passive Migen processes are reused across test/test_io_engine*.py and
test/test_request_gen.py. Only the models with identical cycle behavior live here; the SSD
models stay per-test because they differ by host-memory access mechanism (the dword `mem`
port vs the AXI `ssd_mem` bridge) and by what they capture (e.g. the PRP-list snapshot).
"""

from migen import *
from migen.sim import passive

# Host-memory dword-port model ---------------------------------------------------------------------

def make_mem_model(dut, mem):
    """Model the engine's dword `mem` port backed by a Python dict.

    Contract: when stb is high, perform the access for the address currently on `adr`; assert
    ack for one cycle with dat_r valid that same cycle, then drop ack for one bubble cycle so
    each transfer is unambiguous.
    """
    @passive
    def mem_model():
        yield dut.mem.ack.eq(0)
        while True:
            # Wait for a strobe.
            if (yield dut.mem.stb) == 0:
                yield dut.mem.ack.eq(0)
                yield
                continue
            # Latch the access for the presented address.
            we  = (yield dut.mem.we)
            adr = (yield dut.mem.adr)
            if we:
                mem[adr] = (yield dut.mem.dat_w)
            else:
                yield dut.mem.dat_r.eq(mem.get(adr, 0))
            # Present ack (and, for reads, dat_r) on the next cycle.
            yield dut.mem.ack.eq(1)
            yield
            yield dut.mem.ack.eq(0)
            yield  # Bubble cycle.
    return mem_model

# MMIO doorbell model ------------------------------------------------------------------------------

def make_mmio_model(eng, sq_db_adr, sq_doorbells, cq_db_adr=None, cq_doorbells=None):
    """Model the doorbell MMIO accessor: record rung doorbell values, pulse done after start.

    Records every SQ doorbell value into `sq_doorbells`; when `cq_db_adr`/`cq_doorbells` are
    given, CQ doorbells are recorded too. `eng` is the object exposing mmio_start/_adr/_wdata/
    _done (the engine, or a DUT that forwards them).
    """
    @passive
    def mmio_model():
        yield eng.mmio_done.eq(0)
        while True:
            if (yield eng.mmio_start) == 0:
                yield eng.mmio_done.eq(0)
                yield
                continue
            adr = (yield eng.mmio_adr)
            if adr == sq_db_adr:
                sq_doorbells.append((yield eng.mmio_wdata))
            elif (cq_db_adr is not None) and (adr == cq_db_adr):
                cq_doorbells.append((yield eng.mmio_wdata))
            yield eng.mmio_done.eq(1)
            yield
            yield eng.mmio_done.eq(0)
            yield  # Bubble.
    return mmio_model
