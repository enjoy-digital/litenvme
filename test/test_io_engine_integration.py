# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# End-to-end integration simulation of the hardware NVMe I/O path:
#
#   LiteNVMeIOEngineAXI (engine + dword->AXI bridge)
#        |  .axi (master)                         doorbell port (modeled)
#        v                                              |
#   AXIArbiter ---> LiteNVMeHostMemAXIRAM (real backend BRAM)
#        ^
#        |  .axi (master)
#   model SSD  (reads SQEs the engine wrote, writes CQEs the engine reaps)
#
# This exercises the actual RTL the SoC will use (engine, bridge, AXI arbiter, real
# beat-wide backend) and a model device that round-trips commands through host memory:
# the engine builds SQEs into the backend, the SSD reads them back via a second AXI
# master and posts CQEs, and the engine reaps them. Verifies completion CID order,
# success status, that in-flight never exceeds qd, and the SQ-doorbell sequence.

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.soc.interconnect import axi

from litenvme.io_engine import (
    LiteNVMeIOEngineAXI, LiteNVMeMemPortToAXI, LiteNVMeMemPort,
    NVME_OP_READ, NVME_OP_WRITE, SQE_DWORDS, CQE_DWORDS,
)
from litenvme.hostmem import LiteNVMeHostMemAXIRAM


class _DUT(Module):
    def __init__(self, data_width, size, qsize, qd):
        self.submodules.eng = eng = LiteNVMeIOEngineAXI(
            qid=1, qsize=qsize, qd=qd, data_width=data_width, with_csr=False)
        self.submodules.backend = backend = LiteNVMeHostMemAXIRAM(size=size, data_width=data_width)
        # Model SSD reaches host memory through the same (proven) dword->AXI bridge, at
        # dword granularity, so the test never hand-rolls raw AXI handshakes.
        self.ssd_mem = LiteNVMeMemPort(adr_width=32)
        self.submodules.ssd_bridge = ssd_bridge = LiteNVMeMemPortToAXI(
            self.ssd_mem, data_width=data_width, address_width=32)
        self.submodules.arb = axi.AXIArbiter([eng.axi, ssd_bridge.axi], backend.axi)


class TestIOEngineIntegration(unittest.TestCase):
    def _run_case(self, data_width=128, qsize=8, qd=4, n_cmds=16, op="read"):
        size      = 0x8000
        sq_base   = 0x4000
        cq_base   = 0x3000
        sq_db_adr = 0xe000_1008
        cq_db_adr = 0xe000_100c
        do_write    = (op == "write")

        dut = _DUT(data_width=data_width, size=size, qsize=qsize, qd=qd)
        eng = dut.eng
        smem = dut.ssd_mem

        sq_doorbells = []
        completions  = []
        inflight_hi  = [0]

        reqs = [dict(op=(NVME_OP_WRITE if do_write else NVME_OP_READ), nsid=1,
                     lba=1000+i, nlb=(i % 4)+1, buf=0x5000+i*0x100) for i in range(n_cmds)]

        # --- MMIO doorbell model: record SQ doorbells, pulse done. ----------------------
        @passive
        def mmio_model():
            yield eng.mmio_done.eq(0)
            while True:
                if (yield eng.mmio_start):
                    adr = (yield eng.mmio_adr)
                    if adr == sq_db_adr:
                        sq_doorbells.append((yield eng.mmio_wdata))
                    yield eng.mmio_done.eq(1)
                    yield
                    yield eng.mmio_done.eq(0)
                    yield
                else:
                    yield eng.mmio_done.eq(0)
                    yield

        # --- SSD dword memory access via the bridge (dword address granularity). --------
        def smem_read(dw_adr):
            yield smem.adr.eq(dw_adr)
            yield smem.we.eq(0)
            yield smem.stb.eq(1)
            yield
            cyc = 0
            while (yield smem.ack) == 0:
                yield
                cyc += 1
                if cyc > 2000: raise RuntimeError("ssd read timeout")
            val = (yield smem.dat_r)
            yield smem.stb.eq(0)
            yield
            return val

        def smem_write(dw_adr, value):
            yield smem.adr.eq(dw_adr)
            yield smem.dat_w.eq(value & 0xffffffff)
            yield smem.we.eq(1)
            yield smem.stb.eq(1)
            yield
            cyc = 0
            while (yield smem.ack) == 0:
                yield
                cyc += 1
                if cyc > 2000: raise RuntimeError("ssd write timeout")
            yield smem.stb.eq(0)
            yield smem.we.eq(0)
            yield

        # --- Model SSD: read each submitted SQE, post a matching CQE. --------------------
        @passive
        def ssd_model():
            produced = 0
            dev_head = 0
            yield smem.stb.eq(0)
            while True:
                tail   = sq_doorbells[-1] if sq_doorbells else 0
                navail = (tail - dev_head) % qsize
                if produced < n_cmds and navail > 0:
                    k    = produced
                    slot = dev_head
                    sqe_dw0 = sq_base // 4 + slot * SQE_DWORDS + 0
                    dw0     = yield from smem_read(sqe_dw0)
                    cid     = (dw0 >> 16) & 0xffff
                    phase = 1 ^ ((k // qsize) & 1)
                    sqhd  = (slot + 1) % qsize
                    cqe_dw = cq_base // 4 + (k % qsize) * CQE_DWORDS
                    cqe_dws = [0, 0, sqhd & 0xffff, (cid & 0xffff) | ((phase & 1) << 16)]
                    for d in range(CQE_DWORDS):
                        yield from smem_write(cqe_dw + d, cqe_dws[d])
                    dev_head = (dev_head + 1) % qsize
                    produced += 1
                else:
                    yield

        # --- Completion sink. -----------------------------------------------------------
        @passive
        def cpl_sink():
            yield eng.source.ready.eq(1)
            while True:
                if (yield eng.source.valid) and (yield eng.source.ready):
                    completions.append(((yield eng.source.cid), (yield eng.source.status)))
                inf = (yield eng.inflight)
                if inf > inflight_hi[0]:
                    inflight_hi[0] = inf
                yield

        # --- Request source. ------------------------------------------------------------
        def req_src():
            yield eng.sq_base.eq(sq_base)
            yield eng.cq_base.eq(cq_base)
            yield eng.sq_db_adr.eq(sq_db_adr)
            yield eng.cq_db_adr.eq(cq_db_adr)
            yield eng.enable.eq(1)
            yield
            for r in reqs:
                yield eng.sink.op.eq(r["op"])
                yield eng.sink.nsid.eq(r["nsid"])
                yield eng.sink.lba.eq(r["lba"])
                yield eng.sink.nlb.eq(r["nlb"])
                yield eng.sink.buf.eq(r["buf"])
                yield eng.sink.valid.eq(1)
                yield
                cyc = 0
                while (yield eng.sink.ready) == 0:
                    yield
                    cyc += 1
                    if cyc > 20000: raise RuntimeError("engine never accepted request")
                yield eng.sink.valid.eq(0)
                yield
            yield eng.sink.valid.eq(0)
            for _ in range(200000):
                if len(completions) >= n_cmds:
                    break
                yield

        run_simulation(dut, [req_src(), mmio_model(), ssd_model(), cpl_sink()])

        # --- Checks. --------------------------------------------------------------------
        if len(completions) != n_cmds:
            raise AssertionError(f"expected {n_cmds} completions, got {len(completions)}")
        exp_cids = [(k % qsize) for k in range(n_cmds)]
        got_cids = [c for (c, _s) in completions]
        if got_cids != exp_cids:
            raise AssertionError(f"completion CID order mismatch: {got_cids} != {exp_cids}")
        for (c, s) in completions:
            sc, sct = (s >> 1) & 0xff, (s >> 9) & 0x7
            if sc or sct:
                raise AssertionError(f"cid {c}: non-success status 0x{s:04x}")
        if inflight_hi[0] > qd:
            raise AssertionError(f"inflight high-water {inflight_hi[0]} > qd {qd}")
        # Coalesced doorbells: don't expect one per command; check final tail + bounds.
        if not sq_doorbells:
            raise AssertionError("no SQ doorbell rung")
        if sq_doorbells[-1] != n_cmds % qsize:
            raise AssertionError(f"final SQ tail {sq_doorbells[-1]} != {n_cmds % qsize}")
        if len(sq_doorbells) > n_cmds:
            raise AssertionError(f"coalescing added SQ doorbells: {len(sq_doorbells)} > {n_cmds}")

    # Public tests -----------------------------------------------------------------------

    def test_read_qd4_128b(self):
        self._run_case(data_width=128, qsize=8, qd=4, n_cmds=16, op="read")

    def test_write_qd2_128b(self):
        self._run_case(data_width=128, qsize=8, qd=2, n_cmds=10, op="write")

    def test_read_qd4_64b(self):
        self._run_case(data_width=64, qsize=8, qd=4, n_cmds=12, op="read")


if __name__ == "__main__":
    unittest.main()
