# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause
#
# End-to-end simulation of the hardware benchmark path:
#
#   LiteNVMeRequestGen --source--> engine.sink
#        ^                                |
#        |  sink <--source-- engine   LiteNVMeIOEngineAXI (engine + dword->AXI bridge)
#                                         |  .axi
#                                         v
#                                    AXIArbiter --> LiteNVMeHostMemAXIRAM (real backend)
#                                         ^
#                                         |  .axi
#                                    model SSD (reads SQEs, posts CQEs)
#
# This is exactly what runs on hardware behind --with-io-engine, minus the real SSD/PCIe:
# the generator drives the engine at full rate (no CPU in the loop), the engine builds
# SQEs into the real backend, a model SSD round-trips them into CQEs, and the generator
# counts completions/errors and measures cycles. Verifies the generator drives `count`
# commands, all complete with no error, and the cycle counter is sane.

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.soc.interconnect import axi

from litenvme.request_gen import LiteNVMeRequestGen
from litenvme.io_engine import (
    LiteNVMeIOEngineAXI, LiteNVMeMemPortToAXI, LiteNVMeMemPort,
    NVME_OP_READ, NVME_OP_WRITE, SQE_DWORDS, CQE_DWORDS,
)
from litenvme.hostmem import LiteNVMeHostMemAXIRAM


class _DUT(Module):
    def __init__(self, data_width, size, qsize, qd):
        self.submodules.gen = gen = LiteNVMeRequestGen(with_csr=False)
        self.submodules.eng = eng = LiteNVMeIOEngineAXI(
            qid=1, qsize=qsize, qd=qd, data_width=data_width, with_csr=False)
        # Generator <-> engine streams.
        self.comb += [
            gen.source.connect(eng.sink),
            eng.source.connect(gen.sink),
        ]
        self.submodules.backend = backend = LiteNVMeHostMemAXIRAM(size=size, data_width=data_width)
        # Model SSD reaches host memory through the proven dword->AXI bridge.
        self.ssd_mem = LiteNVMeMemPort(adr_width=32)
        self.submodules.ssd_bridge = ssd_bridge = LiteNVMeMemPortToAXI(
            self.ssd_mem, data_width=data_width, address_width=32)
        self.submodules.arb = axi.AXIArbiter([eng.axi, ssd_bridge.axi], backend.axi)


class TestRequestGen(unittest.TestCase):
    # run_simulation generators can't return values, so the driver records into a dict.
    def _run_and_check(self, **kw):
        result = {}
        data_width = kw.get("data_width", 128)
        qsize = kw.get("qsize", 8); qd = kw.get("qd", 4)
        count = kw.get("count", 20); op = kw.get("op", "read")
        size      = 0x8000
        sq_base, cq_base = 0x4000, 0x3000
        sq_db_adr, cq_db_adr = 0xe000_1008, 0xe000_100c
        buf_base, buf_strd = 0x5000, 0x100
        do_write = (op == "write")

        dut = _DUT(data_width=data_width, size=size, qsize=qsize, qd=qd)
        eng, gen, smem = dut.eng, dut.gen, dut.ssd_mem
        sq_doorbells = []

        @passive
        def mmio_model():
            yield eng.mmio_done.eq(0)
            while True:
                if (yield eng.mmio_start):
                    if (yield eng.mmio_adr) == sq_db_adr:
                        sq_doorbells.append((yield eng.mmio_wdata))
                    yield eng.mmio_done.eq(1); yield
                    yield eng.mmio_done.eq(0); yield
                else:
                    yield eng.mmio_done.eq(0); yield

        def smem_read(a):
            yield smem.adr.eq(a); yield smem.we.eq(0); yield smem.stb.eq(1); yield
            c = 0
            while (yield smem.ack) == 0:
                yield; c += 1
                if c > 4000: raise RuntimeError("ssd read timeout")
            v = (yield smem.dat_r); yield smem.stb.eq(0); yield
            return v

        def smem_write(a, v):
            yield smem.adr.eq(a); yield smem.dat_w.eq(v & 0xffffffff)
            yield smem.we.eq(1); yield smem.stb.eq(1); yield
            c = 0
            while (yield smem.ack) == 0:
                yield; c += 1
                if c > 4000: raise RuntimeError("ssd write timeout")
            yield smem.stb.eq(0); yield smem.we.eq(0); yield

        @passive
        def ssd_model():
            produced = 0
            yield smem.stb.eq(0)
            while True:
                if produced < len(sq_doorbells):
                    k = produced; slot = k % qsize
                    dw0 = yield from smem_read(sq_base // 4 + slot*SQE_DWORDS)
                    cid = (dw0 >> 16) & 0xffff
                    phase = 1 ^ ((k // qsize) & 1); sqhd = (k + 1) % qsize
                    cqe_dw = cq_base // 4 + slot*CQE_DWORDS
                    cqe = [0, 0, sqhd & 0xffff, (cid & 0xffff) | ((phase & 1) << 16)]
                    for d in range(CQE_DWORDS):
                        yield from smem_write(cqe_dw + d, cqe[d])
                    produced += 1
                else:
                    yield

        def drv():
            yield eng.sq_base.eq(sq_base); yield eng.cq_base.eq(cq_base)
            yield eng.sq_db_adr.eq(sq_db_adr); yield eng.cq_db_adr.eq(cq_db_adr)
            yield eng.enable.eq(1)
            yield gen.op.eq(NVME_OP_WRITE if do_write else NVME_OP_READ)
            yield gen.nsid.eq(1); yield gen.base_lba.eq(1000); yield gen.nlb.eq(1)
            yield gen.lba_step.eq(1); yield gen.count.eq(count)
            yield gen.buf_base.eq(buf_base); yield gen.buf_stride.eq(buf_strd)
            yield gen.qmod.eq(qsize)
            yield
            yield gen.start.eq(1); yield
            yield gen.start.eq(0); yield
            c = 0
            while (yield gen.done) == 0:
                yield; c += 1
                if c > 200000: raise RuntimeError("gen never finished")
            result["completed"] = (yield gen.completed)
            result["errors"]    = (yield gen.errors)
            result["cycles"]    = (yield gen.cycles)
            result["doorbells"] = len(sq_doorbells)

        run_simulation(dut, [drv(), mmio_model(), ssd_model()])

        if result.get("completed") != count:
            raise AssertionError(f"completed {result.get('completed')} != count {count}")
        if result.get("errors") != 0:
            raise AssertionError(f"errors = {result.get('errors')} (expected 0)")
        if result.get("doorbells") != count:
            raise AssertionError(f"SQ doorbells {result.get('doorbells')} != count {count}")
        if not (0 < result["cycles"] < 200000):
            raise AssertionError(f"cycles out of range: {result['cycles']}")
        return result

    # Public tests -----------------------------------------------------------------------

    def test_read_qd4_128b(self):
        self._run_and_check(data_width=128, qsize=8, qd=4, count=20, op="read")

    def test_write_qd4_128b(self):
        self._run_and_check(data_width=128, qsize=8, qd=4, count=20, op="write")

    def test_read_qd7_64b(self):
        self._run_and_check(data_width=64, qsize=8, qd=7, count=30, op="read")


if __name__ == "__main__":
    unittest.main()
