# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Simulation of the block streamer over the real I/O path:
#
#   LiteNVMeBlockStreamer  --source--> engine.sink
#        |  .dma.axi (staging)         engine.source --> streamer.sink
#        v                                   ^
#   AXIArbiter --> LiteNVMeHostMemAXIRAM     |  LiteNVMeIOEngineAXI (+ doorbell model)
#        ^                                   |
#        |  .axi (model SSD reads SQEs / posts CQEs)
#
# Write test: stream a known pattern into wr_sink, run the streamer, and check the staging
# region of the backend holds it (+ a write SQE was submitted and a completion consumed).
# Read test: preload staging, run the streamer, and check rd_source emits exactly the bytes
# with `last` on the final beat. Both use the same model SSD/doorbell as the engine tests.

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.soc.interconnect import axi

from litenvme.io_engine import (
    LiteNVMeIOEngineAXI, LiteNVMeMemPortToAXI, LiteNVMeMemPort,
    SQE_DWORDS, CQE_DWORDS,
)
from litenvme.hostmem import LiteNVMeHostMemAXIRAM
from litenvme.block   import LiteNVMeBlockStreamer

# Layout (flat backend, zero-based: engine/streamer/ssd all use window-relative addresses).
DATA_WIDTH   = 256
SIZE         = 0x10000
SQ_BASE      = 0x4000
CQ_BASE      = 0x3000
STAGING_BASE = 0x8000
SQ_DB_ADR    = 0xe000_1008
CQ_DB_ADR    = 0xe000_100c
QSIZE        = 8
QD           = 4


class _DUT(Module):
    def __init__(self, data_width=DATA_WIDTH, size=SIZE, qsize=QSIZE, qd=QD):
        self.submodules.eng = eng = LiteNVMeIOEngineAXI(
            qid=1, qsize=qsize, qd=qd, data_width=data_width, with_csr=False, hostmem_base=0)
        self.submodules.streamer = streamer = LiteNVMeBlockStreamer(
            data_width=data_width, hostmem_base=0, staging_base=STAGING_BASE,
            staging_size=0x4000, with_csr=False)
        self.submodules.backend = backend = LiteNVMeHostMemAXIRAM(size=size, data_width=data_width)
        # Model SSD reaches host memory through the proven dword->AXI bridge.
        self.ssd_mem = LiteNVMeMemPort(adr_width=32)
        self.submodules.ssd_bridge = ssd_bridge = LiteNVMeMemPortToAXI(
            self.ssd_mem, data_width=data_width, address_width=32)
        self.submodules.arb = axi.AXIArbiter(
            [eng.axi, streamer.dma.axi, ssd_bridge.axi], backend.axi)
        # Front-end: the streamer drives the engine.
        self.comb += [
            streamer.source.connect(eng.sink),
            eng.source.connect(streamer.sink),
        ]


class TestBlockStreamer(unittest.TestCase):
    def _beat_val(self, i):
        # 256-bit beat with every dword lane == (i + 1).
        dw = (i + 1) & 0xffffffff
        return sum(dw << (32 * l) for l in range(DATA_WIDTH // 32))

    def _models(self, dut, status=0x0000):
        """Return (mmio_model, ssd_model) passives for a single command, posting `status`."""
        eng = dut.eng
        smem = dut.ssd_mem
        sq_doorbells = []

        @passive
        def mmio_model():
            yield eng.mmio_done.eq(0)
            while True:
                if (yield eng.mmio_start):
                    if (yield eng.mmio_adr) == SQ_DB_ADR:
                        sq_doorbells.append((yield eng.mmio_wdata))
                    yield eng.mmio_done.eq(1)
                    yield
                    yield eng.mmio_done.eq(0)
                    yield
                else:
                    yield eng.mmio_done.eq(0)
                    yield

        def smem_read(dw_adr):
            yield smem.adr.eq(dw_adr); yield smem.we.eq(0); yield smem.stb.eq(1); yield
            cyc = 0
            while (yield smem.ack) == 0:
                yield; cyc += 1
                if cyc > 2000: raise RuntimeError("ssd read timeout")
            val = (yield smem.dat_r); yield smem.stb.eq(0); yield
            return val

        def smem_write(dw_adr, value):
            yield smem.adr.eq(dw_adr); yield smem.dat_w.eq(value & 0xffffffff)
            yield smem.we.eq(1); yield smem.stb.eq(1); yield
            cyc = 0
            while (yield smem.ack) == 0:
                yield; cyc += 1
                if cyc > 2000: raise RuntimeError("ssd write timeout")
            yield smem.stb.eq(0); yield smem.we.eq(0); yield

        @passive
        def ssd_model():
            produced = 0
            yield smem.stb.eq(0)
            while True:
                tail = sq_doorbells[-1] if sq_doorbells else 0
                if produced < 1 and tail != 0:
                    dw0 = yield from smem_read(SQ_BASE // 4 + 0 * SQE_DWORDS + 0)
                    cid = (dw0 >> 16) & 0xffff
                    cqe = [0, 0, (0 + 1) & 0xffff, (cid & 0xffff) | (1 << 16) | (status << 17)]
                    for d in range(CQE_DWORDS):
                        yield from smem_write(CQ_BASE // 4 + 0 * CQE_DWORDS + d, cqe[d])
                    produced += 1
                else:
                    yield

        return mmio_model, ssd_model, sq_doorbells, smem_read, smem_write

    def _config_engine(self, eng):
        yield eng.sq_base.eq(SQ_BASE)
        yield eng.cq_base.eq(CQ_BASE)
        yield eng.sq_db_adr.eq(SQ_DB_ADR)
        yield eng.cq_db_adr.eq(CQ_DB_ADR)
        yield eng.enable.eq(1)
        yield

    def _start_and_wait(self, streamer, write, sector, count, nsid=1, timeout=200000):
        yield streamer.write.eq(write)
        yield streamer.sector.eq(sector)
        yield streamer.count.eq(count)
        yield streamer.nsid.eq(nsid)
        yield streamer.start.eq(1); yield
        yield streamer.start.eq(0)
        seen_busy = False
        for _ in range(timeout):
            if (yield streamer.busy):
                seen_busy = True
            if seen_busy and (yield streamer.done):
                return
            yield
        raise RuntimeError("streamer never completed")

    # Write: stream a pattern in, check it lands in staging. ------------------------------
    def test_write(self):
        dut = _DUT()
        eng, streamer = dut.eng, dut.streamer
        count   = 2  # sectors (1024 B).
        nbeats  = count * 512 // (DATA_WIDTH // 8)
        pattern = [self._beat_val(i) for i in range(nbeats)]
        mmio_model, ssd_model, sq_doorbells, smem_read, _w = self._models(dut)

        def wr_feeder():
            for i in range(nbeats):
                yield streamer.wr_sink.data.eq(pattern[i])
                yield streamer.wr_sink.valid.eq(1)
                yield
                cyc = 0
                while (yield streamer.wr_sink.ready) == 0:
                    yield; cyc += 1
                    if cyc > 50000: raise RuntimeError("wr_sink never accepted")
            yield streamer.wr_sink.valid.eq(0)

        result = {}
        def main():
            yield from self._config_engine(eng)
            yield from self._start_and_wait(streamer, write=1, sector=1000, count=count)
            result["error"] = (yield streamer.error)
            # Read the staging region back (dword lanes) via the ssd bridge.
            beats_dw = DATA_WIDTH // 32
            ok = True
            for i in range(nbeats):
                for l in range(beats_dw):
                    v = yield from smem_read((STAGING_BASE // 4) + i * beats_dw + l)
                    if v != ((i + 1) & 0xffffffff):
                        ok = False
            result["staging_ok"] = ok

        run_simulation(dut, [main(), wr_feeder(), mmio_model(), ssd_model()])
        self.assertEqual(result["error"], 0)
        self.assertTrue(result["staging_ok"], "staging contents != streamed pattern")
        self.assertTrue(sq_doorbells and sq_doorbells[-1] == 1, "no write SQE submitted")

    # Read: preload staging, check rd_source emits it with `last`. ------------------------
    def test_read(self):
        dut = _DUT()
        eng, streamer = dut.eng, dut.streamer
        count   = 2
        nbeats  = count * 512 // (DATA_WIDTH // 8)
        mmio_model, ssd_model, sq_doorbells, _r, smem_write = self._models(dut)
        captured = []

        def rd_sink():
            yield streamer.rd_source.ready.eq(1)
            last_seen = [False]
            while not last_seen[0]:
                if (yield streamer.rd_source.valid) and (yield streamer.rd_source.ready):
                    captured.append((yield streamer.rd_source.data))
                    if (yield streamer.rd_source.last):
                        last_seen[0] = True
                yield

        def main():
            yield from self._config_engine(eng)
            # Preload staging with the pattern (dword lanes).
            beats_dw = DATA_WIDTH // 32
            for i in range(nbeats):
                for l in range(beats_dw):
                    yield from smem_write((STAGING_BASE // 4) + i * beats_dw + l, (i + 1) & 0xffffffff)
            yield from self._start_and_wait(streamer, write=0, sector=2000, count=count)

        run_simulation(dut, [main(), rd_sink(), mmio_model(), ssd_model()])
        expected = [self._beat_val(i) for i in range(nbeats)]
        self.assertEqual(captured, expected, "rd_source data != preloaded staging")

    # Error status sticks and the transfer still completes. ------------------------------
    def test_error_status(self):
        dut = _DUT()
        eng, streamer = dut.eng, dut.streamer
        count  = 1
        nbeats = count * 512 // (DATA_WIDTH // 8)
        # Non-zero SC (bit field): status with SC != 0 (offset 1 in the 16-bit status word).
        mmio_model, ssd_model, sq_doorbells, *_ = self._models(dut, status=0x06)
        result = {}

        def wr_feeder():
            for i in range(nbeats):
                yield streamer.wr_sink.data.eq(self._beat_val(i))
                yield streamer.wr_sink.valid.eq(1)
                yield
                while (yield streamer.wr_sink.ready) == 0:
                    yield
            yield streamer.wr_sink.valid.eq(0)

        def main():
            yield from self._config_engine(eng)
            yield from self._start_and_wait(streamer, write=1, sector=10, count=count)
            result["error"] = (yield streamer.error)

        run_simulation(dut, [main(), wr_feeder(), mmio_model(), ssd_model()])
        self.assertEqual(result["error"], 1, "error status not latched")

    # done is registered: reads 0 within the run (request_gen idiom). --------------------
    def test_done_race(self):
        dut = _DUT()
        eng, streamer = dut.eng, dut.streamer
        mmio_model, ssd_model, *_ = self._models(dut)
        result = {}

        def wr_feeder():
            for i in range(2):
                yield streamer.wr_sink.data.eq(self._beat_val(i))
                yield streamer.wr_sink.valid.eq(1)
                yield
                while (yield streamer.wr_sink.ready) == 0:
                    yield
            yield streamer.wr_sink.valid.eq(0)

        def main():
            yield from self._config_engine(eng)
            yield streamer.write.eq(1); yield streamer.sector.eq(0); yield streamer.count.eq(1)
            yield streamer.start.eq(1); yield
            yield streamer.start.eq(0)
            # A few cycles in, done must be 0 (busy).
            for _ in range(8):
                yield
            result["mid_done"] = (yield streamer.done)
            for _ in range(200000):
                if (yield streamer.done):
                    break
                yield

        run_simulation(dut, [main(), wr_feeder(), mmio_model(), ssd_model()])
        self.assertEqual(result["mid_done"], 0, "done not cleared during the run")


if __name__ == "__main__":
    unittest.main()
