# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Simulation of the RTL init sequencer (litenvme/init.py): drive its abstract cfg / mmio / root /
# host-memory interfaces with simple models + a model "SSD" that posts admin CQEs, and check the
# sequencer walks the whole NVMe bring-up and raises init_done (no init_error).

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litenvme.init import LiteNVMeInitSequencer

BAR0      = 0xe000_0000
HOSTMEM   = 0x1000_0000
ASQ       = HOSTMEM + 0x0000
ACQ       = HOSTMEM + 0x1000
ADM_SQ_DB = BAR0 + 0x1000
ADM_CQ_DB = BAR0 + 0x1004


class _DUT(Module):
    def __init__(self):
        self.submodules.seq = LiteNVMeInitSequencer(
            bar0_base=BAR0, hostmem_base=HOSTMEM, admin_q_entries=2, io_q_entries=64,
            poll_timeout=4096)


class TestInitSequencer(unittest.TestCase):
    def test_bringup(self):
        dut = _DUT()
        s = dut.seq
        mem = {}                 # host memory (dword-addressed).
        sq_doorbells = []        # admin SQ doorbell values rung.
        cq_doorbells = []        # admin CQ doorbell values rung.
        served_ops   = []        # opcode of each admin SQE the model SSD served.

        # --- Generic single-DWORD accessor model (mimics _LiteNVMePCIeAccessor done semantics). ---
        def make_acc(start, we, done, rdata=None, read_fn=None, on_write=None, latency=2):
            @passive
            def model():
                yield done.eq(0)
                while True:
                    if (yield start):
                        is_w = (yield we)
                        yield done.eq(0)                 # clear on start (like start_pulse).
                        if (not is_w) and read_fn is not None:
                            yield rdata.eq(read_fn())
                        if is_w and on_write is not None:
                            on_write((yield we))
                        for _ in range(latency):
                            yield
                        yield done.eq(1)                 # sticky until next start.
                        yield
                    else:
                        yield
            return model

        # CFG: probe read returns a valid VID/DID; writes just ack.
        cfg_model = make_acc(s.cfg_start, s.cfg_we, s.cfg_done,
                             rdata=s.cfg_rdata, read_fn=lambda: 0x5427c0a9)

        # MMIO: reads return CAP (non-zero) / CSTS (RDY=1); record admin doorbell writes.
        @passive
        def mmio_model():
            yield s.mmio_done.eq(0)
            while True:
                if (yield s.mmio_start):
                    we  = (yield s.mmio_we)
                    adr = (yield s.mmio_adr)
                    yield s.mmio_done.eq(0)
                    if not we:
                        if adr == BAR0 + 0x00:   rd = 0x0020_0000_0000_3fff & 0xffffffff  # CAP lo (non-zero).
                        elif adr == BAR0 + 0x1c: rd = 0x1                                  # CSTS.RDY=1.
                        else:                    rd = 0
                        yield s.mmio_rdata.eq(rd)
                    else:
                        wd = (yield s.mmio_wdata)
                        if adr == ADM_SQ_DB: sq_doorbells.append(wd)
                        if adr == ADM_CQ_DB: cq_doorbells.append(wd)
                    yield; yield
                    yield s.mmio_done.eq(1)
                    yield
                else:
                    yield

        # ROOT cfg-mgmt: ack the window write.
        @passive
        def root_model():
            yield s.root_done.eq(0)
            while True:
                if (yield s.root_start):
                    yield s.root_done.eq(0)
                    yield; yield
                    yield s.root_done.eq(1)
                    yield
                else:
                    yield

        # Host-memory dword port (dict-backed; ack one cycle after stb, dat_r valid that cycle).
        @passive
        def mem_model():
            yield s.mem.ack.eq(0)
            while True:
                if (yield s.mem.stb) == 0:
                    yield s.mem.ack.eq(0); yield; continue
                adr = (yield s.mem.adr)
                if (yield s.mem.we):
                    mem[adr] = (yield s.mem.dat_w)
                else:
                    yield s.mem.dat_r.eq(mem.get(adr, 0))
                yield s.mem.ack.eq(1); yield
                yield s.mem.ack.eq(0); yield

        # Model SSD: on each admin SQ doorbell, read the SQE cid and post a success CQE (phase
        # tracking mirrors the sequencer: ADMIN_Q_ENTRIES=2).
        @passive
        def ssd_model():
            served = 0
            dev_head, dev_phase = 0, 1
            while True:
                if len(sq_doorbells) > served:
                    served += 1
                    slot = (served - 1) % 2  # the SQ slot just submitted (sq_tail advanced to served%2).
                    dw0  = mem.get((ASQ + slot * 64) >> 2, 0)
                    cid  = (dw0 >> 16) & 0xffff
                    served_ops.append(dw0 & 0xff)
                    cqe_dw = (ACQ + dev_head * 16) >> 2
                    mem[cqe_dw + 0] = 0
                    mem[cqe_dw + 1] = 0
                    mem[cqe_dw + 2] = (slot + 1) & 0xffff          # sqhd.
                    mem[cqe_dw + 3] = (cid & 0xffff) | (dev_phase << 16)  # success (sc=sct=0).
                    dev_head = (dev_head + 1) % 2
                    if dev_head == 0:
                        dev_phase ^= 1
                yield

        result = {}
        def main():
            yield s.start.eq(1); yield
            yield s.start.eq(0)
            for _ in range(200000):
                if (yield s.init_done) or (yield s.init_error):
                    break
                yield
            result["done"]  = (yield s.init_done)
            result["error"] = (yield s.init_error)
            result["eng_enable"]  = (yield s.eng_enable)
            result["eng_sq_base"] = (yield s.eng_sq_base)
            result["eng_cq_base"] = (yield s.eng_cq_base)

        run_simulation(dut, [main(), cfg_model(), mmio_model(), root_model(), mem_model(), ssd_model()])

        self.assertEqual(result["error"], 0, "init_error asserted")
        self.assertEqual(result["done"], 1, "init_done not reached")
        self.assertEqual(result["eng_enable"], 1, "engine not enabled")
        self.assertEqual(result["eng_sq_base"], HOSTMEM + 0x4000, "wrong IO SQ base")
        self.assertEqual(result["eng_cq_base"], HOSTMEM + 0x3000, "wrong IO CQ base")
        # 3 admin commands submitted + reaped.
        self.assertEqual(len(sq_doorbells), 3, f"expected 3 admin SQ doorbells, got {sq_doorbells}")
        self.assertEqual(len(cq_doorbells), 3, f"expected 3 admin CQ doorbells, got {cq_doorbells}")
        # The 3 admin SQEs, in order, are Set Features (0x09), Create IO CQ (0x05), Create IO SQ
        # (0x01). Captured at submission time (the 2-entry admin ring overwrites slot 0).
        self.assertEqual(served_ops, [0x09, 0x05, 0x01], f"wrong admin opcodes: {[hex(o) for o in served_ops]}")


if __name__ == "__main__":
    unittest.main()
