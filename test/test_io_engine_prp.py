# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause
#
# PRP2 / PRP-list construction test for LiteNVMeIOEngine.
#
# Uses a model dword memory (like test_io_engine.py) so the SQE and the PRP list page
# written by the engine can be inspected directly. A model SSD posts CQEs so commands
# complete. Checks, for transfers of increasing size (512 B LBAs, 4 KiB pages):
#   - npages == 1  -> PRP1 = buf, PRP2 = 0
#   - npages == 2  -> PRP1 = buf, PRP2 = buf + page
#   - npages >= 3  -> PRP1 = buf, PRP2 = list_slot, and the list page holds
#                     buf + page, buf + 2*page, ... (npages-1 entries, 64-bit each)

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litenvme.io_engine import LiteNVMeIOEngine, NVME_OP_READ, SQE_DWORDS, CQE_DWORDS

PAGE  = 4096
LBA   = 512
QSIZE = 8


class TestIOEnginePRP(unittest.TestCase):
    def _run(self, nlb_list):
        sq_base   = 0x1_0000
        cq_base   = 0x2_0000
        prp_base  = 0x4_0000           # one 4 KiB list page per slot.
        sq_db_adr = 0xe000_1008
        cq_db_adr = 0xe000_100c
        qd        = 4

        dut = LiteNVMeIOEngine(qid=1, qsize=QSIZE, qd=qd, with_csr=False,
                               lba_shift=9, page_shift=12)

        mem = {}
        sq_doorbells = []
        completions  = []

        @passive
        def mem_model():
            yield dut.mem.ack.eq(0)
            while True:
                if (yield dut.mem.stb) == 0:
                    yield dut.mem.ack.eq(0); yield; continue
                we = (yield dut.mem.we); adr = (yield dut.mem.adr)
                if we:
                    mem[adr] = (yield dut.mem.dat_w)
                else:
                    yield dut.mem.dat_r.eq(mem.get(adr, 0))
                yield dut.mem.ack.eq(1); yield
                yield dut.mem.ack.eq(0); yield

        @passive
        def mmio_model():
            yield dut.mmio_done.eq(0)
            while True:
                if (yield dut.mmio_start) == 0:
                    yield dut.mmio_done.eq(0); yield; continue
                if (yield dut.mmio_adr) == sq_db_adr:
                    sq_doorbells.append((yield dut.mmio_wdata))
                yield dut.mmio_done.eq(1); yield
                yield dut.mmio_done.eq(0); yield

        sq_dw = sq_base >> 2
        cq_dw = cq_base >> 2

        @passive
        def ssd_model():
            produced = 0
            dev_head = 0
            while True:
                tail   = sq_doorbells[-1] if sq_doorbells else 0
                navail = (tail - dev_head) % QSIZE
                if produced < len(nlb_list) and navail > 0:
                    k = produced; slot = dev_head
                    # Capture the SQE + list page HERE, at device-consume time: it is present
                    # (doorbell rung past it) and provably not yet reused (the engine can only
                    # reuse this slot after we post its CQE below -> reap -> resubmit). Doing it
                    # in a separate process races slot reuse once doorbells are coalesced.
                    base = sq_dw + slot*SQE_DWORDS
                    sqe_snap[k] = [mem.get(base+i, 0) for i in range(SQE_DWORDS)]
                    lp = (prp_base + (slot << 12)) >> 2
                    sqe_snap[(k, "list")] = [mem.get(lp+i, 0) for i in range(1024)]
                    dw0 = mem.get(sq_dw + slot*SQE_DWORDS + 0, 0)
                    cid = (dw0 >> 16) & 0xffff
                    phase = 1 ^ ((k // QSIZE) & 1)
                    sqhd = (slot + 1) % QSIZE
                    b = cq_dw + (k % QSIZE)*CQE_DWORDS
                    mem[b+0] = 0; mem[b+1] = 0; mem[b+2] = sqhd & 0xffff
                    mem[b+3] = (cid & 0xffff) | ((phase & 1) << 16)
                    dev_head = (dev_head + 1) % QSIZE
                    produced += 1
                yield

        @passive
        def cpl_sink():
            yield dut.source.ready.eq(1)
            while True:
                if (yield dut.source.valid) and (yield dut.source.ready):
                    completions.append((yield dut.source.cid))
                yield

        # Capture each command's SQE (slots reused; snapshot at submission).
        sqe_snap = {}

        @passive
        def snapshot():
            seen = 0
            dev_head = 0
            while True:
                tail   = sq_doorbells[-1] if sq_doorbells else 0
                navail = (tail - dev_head) % QSIZE
                if seen < len(nlb_list) and navail > 0:
                    k = seen; slot = dev_head
                    base = sq_dw + slot*SQE_DWORDS
                    sqe_snap[k] = [mem.get(base+i, 0) for i in range(SQE_DWORDS)]
                    # Also snapshot the list page for this slot.
                    lp = (prp_base + (slot << 12)) >> 2
                    sqe_snap[(k, "list")] = [mem.get(lp+i, 0) for i in range(1024)]
                    dev_head = (dev_head + 1) % QSIZE
                    seen += 1
                yield

        bufs = [0x10_0000 + i*0x10000 for i in range(len(nlb_list))]  # page-aligned bufs.

        def drv():
            yield dut.sq_base.eq(sq_base)
            yield dut.cq_base.eq(cq_base)
            yield dut.sq_db_adr.eq(sq_db_adr)
            yield dut.cq_db_adr.eq(cq_db_adr)
            yield dut.prp_list_base.eq(prp_base)
            yield dut.enable.eq(1)
            yield
            for i, nlb in enumerate(nlb_list):
                yield dut.sink.op.eq(NVME_OP_READ)
                yield dut.sink.nsid.eq(1)
                yield dut.sink.lba.eq(1000 + i)
                yield dut.sink.nlb.eq(nlb)
                yield dut.sink.buf.eq(bufs[i])
                yield dut.sink.valid.eq(1)
                yield
                c = 0
                while (yield dut.sink.ready) == 0:
                    yield; c += 1
                    if c > 20000: raise RuntimeError("engine stuck")
                yield dut.sink.valid.eq(0)
                yield
            yield dut.sink.valid.eq(0)
            for _ in range(40000):
                if len(completions) >= len(nlb_list):
                    break
                yield

        # NOTE: snapshot() is intentionally NOT run -- the SQE/list capture now happens inside
        # ssd_model at consume time, which is race-free against slot reuse under coalescing.
        run_simulation(dut, [drv(), mem_model(), mmio_model(), ssd_model(), cpl_sink()])

        if len(completions) != len(nlb_list):
            raise AssertionError(f"expected {len(nlb_list)} completions, got {len(completions)}")

        # Verify PRP fields per command.
        for i, nlb in enumerate(nlb_list):
            sqe = sqe_snap[i]
            buf = bufs[i]
            slot = i % QSIZE
            prp1 = sqe[6] | (sqe[7] << 32)
            prp2 = sqe[8] | (sqe[9] << 32)
            nbytes = nlb * LBA
            npages = (nbytes + PAGE - 1) // PAGE

            if prp1 != buf:
                raise AssertionError(f"cmd {i}: PRP1 0x{prp1:x} != buf 0x{buf:x}")

            if npages == 1:
                if prp2 != 0:
                    raise AssertionError(f"cmd {i} (1 page): PRP2 0x{prp2:x} != 0")
            elif npages == 2:
                if prp2 != buf + PAGE:
                    raise AssertionError(f"cmd {i} (2 pages): PRP2 0x{prp2:x} != buf+page")
            else:
                list_slot = prp_base + (slot << 12)
                if prp2 != list_slot:
                    raise AssertionError(f"cmd {i} ({npages} pages): PRP2 0x{prp2:x} != list_slot 0x{list_slot:x}")
                lp = sqe_snap[(i, "list")]
                for e in range(npages - 1):
                    ent = lp[2*e] | (lp[2*e+1] << 32)
                    exp = buf + (e + 1) * PAGE
                    if ent != exp:
                        raise AssertionError(
                            f"cmd {i}: PRP list[{e}] = 0x{ent:x} != 0x{exp:x}")

    def test_one_page(self):
        # nlb=8 -> 4096 B = 1 page (PRP1 only).
        self._run([8])

    def test_two_pages(self):
        # nlb=16 -> 8192 B = 2 pages (PRP2 = buf+page).
        self._run([16])

    def test_three_pages(self):
        # nlb=24 -> 12288 B = 3 pages (list with 2 entries).
        self._run([24])

    def test_list_many(self):
        # nlb=64 -> 32768 B = 8 pages (list with 7 entries).
        self._run([64])

    def test_mixed(self):
        # A mix exercising all paths back to back (slot reuse).
        self._run([8, 16, 24, 64, 8, 40])


if __name__ == "__main__":
    unittest.main()
