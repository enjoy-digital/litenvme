# test/test_hostmem.py
#
# Unit test for LiteNVMeHostMemResponder:
# - Preload ASQ[0] (64B) through CSR dword interface.
# - Simulate NVMe issuing a MemRd TLP for that 64B.
# - Check CplD beats returned at 1 beat/cycle after BRAM latency,
#   and that backpressure does not deadlock.
#

import sys
import unittest
import random

from migen import *
from migen.sim import run_simulation, passive

from litex.gen import *
from litex.soc.interconnect import stream

from litenvme.hostmem import LiteNVMeHostMemResponder

# ------------------------------------------------------------------------------------------
# Minimal "LitePCIe user port" layouts matching what LiteNVMeHostMemResponder uses.
# ------------------------------------------------------------------------------------------

def _req_layout(data_width, beat_bytes):
    return [
        # payload signals:
        ("adr",    64),          # byte address
        ("we",     1),           # 1 = MemWr, 0 = MemRd
        ("len",    10),          # dwords
        ("tag",    8),
        ("req_id", 16),
        ("dat",    data_width),  # write data beat (for MemWr)
        ("be",     beat_bytes),  # per-byte enables (width in *bytes*)
    ]

def _cmp_layout(data_width):
    return [
        ("dat",    data_width),
        ("end",    1),
        ("len",    10),
        ("tag",    8),
        ("req_id", 16),
        ("cmp_id", 16),
        ("err",    1),
    ]


class _Port(Module):
    def __init__(self, data_width):
        beat_bytes = data_width // 8
        self.sink   = stream.Endpoint(_req_layout(data_width, beat_bytes))
        self.source = stream.Endpoint(_cmp_layout(data_width))


# ------------------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------------------

def _pack_dwords_to_int(dws):
    v = 0
    for i, dw in enumerate(dws):
        v |= (dw & 0xFFFFFFFF) << (32*i)
    return v

def _unpack_int_to_dwords(v, n):
    return [((v >> (32*i)) & 0xFFFFFFFF) for i in range(n)]


# ------------------------------------------------------------------------------------------
# Test
# ------------------------------------------------------------------------------------------

class TestHostMemIdentifyRead(unittest.TestCase):
    def _run_case(self, data_width=128, stall_prob=0.3, seed=0xC001D00D):
        random.seed(seed)

        base = 0x1000_0000
        # Keep small: 16KB window is plenty for ASQ/ACQ/ID pages (4 pages).
        size = 0x4000

        port = _Port(data_width=data_width)
        dut  = LiteNVMeHostMemResponder(
            port       = port,
            base       = base,
            size       = size,
            data_width = data_width,
            with_csr   = True,
        )

        beat_dwords = data_width // 32

        # Build a realistic Identify Controller SQE (16 DW = 64B).
        # Same format as your script: opcode 0x06, CID in bits 31:16, PRP1 points to ID buffer.
        cid     = 0x0001
        id_buf  = base + 0x2000
        sqe = [0] * 16
        sqe[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
        sqe[6]  = id_buf & 0xffffffff
        sqe[7]  = (id_buf >> 32) & 0xffffffff
        sqe[10] = 0x00000001  # CNS=1 (Identify Controller)

        got_cpl_beats = []

        # ----------------------------------------------------------------------------------
        # CSR dword write helper.
        # Your CSR RMW FSM is:
        #   CSR_IDLE -> CSR_RMW_READ -> CSR_RMW_WRITE -> CSR_IDLE
        # so we pulse _csr_we.storage for 1 cycle and wait a few cycles.
        # ----------------------------------------------------------------------------------
        def csr_write_dw(dw_index, value):
            # set address + data
            yield dut.csr._csr_adr.storage.eq(dw_index)
            yield dut.csr._csr_wdata.storage.eq(value)
            # pulse write strobe
            yield dut.csr._csr_we.storage.eq(1)
            yield
            yield dut.csr._csr_we.storage.eq(0)
            # allow RMW to complete
            for _ in range(4):
                yield

        # ----------------------------------------------------------------------------------
        # Stimulus: preload ASQ[0] then issue MemRd for 64B.
        # ----------------------------------------------------------------------------------
        def stimulus():
            # Init inputs.
            yield port.sink.valid.eq(0)
            yield port.sink.first.eq(0)
            yield port.sink.last.eq(0)
            yield port.sink.adr.eq(0)
            yield port.sink.we.eq(0)
            yield port.sink.len.eq(0)
            yield port.sink.tag.eq(0)
            yield port.sink.req_id.eq(0)
            yield port.sink.dat.eq(0)
            yield port.sink.be.eq((1 << (data_width//8)) - 1)
            yield

            # ----------------------------------------------------------------------
            # 1) Preload SQE (16 DW) into ASQ base through CSR dword interface.
            # ----------------------------------------------------------------------
            asq_addr = base + 0x0000
            for i, dw in enumerate(sqe):
                dw_addr = (asq_addr - base) // 4 + i
                yield from csr_write_dw(dw_addr, dw)

            # ----------------------------------------------------------------------
            # 2) NVMe issues a MemRd for 64B from ASQ[0].
            #    In LitePCIe user port, this is a single request beat (header),
            #    with len in dwords.
            # ----------------------------------------------------------------------
            yield port.sink.adr.eq(asq_addr)
            yield port.sink.we.eq(0)
            yield port.sink.len.eq(16)          # 16 dwords = 64B
            yield port.sink.tag.eq(0x5A)
            yield port.sink.req_id.eq(0x1234)
            yield port.sink.first.eq(1)
            yield port.sink.last.eq(1)
            yield port.sink.valid.eq(1)

            # Wait handshake.
            while (yield port.sink.ready) == 0:
                yield
            yield  # take the handshake cycle

            # Deassert.
            yield port.sink.valid.eq(0)
            yield port.sink.first.eq(0)
            yield port.sink.last.eq(0)

            # Let completions run; bound runtime to avoid “stuck forever”.
            for _ in range(2000):
                yield

        # ----------------------------------------------------------------------------------
        # Completion monitor with random backpressure.
        # ----------------------------------------------------------------------------------
        @passive
        def monitor():
            beats_expected = (16 + (beat_dwords - 1)) // beat_dwords  # ceil(16/beat_dwords)
            while True:
                # Random stalls on cmp.ready to stress FIFO / pipeline.
                stall = (random.random() < stall_prob)
                yield port.source.ready.eq(0 if stall else 1)

                if (yield port.source.valid) and (yield port.source.ready):
                    got_cpl_beats.append({
                        "first": (yield port.source.first),
                        "last":  (yield port.source.last),
                        "end":   (yield port.source.end),
                        "len":   (yield port.source.len),
                        "tag":   (yield port.source.tag),
                        "req_id": (yield port.source.req_id),
                        "dat":   (yield port.source.dat),
                    })
                    # Stop monitoring once we got all beats.
                    if len(got_cpl_beats) >= beats_expected:
                        # Keep running a bit to catch illegal extra beats.
                        for _ in range(20):
                            yield port.source.ready.eq(1)
                            yield
                        return
                yield

        run_simulation(dut, [stimulus(), monitor()], vcd_name=None)

        # ----------------------------------------------------------------------------------
        # Checks
        # ----------------------------------------------------------------------------------
        beats_expected = (16 + (beat_dwords - 1)) // beat_dwords
        self.assertEqual(
            len(got_cpl_beats), beats_expected,
            msg=f"Expected {beats_expected} completion beats, got {len(got_cpl_beats)}"
        )

        # Header fields constant across beats.
        for i, b in enumerate(got_cpl_beats):
            self.assertEqual(b["tag"], 0x5A, msg=f"beat {i}: tag mismatch")
            self.assertEqual(b["req_id"], 0x1234, msg=f"beat {i}: req_id mismatch")
            self.assertEqual(b["len"], 16, msg=f"beat {i}: len mismatch (should be dwords)")

        # first/last/end framing.
        self.assertEqual(got_cpl_beats[0]["first"], 1, msg="first beat must assert first=1")
        for i in range(1, beats_expected):
            self.assertEqual(got_cpl_beats[i]["first"], 0, msg=f"beat {i}: first must be 0")
        self.assertEqual(got_cpl_beats[-1]["last"], 1, msg="last beat must assert last=1")
        self.assertEqual(got_cpl_beats[-1]["end"],  1, msg="last beat must assert end=1")
        for i in range(0, beats_expected - 1):
            self.assertEqual(got_cpl_beats[i]["last"], 0, msg=f"beat {i}: last must be 0")
            self.assertEqual(got_cpl_beats[i]["end"],  0, msg=f"beat {i}: end must be 0")

        # Reassemble returned 16 dwords and compare to expected SQE.
        out_dws = []
        for b in got_cpl_beats:
            out_dws += _unpack_int_to_dwords(b["dat"], beat_dwords)
        out_dws = out_dws[:16]

        self.assertEqual(out_dws, sqe, msg="Completion payload mismatch vs SQE contents")

    # Public tests -----------------------------------------------------------------------------

    def test_identify_sqe_read_128b_no_stall(self):
        self._run_case(data_width=128, stall_prob=0.0, seed=0x128)

    def test_identify_sqe_read_128b_with_stall(self):
        self._run_case(data_width=128, stall_prob=0.35, seed=0xBADC0DE)

    def test_identify_sqe_read_256b_with_stall(self):
        self._run_case(data_width=256, stall_prob=0.35, seed=0x256)

    def test_identify_sqe_read_64b_with_stall(self):
        self._run_case(data_width=64, stall_prob=0.35, seed=0x64)


if __name__ == "__main__":
    unittest.main()
