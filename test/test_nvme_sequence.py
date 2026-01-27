# test/test_nvme_sequence.py
#
# Full-ish NVMe sequence simulation around LiteNVMeHostMemResponder:
# - Host preloads ASQ[0] (SQE) through CSR dword interface.
# - NVMe issues MemRd for SQE (64B), Host responds with CplD beats.
# - NVMe writes Identify buffer (PRP1) through MemWr beats.
# - NVMe writes ACQ[0] CQE through MemWr beats.
# - NVMe reads back Identify buffer + CQE via MemRd and checks content.
# - Optional: CSR-only memory R/W self-test in a scratch region.
#
# This validates:
# - MemRd path emits all beats correctly under backpressure.
# - MemWr path absorbs 1 beat/cycle.
# - BRAM contents written by MemWr are readable via MemRd (i.e. real datapath).
# - CSR debug port basic lane mapping works (CSR-only selftest), without relying on CSR
#   to validate DMA-written beat-wide contents.
#
# IMPORTANT:
# - The responder only supports ONE in-flight MemRd.
#   So the TB must not issue a second MemRd until all completions of the first MemRd
#   have been observed (drained).

import unittest
import random
from pathlib import Path
import sys

from migen import *
from migen.sim import run_simulation, passive

from litex.soc.interconnect import stream

from litenvme.hostmem import LiteNVMeHostMemResponder

# ------------------------------------------------------------------------------------------
# Minimal "LitePCIe user port" layouts matching what LiteNVMeHostMemResponder uses.
# ------------------------------------------------------------------------------------------

def _req_layout(data_width, beat_bytes):
    return [
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

class TestNVMeSequenceHostMem(unittest.TestCase):
    def _run_case(self, data_width=128, stall_prob=0.25, seed=0xC001D00D, do_csr_rw_test=True):
        random.seed(seed)

        base = 0x1000_0000
        size = 0x4000  # 16KB window.

        port = _Port(data_width=data_width)
        dut  = LiteNVMeHostMemResponder(
            port       = port,
            base       = base,
            size       = size,
            data_width = data_width,
            with_csr   = True,
        )

        beat_bytes  = data_width // 8
        beat_dwords = data_width // 32
        full_be     = (1 << beat_bytes) - 1

        # ----------------------------------------------------------------------
        # Addresses inside the hostmem window.
        # ----------------------------------------------------------------------
        asq_addr = base + 0x0000
        acq_addr = base + 0x1000
        id_buf   = base + 0x2000

        # Small identify buffer in simulation for speed.
        identify_bytes  = 256
        identify_dwords = identify_bytes // 4

        # ----------------------------------------------------------------------
        # Build Identify Controller SQE (16 DW = 64B).
        # ----------------------------------------------------------------------
        cid = 0x0001
        sqe = [0] * 16
        sqe[0]  = (0x06 & 0xFF) | ((cid & 0xFFFF) << 16)  # opcode + CID
        sqe[6]  = id_buf & 0xFFFFFFFF                     # PRP1 low
        sqe[7]  = (id_buf >> 32) & 0xFFFFFFFF             # PRP1 high
        sqe[10] = 0x00000001                              # CNS=1 (Identify Controller)

        identify_payload = [((0xA5A50000 ^ i) & 0xFFFFFFFF) for i in range(identify_dwords)]
        cqe = [0xDEADBEEF, 0x00010001, 0x00000000, 0xABCD1234]

        # ----------------------------------------------------------------------
        # TB helpers: CSR dword R/W (debug access).
        # ----------------------------------------------------------------------
        def abs_to_dw_index(abs_addr):
            return (abs_addr - base) // 4

        def csr_write_dw(dw_index, value):
            yield dut.csr._csr_adr.storage.eq(dw_index)
            yield dut.csr._csr_wdata.storage.eq(value)
            yield dut.csr._csr_we.storage.eq(1)
            yield
            yield dut.csr._csr_we.storage.eq(0)
            for _ in range(5):
                yield

        def csr_read_dw(dw_index):
            yield dut.csr._csr_adr.storage.eq(dw_index)
            for _ in range(6):
                yield
            return (yield dut.csr._csr_rdata.status)

        # ----------------------------------------------------------------------
        # TB helpers: NVMe-side MemRd / MemWr into hostmem responder.
        #
        # IMPORTANT: In Migen simulation, changing signals after a `yield` only
        # takes effect on the next cycle. If you keep valid high across beats,
        # you can accidentally "replay" the previous beat for one extra cycle.
        #
        # So we use 1-cycle valid pulses per beat (wait-ready, then pulse valid).
        # ----------------------------------------------------------------------

        def _pulse_req_common(addr, we, len_dw, tag, req_id):
            yield port.sink.adr.eq(addr)
            yield port.sink.we.eq(we)
            yield port.sink.len.eq(len_dw)
            yield port.sink.tag.eq(tag)
            yield port.sink.req_id.eq(req_id)
            yield port.sink.be.eq(full_be)

        def nvme_send_memrd(addr, len_dw, tag=0x5A, req_id=0x1234):
            # Wait until DUT is ready to accept the request, then pulse valid for 1 cycle.
            yield from _pulse_req_common(addr, we=0, len_dw=len_dw, tag=tag, req_id=req_id)
            yield port.sink.dat.eq(0)
            yield port.sink.first.eq(1)
            yield port.sink.last.eq(1)

            while (yield port.sink.ready) == 0:
                yield

            yield port.sink.valid.eq(1)
            yield  # handshake cycle
            yield port.sink.valid.eq(0)
            yield port.sink.first.eq(0)
            yield port.sink.last.eq(0)
            yield  # bubble cycle (prevents accidental replay)

        def nvme_send_memwr(addr, payload_dwords, tag=0x22, req_id=0x5678):
            total_dw = len(payload_dwords)
            assert total_dw > 0

            nbeats = (total_dw + (beat_dwords - 1)) // beat_dwords

            # Program constant header fields once.
            yield from _pulse_req_common(addr, we=1, len_dw=total_dw, tag=tag, req_id=req_id)

            for bi in range(nbeats):
                chunk = payload_dwords[bi*beat_dwords:(bi+1)*beat_dwords]
                if len(chunk) < beat_dwords:
                    chunk = chunk + [0] * (beat_dwords - len(chunk))

                yield port.sink.dat.eq(_pack_dwords_to_int(chunk))
                yield port.sink.first.eq(1 if bi == 0 else 0)
                yield port.sink.last.eq(1 if bi == (nbeats - 1) else 0)

                # Wait for ready, then pulse valid for exactly one cycle.
                while (yield port.sink.ready) == 0:
                    yield

                yield port.sink.valid.eq(1)
                yield  # handshake cycle
                yield port.sink.valid.eq(0)
                yield  # bubble cycle (prevents accidental replay)

            yield port.sink.first.eq(0)
            yield port.sink.last.eq(0)


        # ----------------------------------------------------------------------
        # Completion collector: bucket by tag.
        # ----------------------------------------------------------------------
        cpl_by_tag = {}  # tag -> list of beats

        @passive
        def completion_monitor():
            while True:
                stall = (random.random() < stall_prob)
                yield port.source.ready.eq(0 if stall else 1)

                if (yield port.source.valid) and (yield port.source.ready):
                    tag = (yield port.source.tag)
                    if tag not in cpl_by_tag:
                        cpl_by_tag[tag] = []
                    cpl_by_tag[tag].append({
                        "first": (yield port.source.first),
                        "last":  (yield port.source.last),
                        "end":   (yield port.source.end),
                        "len":   (yield port.source.len),
                        "tag":   tag,
                        "req_id": (yield port.source.req_id),
                        "dat":   (yield port.source.dat),
                    })
                yield

        @passive
        def request_monitor():
            cyc = 0
            while True:
                if (yield port.sink.valid) and (yield port.sink.ready):
                    print(f"[REQ ] cyc={cyc} we={(yield port.sink.we)} tag=0x{(yield port.sink.tag):02x} "
                          f"req_id=0x{(yield port.sink.req_id):04x} len_dw={(yield port.sink.len)} "
                          f"first={(yield port.sink.first)} last={(yield port.sink.last)} "
                          f"adr=0x{(yield port.sink.adr):016x}")
                cyc += 1
                yield

        # ----------------------------------------------------------------------
        # Debug traces.
        # ----------------------------------------------------------------------
        @passive
        def req_trace():
            while True:
                if (yield port.sink.valid) and (yield port.sink.ready):
                    print(
                        f"[REQ ] we={(yield port.sink.we)} "
                        f"tag=0x{(yield port.sink.tag):02x} "
                        f"req_id=0x{(yield port.sink.req_id):04x} "
                        f"len_dw={(yield port.sink.len)} "
                        f"adr=0x{(yield port.sink.adr):016x} "
                        f"first={(yield port.sink.first) if hasattr(port.sink, 'first') else 1} "
                        f"last={(yield port.sink.last) if hasattr(port.sink, 'last') else 1}"
                    )
                yield

        @passive
        def cpl_trace():
            while True:
                v = (yield port.source.valid)
                r = (yield port.source.ready)
                if v and not r:
                    print(
                        f"[CPL?] VALID but READY=0 "
                        f"tag=0x{(yield port.source.tag):02x} "
                        f"req_id=0x{(yield port.source.req_id):04x} "
                        f"len_dw={(yield port.source.len)} "
                        f"end={(yield port.source.end)}"
                    )
                if v and r:
                    print(
                        f"[CPL ] tag=0x{(yield port.source.tag):02x} "
                        f"req_id=0x{(yield port.source.req_id):04x} "
                        f"len_dw={(yield port.source.len)} "
                        f"end={(yield port.source.end)} "
                        f"dat=0x{(yield port.source.dat):0{data_width//4}x}"
                    )
                yield

        @passive
        def heartbeat():
            cyc = 0
            while True:
                if (cyc % 500) == 0:
                    print(
                        f"[HB  ] cyc={cyc} "
                        f"sink(v/r)={(yield port.sink.valid)}/{(yield port.sink.ready)} "
                        f"src(v/r)={(yield port.source.valid)}/{(yield port.source.ready)}"
                    )
                cyc += 1
                yield

        # ----------------------------------------------------------------------
        # Optional CSR-only memory R/W self-test.
        # ----------------------------------------------------------------------
        csr_rw_ok = []

        def csr_mem_rw_selftest():
            scratch = base + 0x3000
            scratch_dw = abs_to_dw_index(scratch)

            for i in range(16):
                pat = (0x12340000 ^ (i * 0x1111)) & 0xFFFFFFFF
                yield from csr_write_dw(scratch_dw + i, pat)

            ok = True
            for i in range(16):
                exp = (0x12340000 ^ (i * 0x1111)) & 0xFFFFFFFF
                got = yield from csr_read_dw(scratch_dw + i)
                if got != exp:
                    ok = False
            csr_rw_ok.append(ok)

        # ----------------------------------------------------------------------
        # Wait for completions (with progress + snapshot on timeout).
        # ----------------------------------------------------------------------
        def wait_cpl(tag, beats_expected, timeout_cycles=5000):
            last_n = -1
            for t in range(timeout_cycles):
                n = len(cpl_by_tag.get(tag, []))
                if n != last_n and (n == 0 or (n % 4) == 0):
                    print(f"[WAIT] tag=0x{tag:02x} got={n}/{beats_expected} at t={t}")
                    last_n = n
                if n >= beats_expected:
                    return
                yield

            print(
                f"[TIMEOUT] tag=0x{tag:02x} expected={beats_expected} got={len(cpl_by_tag.get(tag, []))} "
                f"sink(v/r)={(yield port.sink.valid)}/{(yield port.sink.ready)} "
                f"src(v/r)={(yield port.source.valid)}/{(yield port.source.ready)} "
                f"src_tag=0x{(yield port.source.tag):02x} src_len={(yield port.source.len)}"
            )
            raise TimeoutError(
                f"Timeout waiting for tag=0x{tag:02x} beats={beats_expected} "
                f"(got {len(cpl_by_tag.get(tag, []))})"
            )

        # ----------------------------------------------------------------------
        # Main stimulus (single simulation run).
        # ----------------------------------------------------------------------
        def stimulus():
            yield port.sink.valid.eq(0)
            yield port.sink.first.eq(0)
            yield port.sink.last.eq(0)
            yield port.sink.adr.eq(0)
            yield port.sink.we.eq(0)
            yield port.sink.len.eq(0)
            yield port.sink.tag.eq(0)
            yield port.sink.req_id.eq(0)
            yield port.sink.dat.eq(0)
            yield port.sink.be.eq(full_be)
            yield

            if do_csr_rw_test:
                yield from csr_mem_rw_selftest()

            # 1) Host preloads ASQ[0] via CSR writes.
            for i, dw in enumerate(sqe):
                yield from csr_write_dw(abs_to_dw_index(asq_addr) + i, dw)

            # 2) NVMe reads SQE via MemRd (tag 0x5A).
            yield from nvme_send_memrd(asq_addr, len_dw=16, tag=0x5A, req_id=0x1234)
            yield from wait_cpl(tag=0x5A, beats_expected=(16 + (beat_dwords - 1)) // beat_dwords)

            # 3) NVMe writes Identify buffer at PRP1.
            yield from nvme_send_memwr(id_buf, identify_payload, tag=0x33, req_id=0x7777)

            # 4) NVMe writes CQE[0] at ACQ.
            yield from nvme_send_memwr(acq_addr, cqe, tag=0x44, req_id=0x8888)

            for _ in range(80):
                yield

            # 5) NVMe reads back Identify buffer (tag 0x66) THEN waits for it to finish.
            yield from nvme_send_memrd(id_buf, len_dw=identify_dwords, tag=0x66, req_id=0xCAFE)
            yield from wait_cpl(tag=0x66, beats_expected=(identify_dwords + (beat_dwords - 1)) // beat_dwords)

            # 6) NVMe reads back CQE (tag 0x77) after previous MemRd fully drained.
            yield from nvme_send_memrd(acq_addr, len_dw=4, tag=0x77, req_id=0xBEEF)
            yield from wait_cpl(tag=0x77, beats_expected=(4 + (beat_dwords - 1)) // beat_dwords)

            for _ in range(50):
                yield

        # NOTE: set VCD to inspect waveforms when it wedges.
        run_simulation(
            dut,
            [stimulus(), completion_monitor(), request_monitor(), req_trace(), cpl_trace(), heartbeat()],
            vcd_name="nvme_sequence.vcd"
        )

        # ----------------------------------------------------------------------
        # Helpers to decode completions.
        # ----------------------------------------------------------------------
        def reassemble_dwords(beats, n_dwords):
            out = []
            for b in beats:
                out += _unpack_int_to_dwords(b["dat"], beat_dwords)
            return out[:n_dwords]

        def check_cpl_stream(tag, exp_req_id, exp_len_dw, exp_payload_dws):
            self.assertIn(tag, cpl_by_tag, msg=f"Missing completions for tag=0x{tag:02x}")
            beats = cpl_by_tag[tag]

            beats_expected = (exp_len_dw + (beat_dwords - 1)) // beat_dwords
            self.assertEqual(len(beats), beats_expected,
                             msg=f"tag=0x{tag:02x}: expected {beats_expected} beats, got {len(beats)}")

            for i, b in enumerate(beats):
                self.assertEqual(b["req_id"], exp_req_id, msg=f"tag=0x{tag:02x} beat {i}: req_id mismatch")
                self.assertEqual(b["len"],    exp_len_dw, msg=f"tag=0x{tag:02x} beat {i}: len mismatch")

            self.assertEqual(beats[0]["first"], 1, msg=f"tag=0x{tag:02x}: first not asserted")
            for i in range(1, beats_expected):
                self.assertEqual(beats[i]["first"], 0, msg=f"tag=0x{tag:02x} beat {i}: first asserted")
            self.assertEqual(beats[-1]["last"], 1, msg=f"tag=0x{tag:02x}: last not asserted")
            self.assertEqual(beats[-1]["end"],  1, msg=f"tag=0x{tag:02x}: end not asserted")

            got = reassemble_dwords(beats, exp_len_dw)
            self.assertEqual(got, exp_payload_dws, msg=f"tag=0x{tag:02x}: payload mismatch")

        # ----------------------------------------------------------------------
        # Checks.
        # ----------------------------------------------------------------------
        check_cpl_stream(tag=0x5A, exp_req_id=0x1234, exp_len_dw=16,              exp_payload_dws=sqe)
        check_cpl_stream(tag=0x66, exp_req_id=0xCAFE, exp_len_dw=identify_dwords, exp_payload_dws=identify_payload)
        check_cpl_stream(tag=0x77, exp_req_id=0xBEEF, exp_len_dw=4,               exp_payload_dws=cqe)

        if do_csr_rw_test:
            self.assertTrue(len(csr_rw_ok) == 1 and csr_rw_ok[0] is True,
                            msg="CSR-only memory R/W self-test failed")

    # Public tests -----------------------------------------------------------------------------

    def test_full_sequence_64b(self):
        self._run_case(data_width=64, stall_prob=0.35, seed=0x64)

    def test_full_sequence_128b(self):
        self._run_case(data_width=128, stall_prob=0.25, seed=0x128)

    def test_full_sequence_256b(self):
        self._run_case(data_width=256, stall_prob=0.25, seed=0x256)


if __name__ == "__main__":
    unittest.main()
