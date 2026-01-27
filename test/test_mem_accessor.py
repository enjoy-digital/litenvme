# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause
#
# Unit tests for LiteNVMePCIeMmioAccessor:
# - Read/write request fields and completion handling.
# - Invalid wsel handling.
# - Timeout behavior.

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.gen import *
from litex.soc.interconnect import stream

from litenvme.mem import LiteNVMePCIeMmioAccessor


# Minimal "LitePCIe user port" layouts matching what LiteNVMePCIeMmioAccessor uses.

def _req_layout(data_width):
    return [
        ("adr",     64),
        ("we",      1),
        ("len",    10),
        ("tag",     8),
        ("dat", data_width),
        ("channel", 2),
    ]


def _cmp_layout(data_width):
    return [
        ("dat", data_width),
        ("err", 1),
    ]


class _Port(Module):
    def __init__(self, data_width):
        self.channel = Signal(2, reset=0)
        self.source  = stream.Endpoint(_req_layout(data_width))
        self.sink    = stream.Endpoint(_cmp_layout(data_width))


# Test

class TestMemAccessor(unittest.TestCase):
    def _run_read(self, rdata=0x11223344, timeout=64):
        port = _Port(data_width=128)
        dut  = LiteNVMePCIeMmioAccessor(port, tag=0x55, timeout=timeout)

        captured = {}
        last = {"done": 0, "err": 0, "rdata": 0}

        def stimulus():
            yield dut.adr.eq(0x1234_5678)
            yield dut.len.eq(1)
            yield dut.wsel.eq(0xF)
            yield dut.we.eq(0)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)

            for _ in range(timeout + 20):
                last["done"] = (yield dut.done)
                last["err"] = (yield dut.err)
                last["rdata"] = (yield dut.rdata)
                if (yield dut.done):
                    break
                yield

        @passive
        def driver():
            yield port.source.ready.eq(1)
            yield port.sink.valid.eq(0)
            yield port.sink.err.eq(0)
            yield port.sink.dat.eq(0)
            while True:
                if (yield port.source.valid) and (yield port.source.ready):
                    captured["we"] = (yield port.source.we)
                    captured["len"] = (yield port.source.len)
                    captured["tag"] = (yield port.source.tag)
                    captured["adr"] = (yield port.source.adr)
                    for _ in range(2):
                        yield
                    yield port.sink.valid.eq(1)
                    yield port.sink.dat.eq(rdata)
                    while (yield port.sink.ready) == 0:
                        yield
                    yield
                    yield port.sink.valid.eq(0)
                    return
                yield

        run_simulation(dut, [stimulus(), driver()], vcd_name=None)

        self.assertEqual(captured.get("we"), 0)
        self.assertEqual(captured.get("len"), 1)
        self.assertEqual(captured.get("tag"), 0x55)
        self.assertEqual(captured.get("adr"), 0x1234_5678)

        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 0)
        self.assertEqual(last["rdata"], rdata)

    def _run_write(self, wdata=0xAABBCCDD, timeout=64):
        port = _Port(data_width=128)
        dut  = LiteNVMePCIeMmioAccessor(port, tag=0x66, timeout=timeout)

        captured = {}
        last = {"done": 0, "err": 0}

        def stimulus():
            yield dut.adr.eq(0xDEAD_BEEF)
            yield dut.len.eq(1)
            yield dut.wsel.eq(0xF)
            yield dut.we.eq(1)
            yield dut.wdata.eq(wdata)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)

            for _ in range(timeout + 20):
                last["done"] = (yield dut.done)
                last["err"] = (yield dut.err)
                if (yield dut.done):
                    break
                yield

        @passive
        def driver():
            yield port.source.ready.eq(1)
            yield port.sink.valid.eq(0)
            yield port.sink.err.eq(0)
            yield port.sink.dat.eq(0)
            while True:
                if (yield port.source.valid) and (yield port.source.ready):
                    captured["we"] = (yield port.source.we)
                    captured["adr"] = (yield port.source.adr)
                    captured["dat"] = (yield port.source.dat)
                    for _ in range(2):
                        yield
                    yield port.sink.valid.eq(1)
                    while (yield port.sink.ready) == 0:
                        yield
                    yield
                    yield port.sink.valid.eq(0)
                    return
                yield

        run_simulation(dut, [stimulus(), driver()], vcd_name=None)

        self.assertEqual(captured.get("we"), 1)
        self.assertEqual(captured.get("adr"), 0xDEAD_BEEF)
        for i in range(4):
            shift = 32 * i
            self.assertEqual((captured["dat"] >> shift) & 0xFFFFFFFF, wdata)

        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 0)

    def _run_bad_wsel(self):
        port = _Port(data_width=64)
        dut  = LiteNVMePCIeMmioAccessor(port, tag=0x00, timeout=16)

        seen_req = {"count": 0}
        last = {"done": 0, "err": 0}

        def stimulus():
            yield dut.we.eq(1)
            yield dut.wsel.eq(0x7)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            for _ in range(20):
                last["done"] = (yield dut.done)
                last["err"] = (yield dut.err)
                yield

        @passive
        def driver():
            yield port.source.ready.eq(1)
            while True:
                if (yield port.source.valid) and (yield port.source.ready):
                    seen_req["count"] += 1
                yield

        run_simulation(dut, [stimulus(), driver()], vcd_name=None)

        self.assertEqual(seen_req["count"], 0)
        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 1)

    def _run_timeout(self):
        port = _Port(data_width=64)
        dut  = LiteNVMePCIeMmioAccessor(port, tag=0x00, timeout=8)

        last = {"done": 0, "err": 0}

        def stimulus():
            yield dut.we.eq(0)
            yield dut.wsel.eq(0xF)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            for _ in range(40):
                last["done"] = (yield dut.done)
                last["err"] = (yield dut.err)
                if (yield dut.done):
                    break
                yield

        @passive
        def driver():
            yield port.source.ready.eq(1)
            yield port.sink.valid.eq(0)
            while True:
                yield

        run_simulation(dut, [stimulus(), driver()], vcd_name=None)

        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 1)

    def test_mem_read(self):
        self._run_read()

    def test_mem_write(self):
        self._run_write()

    def test_mem_bad_wsel(self):
        self._run_bad_wsel()

    def test_mem_timeout(self):
        self._run_timeout()


if __name__ == "__main__":
    unittest.main()
