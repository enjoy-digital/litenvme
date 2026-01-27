# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause
#
# Unit tests for LiteNVMePCIeCfgAccessor:
# - Read transaction fields and completion handling.
# - Write transaction fields and completion handling.
# - Timeout behavior.

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.gen import *
from litex.soc.interconnect import stream

from litenvme.cfg import LiteNVMePCIeCfgAccessor


# Minimal "LitePCIe user port" layouts matching what LiteNVMePCIeCfgAccessor uses.

def _req_layout(data_width):
    return [
        ("adr",          64),
        ("we",           1),
        ("len",         10),
        ("tag",          8),
        ("req_id",      16),
        ("dat",  data_width),
        ("channel",      2),
        ("is_cfg",       1),
        ("bus_number",   8),
        ("device_no",    5),
        ("func",         3),
        ("ext_reg",      3),
        ("register_no",  6),
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

class TestCfgAccessor(unittest.TestCase):
    def _run_read(self, rdata=0x12345678, timeout=64):
        port = _Port(data_width=128)
        dut  = LiteNVMePCIeCfgAccessor(port, requester_id=0xBEEF, tag=0x5A, timeout=timeout)

        captured = {}
        last = {"done": 0, "err": 0, "rdata": 0}

        def stimulus():
            yield dut.bus.eq(0x12)
            yield dut.device.eq(0x1a)
            yield dut.function.eq(0x3)
            yield dut.reg.eq(0x10)
            yield dut.ext_reg.eq(0x4)
            yield dut.we.eq(0)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)

            # Wait for completion.
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
                    captured["req_id"] = (yield port.source.req_id)
                    captured["is_cfg"] = (yield port.source.is_cfg)
                    captured["bus_number"] = (yield port.source.bus_number)
                    captured["device_no"] = (yield port.source.device_no)
                    captured["func"] = (yield port.source.func)
                    captured["ext_reg"] = (yield port.source.ext_reg)
                    captured["register_no"] = (yield port.source.register_no)
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
        self.assertEqual(captured.get("tag"), 0x5A)
        self.assertEqual(captured.get("req_id"), 0xBEEF)
        self.assertEqual(captured.get("is_cfg"), 1)
        self.assertEqual(captured.get("bus_number"), 0x12)
        self.assertEqual(captured.get("device_no"), 0x1A)
        self.assertEqual(captured.get("func"), 0x3)
        self.assertEqual(captured.get("ext_reg"), 0x4)
        self.assertEqual(captured.get("register_no"), 0x10)

        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 0)
        self.assertEqual(last["rdata"], rdata)

    def _run_write(self, wdata=0xDEADBEEF, timeout=64):
        port = _Port(data_width=128)
        dut  = LiteNVMePCIeCfgAccessor(port, requester_id=0x1234, tag=0xA5, timeout=timeout)

        captured = {}
        last = {"done": 0, "err": 0}

        def stimulus():
            yield dut.bus.eq(0x00)
            yield dut.device.eq(0x00)
            yield dut.function.eq(0x0)
            yield dut.reg.eq(0x01)
            yield dut.ext_reg.eq(0x0)
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
        for i in range(4):
            shift = 32 * i
            self.assertEqual((captured["dat"] >> shift) & 0xFFFFFFFF, wdata)

        self.assertEqual(last["done"], 1)
        self.assertEqual(last["err"], 0)

    def _run_timeout(self):
        port = _Port(data_width=64)
        dut  = LiteNVMePCIeCfgAccessor(port, requester_id=0x0000, tag=0x00, timeout=8)
        last = {"done": 0, "err": 0}

        def stimulus():
            yield dut.we.eq(0)
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

    def test_cfg_read(self):
        self._run_read()

    def test_cfg_write(self):
        self._run_write()

    def test_cfg_timeout(self):
        self._run_timeout()


if __name__ == "__main__":
    unittest.main()
