# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause
#
# Simulation of LiteNVMeMemPortToAXI against the real LiteNVMeHostMemAXIRAM backend.
#
# Validates the dword <-> beat-wide AXI bridge end to end on the actual backend RTL:
# - dword writes land in the correct beat + lane (byte-strobed, no clobber of neighbours),
# - dword reads return the correct lane,
# - addressing is correct across lane boundaries and across beats,
# for data_width in {64, 128, 256} (i.e. beat_dwords in {2, 4, 8}).

import unittest

from migen import *
from migen.sim import run_simulation

from litenvme.io_engine import LiteNVMeMemPortToAXI, LiteNVMeMemPort
from litenvme.hostmem import LiteNVMeHostMemAXIRAM


class _Bridged(Module):
    def __init__(self, data_width, size):
        self.mem = LiteNVMeMemPort(adr_width=32)
        self.submodules.bridge  = bridge  = LiteNVMeMemPortToAXI(self.mem, data_width=data_width)
        self.submodules.backend = backend = LiteNVMeHostMemAXIRAM(size=size, data_width=data_width)
        self.comb += bridge.axi.connect(backend.axi)


class TestMemPortToAXI(unittest.TestCase):
    def _run_case(self, data_width=128, n_dwords=40):
        size = 0x1000
        dut = _Bridged(data_width=data_width, size=size)

        written = {}

        def mem_write(dw_adr, value):
            yield dut.mem.adr.eq(dw_adr)
            yield dut.mem.dat_w.eq(value & 0xffffffff)
            yield dut.mem.we.eq(1)
            yield dut.mem.stb.eq(1)
            yield
            cyc = 0
            while (yield dut.mem.ack) == 0:
                yield
                cyc += 1
                assert cyc < 1000, "write ack timeout"
            yield dut.mem.stb.eq(0)
            yield dut.mem.we.eq(0)
            yield  # bubble

        def mem_read(dw_adr):
            yield dut.mem.adr.eq(dw_adr)
            yield dut.mem.we.eq(0)
            yield dut.mem.stb.eq(1)
            yield
            cyc = 0
            while (yield dut.mem.ack) == 0:
                yield
                cyc += 1
                assert cyc < 1000, "read ack timeout"
            val = (yield dut.mem.dat_r)
            yield dut.mem.stb.eq(0)
            yield  # bubble
            return val

        def stim():
            yield dut.mem.stb.eq(0)
            yield
            # Write a distinct value to each dword (spans multiple beats + all lanes).
            for i in range(n_dwords):
                v = (0xC0DE0000 + i) & 0xffffffff
                written[i] = v
                yield from mem_write(i, v)
            # Read each back and check.
            for i in range(n_dwords):
                got = yield from mem_read(i)
                assert got == written[i], \
                    f"dw {i}: got 0x{got:08x} exp 0x{written[i]:08x} (dw={data_width})"
            # Overwrite one lane in the middle and confirm neighbours are intact.
            yield from mem_write(5, 0xABCD1234)
            written[5] = 0xABCD1234
            for i in (3, 4, 5, 6, 7):
                got = yield from mem_read(i)
                assert got == written[i], \
                    f"after RMW, dw {i}: got 0x{got:08x} exp 0x{written[i]:08x}"

        run_simulation(dut, [stim()], vcd_name="io_engine_axi.vcd")

    def test_bridge_64b(self):
        self._run_case(data_width=64)

    def test_bridge_128b(self):
        self._run_case(data_width=128)

    def test_bridge_256b(self):
        self._run_case(data_width=256)


if __name__ == "__main__":
    unittest.main()
