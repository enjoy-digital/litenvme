#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest

from litenvme.gen import load_config, _resolve_platform, LiteNVMeCore

# Helpers ------------------------------------------------------------------------------------------

def elaborate_rtl(**overrides):
    cfg = load_config(os.path.join("examples", "alibaba_xcku3p.yml"))
    cfg.update(dict(init_mode="rtl", cpu=None, uart="stub", firmware="none"), **overrides)
    platform = _resolve_platform(cfg)
    soc = LiteNVMeCore(platform, cfg)
    return soc, platform

def has_false_path(platform, a, b):
    return any({id(x), id(y)} == {id(a), id(b)} for x, y in platform.toolchain.false_paths)

# Test Generated Core Clocking ---------------------------------------------------------------------

class TestGeneratedCoreClocking(unittest.TestCase):
    def test_clk_external_has_sys_pcie_false_path(self):
        soc, platform = elaborate_rtl(clk_external=True)
        self.assertTrue(has_false_path(platform, soc.crg.cd_sys.clk, soc.pcie_phy.cd_pcie.clk))

    def test_clk_internal_has_no_sys_pcie_false_path(self):
        # Internal mode: sys is the PCIe user clock (same clock, no false path).
        soc, platform = elaborate_rtl(clk_external=False)
        self.assertFalse(has_false_path(platform, soc.crg.cd_sys.clk, soc.pcie_phy.cd_pcie.clk))


if __name__ == "__main__":
    unittest.main()
