#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest

from litenvme.gen import load_config, _resolve_platform, LiteNVMeCore

# Helpers ------------------------------------------------------------------------------------------

def elaborate(**overrides):
    cfg = load_config(os.path.join("examples", "alibaba_xcku3p.yml"))
    cfg.update(overrides)
    platform = _resolve_platform(cfg)
    soc = LiteNVMeCore(platform, cfg)
    soc.finalize()
    return soc

def csr_banks(soc):
    return {name: region.obj for name, region in soc.csr.regions.items() if isinstance(region.obj, list)}

# Test Generated Core CSRs -------------------------------------------------------------------------

class TestGeneratedCoreCSRs(unittest.TestCase):
    def test_firmware_mode_csrs_registered_once(self):
        soc   = elaborate()
        banks = csr_banks(soc)
        # The core's sub-blocks are collected once, under the "nvme" bank.
        self.assertIn("nvme", banks)
        for name in ["nvme_cfg", "nvme_mmio", "nvme_engine"]:
            self.assertNotIn(name, banks)
        # No CSR object is mapped in two banks.
        seen = {}
        for bank, csrs in banks.items():
            for csr in csrs:
                self.assertNotIn(id(csr), seen, f"{csr.name} in {seen.get(id(csr))} and {bank}")
                seen[id(csr)] = bank
        names = [csr.name for csr in banks["nvme"]]
        for name in ["cfg_cfg_ctrl", "mmio_mem_ctrl", "io_engine_engine_enable"]:
            self.assertIn(name, names)

    def test_rtl_mode_has_no_nvme_csrs(self):
        soc   = elaborate(init_mode="rtl", cpu=None, uart="stub", firmware="none")
        banks = csr_banks(soc)
        self.assertFalse(any(name.startswith("nvme") for name in banks))


if __name__ == "__main__":
    unittest.main()
