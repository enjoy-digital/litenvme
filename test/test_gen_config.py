# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import shutil
import tempfile
import unittest
import subprocess

from litenvme.gen import _normalize_config, load_config, validate_config, prp_list_offset
from litenvme.gen import _resolve_platform, LiteNVMeCore


BASE_CONFIG = {
    "phy"        : "USPPCIEPHY",
    "phy_device" : "xcku3p-ffvb676-2-e",
}

EXAMPLE = os.path.join("examples", "alibaba_xcku3p.yml")

def example_config(**overrides):
    cfg = load_config(EXAMPLE)
    cfg.update(overrides)
    return cfg

def elaborate(cfg):
    platform = _resolve_platform(cfg)
    soc = LiteNVMeCore(platform, cfg)
    soc.finalize()
    return soc

def riscv_toolchain():
    return any(shutil.which(f"{t}-gcc") for t in ["riscv64-unknown-elf", "riscv-none-elf", "riscv64-linux-gnu"])


class TestGeneratorConfig(unittest.TestCase):
    def test_example_config_loads(self):
        cfg = load_config(os.path.join("examples", "alibaba_xcku3p.yml"))
        self.assertEqual(cfg["init_mode"], "firmware")
        # The example core self-initializes (status_init_done rises): "none" boots the BIOS.
        self.assertEqual(cfg["firmware"], "minimal")
        self.assertFalse(cfg["with_request_gen"])
        self.assertTrue(cfg["with_block_streamer"])

    def test_unknown_key_rejected(self):
        cfg = dict(BASE_CONFIG, unknown_option=True)
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "Unknown LiteNVMe config key"):
            validate_config(cfg)

    def test_bad_init_mode_rejected(self):
        cfg = dict(BASE_CONFIG, init_mode="software")
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "init_mode"):
            validate_config(cfg)

    def test_rtl_init_defaults_to_cpuless(self):
        # qsize 8: with the default 64-entry ring, the PRP-list region (256 KiB) does not fit
        # next to the default 256 KiB staging region in the 512 KiB window.
        cfg = dict(BASE_CONFIG, init_mode="rtl", with_block_streamer=True, qsize=8, qd=4)
        cfg = _normalize_config(cfg)
        validate_config(cfg)
        self.assertIsNone(cfg["cpu"])
        self.assertEqual(cfg["uart"], "stub")
        self.assertEqual(cfg["firmware"], "none")
        self.assertFalse(cfg["with_request_gen"])

    def test_firmware_mode_requires_cpu(self):
        cfg = dict(BASE_CONFIG, init_mode="firmware", cpu=None)
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "requires a CPU"):
            validate_config(cfg)

    def test_load_config_uses_safe_loader(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
            f.write("{'phy': 'USPPCIEPHY', 'phy_device': 'xcku3p-ffvb676-2-e'}\n")
            filename = f.name
        try:
            cfg = load_config(filename)
        finally:
            os.unlink(filename)
        self.assertEqual(cfg["init_mode"], "firmware")

    # Host-memory layout ---------------------------------------------------------------------------

    def test_example_prp_list_clear_of_staging(self):
        for init_mode in ["firmware", "rtl"]:
            cfg    = example_config(init_mode=init_mode)
            offset = prp_list_offset(cfg)
            end    = offset + cfg["qsize"]*0x1000
            self.assertGreaterEqual(offset, 0x7000)
            self.assertLessEqual(end, cfg["staging_base"])

    def test_prp_list_default_placement(self):
        # Without prp_list_offset, the legacy place (0x10000 + qsize*4 KiB = 0x18000) is inside
        # the example's staging region: the generator moves it after the queues.
        cfg = example_config()
        del cfg["prp_list_offset"]
        validate_config(cfg)
        self.assertEqual(prp_list_offset(cfg), 0x7000)
        # Default config (request generator, 64 entries, 512 KiB): legacy place is outside the window.
        cfg = _normalize_config(dict(BASE_CONFIG))
        validate_config(cfg)
        self.assertLessEqual(prp_list_offset(cfg) + 64*0x1000, 0x8_0000)

    def test_prp_list_overlapping_staging_rejected(self):
        cfg = example_config(prp_list_offset=0x18000)
        with self.assertRaisesRegex(ValueError, "PRP-list region"):
            validate_config(cfg)

    def test_prp_list_outside_window_rejected(self):
        cfg = example_config(prp_list_offset=0x3c000)
        with self.assertRaisesRegex(ValueError, "PRP-list region"):
            validate_config(cfg)

    def test_qsize_range(self):
        for qsize in [1, 128]:
            cfg = _normalize_config(dict(BASE_CONFIG, qsize=qsize, qd=1))
            with self.assertRaisesRegex(ValueError, "qsize"):
                validate_config(cfg)

    def test_firmware_mode_needs_qid1_and_hostmem_csr(self):
        with self.assertRaisesRegex(ValueError, "qid"):
            validate_config(example_config(qid=2))
        with self.assertRaisesRegex(ValueError, "with_hostmem_csr"):
            validate_config(example_config(with_hostmem_csr=False))

    # Generated core (elaboration) -----------------------------------------------------------------

    def test_firmware_mode_core_exposes_firmware_layout(self):
        cfg = example_config()
        soc = elaborate(cfg)
        # Host-memory CSR present (the firmware writes SQEs and reads CQEs through it).
        names = [csr.name for csr in soc.csr.regions["nvme"].obj]
        self.assertIn("hostmem_csr_csr_adr", names)
        self.assertIn("hostmem_csr_csr_rdata", names)
        # Layout constants for the firmware (generated/soc.h).
        self.assertEqual(soc.constants["LITENVME_HOSTMEM_BASE"],  cfg["hostmem_base"])
        self.assertEqual(soc.constants["LITENVME_IO_Q_ENTRIES"],  cfg["qsize"])
        self.assertEqual(soc.constants["LITENVME_PRP_LIST_BASE"], cfg["hostmem_base"] + 0x7000)

    @unittest.skipUnless(riscv_toolchain() and shutil.which("make"), "needs a RISC-V toolchain")
    def test_example_minimal_firmware_builds(self):
        # Two-pass build of the example (firmware "minimal"): the firmware must compile and link
        # against the generated core's CSR names and be baked into the ROM.
        with tempfile.TemporaryDirectory() as d:
            subprocess.check_call(["python3", "-m", "litenvme.gen", EXAMPLE, "--output-dir", d],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rom = os.path.join(d, "gateware", "litenvme_core_rom.init")
            with open(rom) as f:
                words = [l for l in f.read().split() if int(l, 16) != 0]
            self.assertGreater(len(words), 0, "empty ROM: firmware not baked")


if __name__ == "__main__":
    unittest.main()
