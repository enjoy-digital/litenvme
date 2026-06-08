# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import tempfile
import unittest

from litenvme.gen import _normalize_config, load_config, validate_config


BASE_CONFIG = {
    "phy"        : "USPPCIEPHY",
    "phy_device" : "xcku3p-ffvb676-2-e",
}


class TestGeneratorConfig(unittest.TestCase):
    def test_example_config_loads(self):
        cfg = load_config(os.path.join("examples", "alibaba_xcku3p.yml"))
        self.assertEqual(cfg["init_mode"], "firmware")
        self.assertEqual(cfg["firmware"], "none")
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
        cfg = dict(BASE_CONFIG, init_mode="rtl", with_block_streamer=True)
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


if __name__ == "__main__":
    unittest.main()
