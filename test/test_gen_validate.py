#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litenvme.gen import _normalize_config, validate_config

BASE_CONFIG = {
    "phy"        : "USPPCIEPHY",
    "phy_device" : "xcku3p-ffvb676-2-e",
}

# Test Generator Validation ------------------------------------------------------------------------

class TestGeneratorValidation(unittest.TestCase):
    def test_supported_data_widths(self):
        for data_width in [64, 128, 256]:
            validate_config(_normalize_config(dict(BASE_CONFIG, data_width=data_width)))

    def test_512_bit_rejected(self):
        # The host-memory responder asserts data_width in (64, 128, 256): reject 512 at config time.
        cfg = _normalize_config(dict(BASE_CONFIG, data_width=512))
        with self.assertRaisesRegex(ValueError, "data_width"):
            validate_config(cfg)


if __name__ == "__main__":
    unittest.main()
