#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Ensure this directory is importable so tests can share `models.py` regardless of how pytest
# is invoked.

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
