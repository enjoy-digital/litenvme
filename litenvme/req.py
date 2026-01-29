#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect.csr import *

# LiteNVMe Request CSR ---------------------------------------------------------------------------

class LiteNVMeRequestCSR(Module):
    """CSR request interface for firmware-driven NVMe operations."""
    def __init__(self):
        # Request parameters.
        self._req_op     = CSRStorage(8,  description="Request opcode (0=read, 1=write, 2=identify).")
        self._req_nsid   = CSRStorage(32, description="NSID.")
        self._req_lba_lo = CSRStorage(32, description="LBA low.")
        self._req_lba_hi = CSRStorage(32, description="LBA high.")
        self._req_nlb    = CSRStorage(32, description="NLB (number of blocks).")
        self._req_buf_lo = CSRStorage(32, description="PRP1 buffer address low.")
        self._req_buf_hi = CSRStorage(32, description="PRP1 buffer address high.")
        self._req_bar0_lo = CSRStorage(32, description="BAR0 base low.")
        self._req_bar0_hi = CSRStorage(32, description="BAR0 base high.")

        # Control.
        self._req_ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, description="Start request (set to 1, firmware clears)."),
        ])

        # Status (written by firmware).
        self._req_status = CSRStorage(fields=[
            CSRField("busy",  size=1, offset=0, description="Request in progress (firmware-owned)."),
            CSRField("done",  size=1, offset=1, description="Request complete (firmware-owned)."),
            CSRField("error", size=1, offset=2, description="Request error (firmware-owned)."),
        ])
        self._req_cqe_status = CSRStorage(32, description="CQE DW3 status (firmware-owned).")
