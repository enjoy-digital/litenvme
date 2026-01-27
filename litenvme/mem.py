#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# LiteNVMe PCIe MMIO Accessor ----------------------------------------------------------------------

class LiteNVMePCIeMmioAccessor(LiteXModule):
    """PCIe MEM Read/Write accessor (MMIO-over-REQUEST).

    Small helper to issue Memory reads/writes through the LitePCIe crossbar master port,
    reusing the shared REQUEST layout.

    Notes:
    - Only a single outstanding transaction is supported (one completion expected).
    - For now, only full DWORD writes are supported (wsel=0xf).
    """
    def __init__(self, port, tag=0x44, timeout=2**20, with_csr=False):
        self.port = port

        # User Interface.
        self.start = Signal()
        self.we    = Signal()     # 0: Read / 1: Write.
        self.adr   = Signal(64)   # Byte address.
        self.wdata = Signal(32)   # Write data (DWORD).
        self.wsel  = Signal(4)    # Byte enable (only 0xf supported).
        self.len   = Signal(10)   # DWORD count (start with 1).

        self.done  = Signal()
        self.err   = Signal()
        self.rdata = Signal(32)

        # # #

        req_sink   = port.source
        cmp_source = port.sink

        # Timer (simple timeout watchdog).
        timer = Signal(max=timeout)
        self.timer_dbg = Signal(16)
        self.comb += self.timer_dbg.eq(timer[:16])

        # Start Pulse.
        start_d     = Signal()
        start_pulse = Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

        # Completion match (single outstanding transaction).
        cpl_match = Signal()
        self.comb += cpl_match.eq(cmp_source.valid)

        # Sticky Status (cleared on new start).
        done_r = Signal()
        err_r  = Signal()
        self.comb += [
            self.done.eq(done_r),
            self.err.eq(err_r),
        ]
        self.sync += If(start_pulse,
            done_r.eq(0),
            err_r.eq(0),
        )

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(timer, 0),
            If(start_pulse,
                If(self.we & (self.wsel != 0xf),
                    NextState("BAD-WSEL"),
                ).Else(
                    NextState("SEND"),
                )
            )
        )
        fsm.act("BAD-WSEL",
            # Unsupported partial write for now.
            NextValue(done_r, 1),
            NextValue(err_r,  1),
            NextState("IDLE"),
        )
        fsm.act("SEND",
            req_sink.valid.eq(1),
            req_sink.first.eq(1),
            req_sink.last.eq(1),

            # Shared REQUEST fields.
            req_sink.we.eq(self.we),
            req_sink.adr.eq(self.adr),
            req_sink.len.eq(self.len),
            req_sink.tag.eq(tag),

            # Data (writes); reads ignore it.
            req_sink.dat.eq(Replicate(self.wdata, len(req_sink.dat)//32)),

            # Route.
            req_sink.channel.eq(port.channel),

            If(req_sink.ready,
                NextState("WAIT"),
            )
        )
        fsm.act("WAIT",
            cmp_source.ready.eq(1),
            NextValue(timer, timer + 1),

            If(cpl_match,
                NextValue(self.rdata, cmp_source.dat[:32]),
                NextState("LATCH"),
            ).Elif(timer == (timeout - 1),
                NextState("TIMEOUT"),
            )
        )
        fsm.act("LATCH",
            NextValue(done_r, 1),
            If(cmp_source.err,
                NextValue(err_r, 1),
            ),
            NextState("IDLE"),
        )
        fsm.act("TIMEOUT",
            NextValue(done_r, 1),
            NextValue(err_r,  1),
            NextState("IDLE"),
        )

        # CSRs.
        if with_csr:
            self.add_csr()

    def add_csr(self):
        # mem_ctrl:
        # - start: Pulse to launch a new MEM transaction.
        # - we:    0=Read, 1=Write.
        # - wsel:  Byte enable (only 0xf supported for now).
        # - len:   DWORD count (use 1 for single access).
        self._mem_ctrl = CSRStorage(fields=[
            CSRField("start", size=1,  offset=0, pulse=True, values=[
                ("``1``", "Start a new MEM transaction."),
            ]),
            CSRField("we",    size=1,  offset=1, values=[
                ("``0``", "MEM Read."),
                ("``1``", "MEM Write."),
            ]),
            CSRField("wsel",  size=4,  offset=4, values=[
                ("``0xf``", "Full DWORD write (only supported mode)."),
            ]),
            CSRField("len",   size=10, offset=8, description="DWORD count (start with 1)."),
        ])

        # mem_adr_l/h:
        # - 64-bit byte address split into low/high DWORDs.
        self._mem_adr_l = CSRStorage(32, description="MEM address low (bytes).")
        self._mem_adr_h = CSRStorage(32, description="MEM address high (bytes).")

        # mem_wdata:
        # - DWORD write payload for MEM writes.
        self._mem_wdata = CSRStorage(32, description="MEM write data (DWORD).")

        # mem_stat:
        # - done: Set when the transaction completed (or timed out).
        # - err:  Completion error or timeout / invalid wsel.
        self._mem_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="Transaction done."),
            CSRField("err",  size=1, offset=1, description="Transaction error (completion error/timeout/invalid wsel)."),
        ])

        # mem_rdata:
        # - DWORD read payload for MEM reads (valid when done=1 and err=0).
        self._mem_rdata = CSRStatus(32, description="MEM read data (DWORD).")

        # Connections.
        self.comb += [
            # Control.
            self.start.eq(self._mem_ctrl.fields.start),
            self.we.eq(self._mem_ctrl.fields.we),
            self.wsel.eq(self._mem_ctrl.fields.wsel),
            self.len.eq(self._mem_ctrl.fields.len),

            # Addressing / Data.
            self.adr.eq(Cat(self._mem_adr_l.storage, self._mem_adr_h.storage)),
            self.wdata.eq(self._mem_wdata.storage),

            # Status / Readback.
            self._mem_stat.fields.done.eq(self.done),
            self._mem_stat.fields.err.eq(self.err),
            self._mem_rdata.status.eq(self.rdata),
        ]
