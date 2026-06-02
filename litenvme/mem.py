#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# LiteNVMe PCIe Accessor (base) --------------------------------------------------------------------

class _LiteNVMePCIeAccessor(LiteXModule):
    """Shared single-outstanding accessor over a LitePCIe crossbar master port.

    Issues one request (Config or Memory) and, for reads, waits for the completion. Subclasses
    provide the request-specific fields via ``send_request()`` and their own CSR map via
    ``add_csr()``; everything else -- timer, start-pulse, completion handling, sticky status -- is
    shared here so the (subtle) completion logic lives in a single place.

    Hooks:
    - ``send_request(req_sink)``: return the request-field assignments specific to the accessor.
    - ``posted_writes``: ``True`` if writes complete on acceptance (no completion, e.g. MemWr),
      ``False`` if writes also wait for a completion (e.g. CfgWr).
    - ``bad_wsel_cond()``: optional condition that routes a start to an error state (MMIO wsel check).
    """
    posted_writes = False

    def __init__(self, port, tag, timeout=2**20, with_csr=False):
        self.port = port

        # User Interface (request-specific signals are added by the subclass before super().__init__).
        self.start = Signal()
        self.we    = Signal()     # 0: Read / 1: Write.
        self.wdata = Signal(32)   # Write data (DWORD).
        self.done  = Signal()
        self.err   = Signal()
        self.rdata = Signal(32)

        # # #

        req_sink   = port.source
        cmp_source = port.sink

        # Timer (simple timeout watchdog).
        timer = Signal(max=timeout)

        # Start Pulse.
        start_d     = Signal()
        start_pulse = Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

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

        # Optional error / posted-write routing (subclass-defined).
        bad_wsel  = self.bad_wsel_cond()
        idle_send = NextState("SEND") if bad_wsel is None else \
                    If(bad_wsel, NextState("BAD-WSEL")).Else(NextState("SEND"))
        send_done = If(self.we, NextState("WRITE-LATCH")).Else(NextState("WAIT")) \
                    if self.posted_writes else NextState("WAIT")

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(timer, 0),
            # Drain any stale completion left pending from a prior transaction: the crossbar holds a
            # leftover completion beat valid until consumed, and `cmp_source.ready` is only asserted
            # from WAIT -- so a completion that lands while idle would otherwise be matched by the
            # NEXT read's WAIT, latching the previous transaction's data (off-by-one "stale read",
            # seen at data_width=256). Flushing first makes the first beat seen in WAIT genuinely ours.
            cmp_source.ready.eq(1),
            If(start_pulse, idle_send),
        )
        fsm.act("SEND",
            # Keep draining stale completions until our request is accepted (our own completion
            # cannot arrive before then), so none can slip into WAIT ahead of ours.
            cmp_source.ready.eq(1),
            req_sink.valid.eq(1),
            req_sink.first.eq(1),
            req_sink.last.eq(1),

            # Shared REQUEST fields.
            req_sink.we.eq(self.we),
            req_sink.tag.eq(tag),
            req_sink.dat.eq(Replicate(self.wdata, len(req_sink.dat)//32)),  # writes; reads ignore it.
            req_sink.channel.eq(port.channel),

            # Request-specific fields (address/length, CFG BDF, ...).
            *self.send_request(req_sink),

            If(req_sink.ready, send_done),
        )
        fsm.act("WAIT",
            cmp_source.ready.eq(1),
            NextValue(timer, timer + 1),

            If(cmp_source.valid,
                # Read data lives in the FIRST beat (our reads are single-DWORD, len=1).
                If(cmp_source.first,
                    NextValue(self.rdata, cmp_source.dat[:32]),
                ),
                If(cmp_source.err,
                    NextValue(err_r, 1),
                ),
                # Only retire once the completion's END beat is consumed. The shared
                # LitePCIeTLPController delivers completions strictly in request order and advances
                # its req_queue solely on (valid & last & end & ready). At data_width=128 a 1-DWORD
                # completion is a single beat (first==end); at 256 it spans >1 beat, and returning on
                # the first beat left a trailing END beat unconsumed -> req_queue never advanced ->
                # the NEXT read mis-consumed it (off-by-one "stale read"). Drain through END.
                If(cmp_source.end,
                    NextState("LATCH"),
                ),
            ).Elif(timer == (timeout - 1),
                NextState("TIMEOUT"),
            )
        )
        fsm.act("LATCH",
            NextValue(done_r, 1),
            NextState("IDLE"),
        )
        fsm.act("TIMEOUT",
            NextValue(done_r, 1),
            NextValue(err_r,  1),
            NextState("IDLE"),
        )
        if self.posted_writes:
            fsm.act("WRITE-LATCH",
                NextValue(done_r, 1),
                NextState("IDLE"),
            )
        if bad_wsel is not None:
            fsm.act("BAD-WSEL",
                NextValue(done_r, 1),
                NextValue(err_r,  1),
                NextState("IDLE"),
            )

        # CSRs.
        if with_csr:
            self.add_csr()

    # Hooks (overridden by subclasses) -------------------------------------------------------------

    def send_request(self, req_sink):
        return []

    def bad_wsel_cond(self):
        return None

    def add_csr(self):
        pass

# LiteNVMe PCIe MMIO Accessor ----------------------------------------------------------------------

class LiteNVMePCIeMmioAccessor(_LiteNVMePCIeAccessor):
    """PCIe MEM Read/Write accessor (MMIO-over-REQUEST).

    Issues BAR0 memory reads/writes through a LitePCIe crossbar master port.
    - Single outstanding transaction; reads wait for a completion, writes are posted.
    - Only full DWORD writes are supported (wsel=0xf).
    """
    posted_writes = True

    def __init__(self, port, tag=0x44, timeout=2**20, with_csr=False):
        self.adr  = Signal(64)   # Byte address.
        self.wsel = Signal(4)    # Byte enable (only 0xf supported).
        self.len  = Signal(10)   # DWORD count (start with 1).
        super().__init__(port, tag=tag, timeout=timeout, with_csr=with_csr)

    def send_request(self, req_sink):
        return [
            req_sink.adr.eq(self.adr),
            req_sink.len.eq(self.len),
        ]

    def bad_wsel_cond(self):
        return self.we & (self.wsel != 0xf)

    def add_csr(self):
        # mem_ctrl: start (pulse), we (0=Read/1=Write), wsel (only 0xf), len (DWORD count).
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

        # mem_adr_l/h: 64-bit byte address split into low/high DWORDs.
        self._mem_adr_l = CSRStorage(32, description="MEM address low (bytes).")
        self._mem_adr_h = CSRStorage(32, description="MEM address high (bytes).")

        # mem_wdata: DWORD write payload.
        self._mem_wdata = CSRStorage(32, description="MEM write data (DWORD).")

        # mem_stat: done (completed or timed out), err (completion error/timeout/invalid wsel).
        self._mem_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="Transaction done."),
            CSRField("err",  size=1, offset=1, description="Transaction error (completion error/timeout/invalid wsel)."),
        ])

        # mem_rdata: DWORD read payload (valid when done=1 and err=0).
        self._mem_rdata = CSRStatus(32, description="MEM read data (DWORD).")

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
