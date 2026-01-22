from migen import *
from litex.gen import *

from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField


class LiteNVMePCIeMmioAccessor(LiteXModule):
    """Issue PCIe MEM reads/writes through a crossbar master port."""
    def __init__(self, port, tag=0x44, timeout=2**20, with_csr=False):
        req_sink   = port.source
        cmp_source = port.sink

        # User Interface -------------------------------------------------------------------------
        self.start  = Signal()
        self.we     = Signal()     # 0: Read / 1: Write.
        self.adr    = Signal(64)   # Byte address.
        self.wdata  = Signal(32)
        self.wsel   = Signal(4)    # Byte enable for writes (currently only 0xf supported).
        self.len    = Signal(10)   # DWORD count (use 1 for now).

        self.done   = Signal()
        self.err    = Signal()
        self.rdata  = Signal(32)

        # CSRs -----------------------------------------------------------------------------------
        if with_csr:
            self.add_csr()

        # Internal -------------------------------------------------------------------------------
        timer = Signal(max=timeout)
        self.timer_dbg = Signal(16)
        self.comb += self.timer_dbg.eq(timer[:16])

        # Start edge -----------------------------------------------------------------------------
        start_d     = Signal()
        start_pulse = Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

        # Sticky status --------------------------------------------------------------------------
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

        # Completion match -----------------------------------------------------------------------
        self.cpl_match = cpl_match = Signal()
        self.comb += cpl_match.eq(cmp_source.valid)

        # Defaults -------------------------------------------------------------------------------
        self.comb += [
            req_sink.valid.eq(0),
            req_sink.first.eq(0),
            req_sink.last.eq(0),
            cmp_source.ready.eq(0),
        ]

        # FSM ------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(timer, 0),
            If(start_pulse,
                If(self.we & (self.wsel != 0xf),
                    NextState("BAD-WSEL")
                ).Else(
                    NextState("SEND")
                )
            )
        )

        fsm.act("BAD-WSEL",
            NextValue(done_r, 1),
            NextValue(err_r,  1),
            NextState("IDLE")
        )

        fsm.act("SEND",
            req_sink.valid.eq(1),
            req_sink.first.eq(1),
            req_sink.last.eq(1),

            req_sink.we.eq(self.we),
            req_sink.adr.eq(self.adr),
            req_sink.len.eq(self.len),
            req_sink.tag.eq(tag),

            req_sink.dat.eq(Replicate(self.wdata, len(req_sink.dat)//32)),

            req_sink.channel.eq(port.channel),

            If(req_sink.ready,
                NextState("WAIT")
            )
        )

        fsm.act("WAIT",
            cmp_source.ready.eq(1),
            NextValue(timer, timer + 1),

            If(cpl_match,
                NextValue(self.rdata, cmp_source.dat[:32]),
                NextState("LATCH")
            ).Elif(timer == (timeout - 1),
                NextState("TIMEOUT")
            )
        )

        fsm.act("LATCH",
            NextValue(done_r, 1),
            If(cmp_source.err,
                NextValue(err_r, 1)
            ),
            NextState("IDLE")
        )

        fsm.act("TIMEOUT",
            NextValue(done_r, 1),
            NextValue(err_r,  1),
            NextState("IDLE")
        )

        # CSR wiring ------------------------------------------------------------------------------
        if with_csr:
            self.comb += [
                self.start.eq(self._mem_ctrl.fields.start),
                self.we.eq(self._mem_ctrl.fields.we),

                self.adr.eq(Cat(self._mem_adr_l.storage, self._mem_adr_h.storage)),
                self.wdata.eq(self._mem_wdata.storage),
                self.wsel.eq(self._mem_ctrl.fields.wsel),
                self.len.eq(self._mem_ctrl.fields.len),

                self._mem_stat.fields.done.eq(self.done),
                self._mem_stat.fields.err.eq(self.err),
                self._mem_rdata.status.eq(self.rdata),
            ]

    def add_csr(self):
        self._mem_ctrl = CSRStorage(fields=[
            CSRField("start", size=1,  offset=0),
            CSRField("we",    size=1,  offset=1),
            CSRField("wsel",  size=4,  offset=4),
            CSRField("len",   size=10, offset=8),
        ])
        self._mem_adr_l = CSRStorage(32)
        self._mem_adr_h = CSRStorage(32)
        self._mem_wdata = CSRStorage(32)
        self._mem_stat  = CSRStatus(fields=[
            CSRField("done", size=1, offset=0),
            CSRField("err",  size=1, offset=1),
        ])
        self._mem_rdata = CSRStatus(32)
