from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField


class LiteNVMePCIeCfgAccessor(LiteXModule):
    """Issue PCIe CFG0 Reads/Writes using the shared REQUEST layout (CFG-over-REQUEST).

    Expects the crossbar master port REQUEST layout to include CONFIGURATION extensions:
      - is_cfg, bus_number, device_no, func, ext_reg, register_no
    """
    def __init__(self, port, requester_id=0x0000, tag=0x42, timeout=2**20, with_csr=False):
        req_sink   = port.source
        cmp_source = port.sink

        # User Interface -------------------------------------------------------------------------
        self.start    = Signal()
        self.we       = Signal()     # 0: Read / 1: Write.
        self.wdata    = Signal(32)   # Write data (DWORD).
        self.bus      = Signal(8)
        self.device   = Signal(5)
        self.function = Signal(3)
        self.reg      = Signal(6)    # DWORD index (offset/4).
        self.ext_reg  = Signal(3)    # Extended register.

        self.done     = Signal()
        self.err      = Signal()
        self.rdata    = Signal(32)

        # CSRs -----------------------------------------------------------------------------------
        if with_csr:
            self.add_csr()

        # Internal -------------------------------------------------------------------------------
        timer = Signal(max=timeout)
        self.timer_dbg = Signal(16)
        self.comb += self.timer_dbg.eq(timer[:16])

        # Completion match -----------------------------------------------------------------------
        self.cpl_match = cpl_match = Signal()
        self.comb += cpl_match.eq(cmp_source.valid)

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
                NextState("SEND")
            )
        )
        fsm.act("SEND",
            req_sink.valid.eq(1),
            req_sink.first.eq(1),
            req_sink.last.eq(1),

            # Shared REQUEST layout fields.
            req_sink.req_id.eq(requester_id),
            req_sink.we.eq(self.we),
            req_sink.adr.eq(0),   # Unused for CFG but must be driven.
            req_sink.len.eq(1),   # CFG is 1DW.
            req_sink.tag.eq(tag),

            # Data (writes); reads ignore it.
            req_sink.dat.eq(Replicate(self.wdata, len(req_sink.dat)//32)),

            # Route.
            req_sink.channel.eq(port.channel),

            # CFG-over-REQUEST fields.
            req_sink.is_cfg.eq(1),
            req_sink.bus_number.eq(self.bus),
            req_sink.device_no.eq(self.device),
            req_sink.func.eq(self.function),
            req_sink.ext_reg.eq(self.ext_reg),
            req_sink.register_no.eq(self.reg),

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
                self.start.eq(self._cfg_ctrl.fields.start),
                self.we.eq(self._cfg_ctrl.fields.we),

                self.bus.eq(self._cfg_bdf.fields.bus),
                self.device.eq(self._cfg_bdf.fields.dev),
                self.function.eq(self._cfg_bdf.fields.fn),
                self.reg.eq(self._cfg_bdf.fields.reg),
                self.ext_reg.eq(self._cfg_bdf.fields.ext),

                self.wdata.eq(self._cfg_wdata.storage),

                self._cfg_stat.fields.done.eq(self.done),
                self._cfg_stat.fields.err.eq(self.err),
                self._cfg_rdata.status.eq(self.rdata),
            ]

    def add_csr(self):
        self._cfg_ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0),
            CSRField("we",    size=1, offset=1),
        ])
        self._cfg_bdf = CSRStorage(fields=[
            CSRField("bus", size=8,  offset=0),
            CSRField("dev", size=5,  offset=8),
            CSRField("fn",  size=3,  offset=13),
            CSRField("reg", size=6,  offset=16),
            CSRField("ext", size=3,  offset=22),
        ])
        self._cfg_wdata = CSRStorage(32)
        self._cfg_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0),
            CSRField("err",  size=1, offset=1),
        ])
        self._cfg_rdata = CSRStatus(32)
