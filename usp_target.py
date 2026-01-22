#!/usr/bin/env python3

import os

from migen import *
from litex.gen import *

from usp_platform import Platform

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock import USMMCM
from litex.soc.cores.led   import LedChaser

from litepcie.phy.usppciephy import USPPCIEPHY

from litescope import LiteScopeAnalyzer

# LitePCIe CFG Master ------------------------------------------------------------------------------

class LitePCIeCFGMaster(LiteXModule):
    def __init__(self, cfg_sink, cmp_source, requester_id=0x0000, tag=0x42, timeout=2**20, with_csr=False):
        # User Interface ---------------------------------------------------------------------------
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

        # CSRs -------------------------------------------------------------------------------------
        if with_csr:
            self.add_csr()

        # Internal ---------------------------------------------------------------------------------
        timer = Signal(max=timeout)
        self.timer_dbg = Signal(16)
        self.comb += self.timer_dbg.eq(timer[:16])

        # Completion match (LitePCIe completion stream layout).
        self.cpl_match = cpl_match = Signal()
        self.comb += [
            cpl_match.eq(cmp_source.valid &
                         (cmp_source.tag    == tag) &
                         (cmp_source.req_id == requester_id)),
        ]

        # Start edge (launch once per pulse) -------------------------------------------------------
        start_d     = Signal()
        start_pulse = Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

        # Sticky status ----------------------------------------------------------------------------
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

        # Defaults.
        self.comb += [
            cfg_sink.valid.eq(0),
            cfg_sink.first.eq(0),
            cfg_sink.last.eq(0),
            cmp_source.ready.eq(1),
        ]

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(timer, 0),
            If(start_pulse,
                NextState("SEND")
            )
        )
        fsm.act("SEND",
            cfg_sink.valid.eq(1),
            cfg_sink.first.eq(1),
            cfg_sink.last.eq(1),

            # High-level CFG request fields (packetizer will build TLP header).
            cfg_sink.req_id.eq(requester_id),
            cfg_sink.we.eq(self.we),
            cfg_sink.bus_number.eq(self.bus),
            cfg_sink.device_no.eq(self.device),
            cfg_sink.func.eq(self.function),
            cfg_sink.ext_reg.eq(self.ext_reg),
            cfg_sink.register_no.eq(self.reg),
            cfg_sink.tag.eq(tag),

            # Data (used for writes, ignored for reads).
            cfg_sink.dat.eq(Replicate(self.wdata, len(cfg_sink.dat)//32)),

            cfg_sink.channel.eq(0),

            If(cfg_sink.ready,
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

        # CSR Wiring -------------------------------------------------------------------------------
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
            CSRField("we",    size=1, offset=1),  # 0: Read / 1: Write.
        ])
        self._cfg_bdf = CSRStorage(fields=[
            CSRField("bus", size=8,  offset=0),
            CSRField("dev", size=5,  offset=8),
            CSRField("fn",  size=3,  offset=13),
            CSRField("reg", size=6,  offset=16),  # DWORD index.
            CSRField("ext", size=3,  offset=22),  # Extended reg.
        ])
        self._cfg_wdata = CSRStorage(32)
        self._cfg_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0),
            CSRField("err",  size=1, offset=1),
        ])
        self._cfg_rdata = CSRStatus(32)

# LitePCIe MEM Master ------------------------------------------------------------------------------

class LitePCIeMEMMaster(LiteXModule):
    def __init__(self, port, tag=0x44, timeout=2**20, with_csr=False):
        req_sink   = port.source
        cmp_source = port.sink

        # User Interface ---------------------------------------------------------------------------
        self.start  = Signal()
        self.we     = Signal()     # 0: Read / 1: Write.
        self.adr    = Signal(64)   # Byte address.
        self.wdata  = Signal(32)
        self.wsel   = Signal(4)    # Byte enable for writes (currently only 0xf supported).
        self.len    = Signal(10)   # DWORD count (use 1 for now).

        self.done   = Signal()
        self.err    = Signal()
        self.rdata  = Signal(32)

        # CSRs -------------------------------------------------------------------------------------
        if with_csr:
            self.add_csr()

        # Internal ---------------------------------------------------------------------------------
        timer = Signal(max=timeout)
        self.timer_dbg = Signal(16)
        self.comb += self.timer_dbg.eq(timer[:16])

        # Start edge -------------------------------------------------------------------------------
        start_d     = Signal()
        start_pulse = Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

        # Sticky status ----------------------------------------------------------------------------
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

        # Completion match -------------------------------------------------------------------------
        self.cpl_match = cpl_match = Signal()
        #self.comb += cpl_match.eq(cmp_source.valid & (cmp_source.tag == tag))
        self.comb += cpl_match.eq(cmp_source.valid)

        # Defaults ---------------------------------------------------------------------------------
        self.comb += [
            req_sink.valid.eq(0),
            req_sink.first.eq(0),
            req_sink.last.eq(0),
            cmp_source.ready.eq(1),
        ]

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(timer, 0),
            If(start_pulse,
                # For now, only support full DWORD writes.
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

            # Request fields (crossbar request layout).
            req_sink.we.eq(self.we),
            req_sink.adr.eq(self.adr),
            req_sink.len.eq(self.len),   # DWORDs.
            req_sink.tag.eq(tag),

            # Data (writes); reads ignore it.
            req_sink.dat.eq(Replicate(self.wdata, len(req_sink.dat)//32)),

            # Route.
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

        # CSR Wiring -------------------------------------------------------------------------------
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
            CSRField("we",    size=1,  offset=1),   # 0: Read / 1: Write.
            CSRField("wsel",  size=4,  offset=4),   # Byte enable.
            CSRField("len",   size=10, offset=8),   # DWORD count.
        ])
        self._mem_adr_l = CSRStorage(32)
        self._mem_adr_h = CSRStorage(32)
        self._mem_wdata = CSRStorage(32)
        self._mem_stat  = CSRStatus(fields=[
            CSRField("done", size=1, offset=0),
            CSRField("err",  size=1, offset=1),
        ])
        self._mem_rdata = CSRStatus(32)


# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # Clk.
        clk200 = platform.request("clk200")

        # PLL.
        self.pll = pll = USMMCM(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=False)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sys_clk_freq=int(125e6), with_led_chaser=True, **kwargs):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteNVME Test SoC.", ident_version=True)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # PCIe -------------------------------------------------------------------------------------
        with_pcie = True
        if with_pcie:
            pcie_pads = platform.request("pcie_x4")
            pcie_pads_rst_n = pcie_pads.rst_n
            pcie_pads.rst_n = Signal()
            self.pcie_phy = USPPCIEPHY(platform, pcie_pads,
                speed      = "gen3",
                data_width = 128,
                ip_name    = "pcie4c_uscale_plus",
                bar0_size  = 0x20000,
                mode       = "RootPort",
            )
            self.add_pcie(phy=self.pcie_phy)

            # Set manual locations to avoid Vivado to remap lanes.
            platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y3 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[0].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y2 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[1].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y1 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[2].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y0 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[3].GTHE4_CHANNEL_PRIM_INST}}]")

            # OneShot Reset.
            from litex.gen.genlib.misc import WaitTimer
            self.pcie_rst_timer = WaitTimer(int(100e-3 * sys_clk_freq))  # 100ms.
            self.comb += self.pcie_rst_timer.wait.eq(1)
            self.comb += [
                pcie_pads_rst_n.eq(self.pcie_rst_timer.done),
                pcie_pads.rst_n.eq(self.pcie_rst_timer.done),
            ]

            self.comb += [
                #platform.request("user_led", 1).eq(self.rst_n_timer_pulse.done),
                platform.request("user_led", 2).eq(self.pcie_phy._link_status.fields.status),
                platform.request("user_led", 3).eq(self.pcie_phy._link_status.fields.phy_down),
            ]


            # CFG Master ----------------------------------------------------------------------------
            endpoint   = self.pcie_endpoint
            cfg_sink   = endpoint.req_packetizer.cfg_sink
            cmp_source = endpoint.cmp_depacketizer.cmp_source

            requester_id = 0x0000
            if hasattr(self.pcie_phy, "requester_id"):
                requester_id = self.pcie_phy.requester_id

            self.cfgm = LitePCIeCFGMaster(
                cfg_sink     = cfg_sink,
                cmp_source   = cmp_source,
                requester_id = requester_id,
                tag          = 0x42,
                with_csr     = True,
            )
            self.add_module(name="pcie_cfgm", module=self.cfgm)

            # MEM Master ----------------------------------------------------------------------------
            self.mem_port = mem_port = endpoint.crossbar.get_master_port()
            self.mem = LitePCIeMEMMaster(
                port     = mem_port,
                tag      = 0x0,
                #tag      = 0x44,
                with_csr = True,
            )
            self.add_module(name="pcie_mem", module=self.mem)


    # LiteScope Probes (Debug) -----------------------------------------------------------------

    # PCIe.
    def add_pcie_probe(self):
        #self.pcie_phy.add_ltssm_tracer()

        endpoint   = self.pcie_endpoint
        cfg_sink   = endpoint.req_packetizer.cfg_sink
        cmp_source = endpoint.cmp_depacketizer.cmp_source

        cmp_source_dat = Signal(32)
        self.comb += cmp_source_dat.eq(cmp_source.dat[:32])

        analyzer_signals = [
            # Reset / Link ------------------------------------------------------------------------
            self.pcie_phy.pcie_rst_n,
            self.pcie_phy._link_status.fields.status,
            self.pcie_phy._link_status.fields.phy_down,
            self.pcie_phy._link_status.fields.rate,
            self.pcie_phy._link_status.fields.width,
            self.pcie_phy._link_status.fields.ltssm,
            self.pcie_phy.cmp_source.valid,
            self.pcie_phy.cmp_source.ready,
            self.pcie_phy.cmp_source.last,
            self.pcie_phy.req_sink.valid,
            self.pcie_phy.req_sink.ready,
            self.pcie_phy.req_sink.last,

            # Endpoint.
            self.pcie_endpoint.req_packetizer.tlp_raw.valid,
            self.pcie_endpoint.req_packetizer.tlp_raw.ready,
            self.pcie_endpoint.req_packetizer.header_inserter.sink.valid,
            self.pcie_endpoint.req_packetizer.header_inserter.sink.ready,
            self.pcie_endpoint.req_packetizer.header_inserter.sink.first,
            self.pcie_endpoint.req_packetizer.header_inserter.sink.last,
            self.pcie_endpoint.req_packetizer.header_inserter.source.valid,
            self.pcie_endpoint.req_packetizer.header_inserter.source.ready,
            self.pcie_endpoint.req_packetizer.header_inserter.source.first,
            self.pcie_endpoint.req_packetizer.header_inserter.source.last,

            # CFG Master --------------------------------------------------------------------------
            self.cfgm.start,
            self.cfgm.done,
            self.cfgm.err,
            self.cfgm.fsm,
            self.cfgm.cpl_match,
            self.cfgm.timer_dbg,
            self.cfgm.bus,
            self.cfgm.device,
            self.cfgm.function,
            self.cfgm.reg,
            self.cfgm.ext_reg,

#            # CFG Request Stream (cfg_sink) -------------------------------------------------------
#            cfg_sink.valid,
#            cfg_sink.ready,
#            cfg_sink.last,
#            cfg_sink.req_id,
#            cfg_sink.we,
#            cfg_sink.bus_number,
#            cfg_sink.device_no,
#            cfg_sink.func,
#            cfg_sink.ext_reg,
#            cfg_sink.register_no,
#            cfg_sink.tag,
#
#            # Completion Stream (cmp_source) ------------------------------------------------------
#            cmp_source.valid,
#            cmp_source.ready,
#            cmp_source.first,
#            cmp_source.last,
#            cmp_source.err,
#            cmp_source.tag,
#            cmp_source.req_id,
#            cmp_source.adr,
#            cmp_source.len,
#            cmp_source.end,
#            #cmp_source.dat[:32],
#            cmp_source_dat,

            # MEM.
            self.mem.fsm,
            self.mem.cpl_match,

            # MEM Request Stream (crossbar master port) -------------------------------------------
            self.mem_port.source.valid,
            self.mem_port.source.ready,
            self.mem_port.source.first,
            self.mem_port.source.last,
            self.mem_port.source.we,
            self.mem_port.source.adr,
            self.mem_port.source.len,
            self.mem_port.source.tag,
            #self.mem_port.source.be,

            # MEM Completion Stream ---------------------------------------------------------------
            self.mem_port.sink.valid,
            self.mem_port.sink.ready,
            self.mem_port.sink.first,
            self.mem_port.sink.last,
            self.mem_port.sink.err,
            self.mem_port.sink.tag,
            self.mem_port.sink.req_id,
            self.mem_port.sink.adr,
            self.mem_port.sink.len,
            self.mem_port.sink.end,
        ]

        dep = self.pcie_endpoint.cmp_depacketizer

        analyzer_signals += [
            # Depacketizer input (from PHY -> depacketizer).
            dep.sink.valid,
            dep.sink.ready,
            dep.sink.last,

            # Header extracter I/O.
            dep.header_extracter.sink.valid,
            dep.header_extracter.sink.ready,
            dep.header_extracter.sink.last,
            dep.header_extracter.source.valid,
            dep.header_extracter.source.ready,
            dep.header_extracter.source.first,
            dep.header_extracter.source.last,

            # Dispatch sink (common decoded stream).
            dep.dispatch_sink.valid,
            dep.dispatch_sink.ready,
            dep.dispatch_sink.first,
            dep.dispatch_sink.last,
            dep.dispatch_sink.fmt,
            dep.dispatch_sink.type,

            # Dispatcher select (which output we’re routing to).
            dep.dispatcher.sel,

            # Completion dispatch source (before tlp_cmp).
            dep.dispatch_sources["COMPLETION"].valid,
            dep.dispatch_sources["COMPLETION"].ready,
            dep.dispatch_sources["COMPLETION"].first,
            dep.dispatch_sources["COMPLETION"].last,

            # tlp_cmp endpoint (after completion header decode).
            dep.tlp_cmp.valid,
            dep.tlp_cmp.ready,
            dep.tlp_cmp.first,
            dep.tlp_cmp.last,
            dep.tlp_cmp.tag,

            # Final completion stream exposed by depacketizer.
            dep.cmp_source.valid,
            dep.cmp_source.ready,
            dep.cmp_source.first,
            dep.cmp_source.last,
            dep.cmp_source.tag,
        ]

        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 2048,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="LiteNVME Test SoC.")
    parser.add_target_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    args = parser.parse_args()

    soc = BaseSoC(sys_clk_freq=args.sys_clk_freq, **parser.soc_argdict)
    soc.add_pcie_probe()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
