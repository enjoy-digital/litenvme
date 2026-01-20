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

class LitePCIeCFGMaster(LiteXModule, AutoCSR):
    def __init__(self, cfg_sink, cmp_source, requester_id=0x0000, tag=0x42, timeout=2**20, with_csr=False):
        # User Interface ---------------------------------------------------------------------------
        self.start    = Signal()
        self.bus      = Signal(8)
        self.device   = Signal(5)
        self.function = Signal(3)
        self.reg      = Signal(6)  # DWORD index (offset/4).
        self.ext_reg  = Signal(3)  # Extended register (per your layout, 3-bit).

        self.done     = Signal()
        self.err      = Signal()
        self.rdata    = Signal(32)

        # CSRs -------------------------------------------------------------------------------------
        if with_csr:
            self.add_csr()

        # Internal ---------------------------------------------------------------------------------
        timer = Signal(max=timeout)

        # Completion match (LitePCIe completion stream layout).
        cpl_match = Signal()
        self.comb += [
            cpl_match.eq(cmp_source.valid &
                         (cmp_source.tag    == tag) &
                         (cmp_source.req_id == requester_id)),
        ]

        # Defaults.
        self.comb += [
            cfg_sink.valid.eq(0),
            cfg_sink.last.eq(1),
            cmp_source.ready.eq(0),
            self.done.eq(0),
        ]

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(self.err,  0),
            NextValue(timer,     0),
            If(self.start,
                NextState("SEND")
            )
        )
        fsm.act("SEND",
            cfg_sink.valid.eq(1),

            # High-level CFG request fields (packetizer will build TLP header).
            cfg_sink.req_id.eq(requester_id),
            cfg_sink.we.eq(0),  # Read.
            cfg_sink.bus_number.eq(self.bus),
            cfg_sink.device_no.eq(self.device),
            cfg_sink.func.eq(self.function),
            cfg_sink.ext_reg.eq(self.ext_reg),
            cfg_sink.register_no.eq(self.reg),
            cfg_sink.tag.eq(tag),

            # No data for reads.
            cfg_sink.dat.eq(0),

            # Route: keep 0 for now (single-user).
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
                If(cmp_source.err,
                    NextValue(self.err, 1)
                ),
                NextState("DONE")
            ).Elif(timer == (timeout - 1),
                NextValue(self.err, 1),
                NextState("DONE")
            )
        )
        fsm.act("DONE",
            self.done.eq(1),
            If(~self.start,
                NextState("IDLE")
            )
        )

        # CSR Wiring -------------------------------------------------------------------------------
        if with_csr:
            self.comb += [
                self.start.eq(self._cfg_ctrl.fields.start),
                self.bus.eq(self._cfg_bdf.fields.bus),
                self.device.eq(self._cfg_bdf.fields.dev),
                self.function.eq(self._cfg_bdf.fields.fn),
                self.reg.eq(self._cfg_bdf.fields.reg),
                self.ext_reg.eq(self._cfg_bdf.fields.ext),

                self._cfg_stat.fields.done.eq(self.done),
                self._cfg_stat.fields.err.eq(self.err),
                self._cfg_rdata.status.eq(self.rdata),
            ]

    def add_csr(self):
        self._cfg_ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0),
        ])
        self._cfg_bdf = CSRStorage(fields=[
            CSRField("bus", size=8,  offset=0),
            CSRField("dev", size=5,  offset=8),
            CSRField("fn",  size=3,  offset=13),
            CSRField("reg", size=6,  offset=16),  # DWORD index.
            CSRField("ext", size=3,  offset=22),  # Extended reg.
        ])
        self._cfg_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0),
            CSRField("err",  size=1, offset=1),
        ])
        self._cfg_rdata = CSRStatus(32)

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

            # Set manual locations to avoid Vivado to remap lanes.
            platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y3 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[0].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y2 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[1].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y1 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[2].GTHE4_CHANNEL_PRIM_INST}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTHE4_CHANNEL_X0Y0 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gthe4_channel_gen.gen_gthe4_channel_inst[3].GTHE4_CHANNEL_PRIM_INST}}]")

            # RstN Generation.
            from litex.gen.genlib.misc import WaitTimer
            self.rst_n_timer       = WaitTimer(2      * sys_clk_freq) # 2s   period.
            self.rst_n_timer_pulse = WaitTimer(100e-3 * sys_clk_freq) # 10ms pulse.
            self.comb += [
                self.rst_n_timer.wait.eq(       ~self.rst_n_timer.done),
                self.rst_n_timer_pulse.wait.eq( ~self.rst_n_timer.done),
                pcie_pads_rst_n.eq(self.rst_n_timer_pulse.done),
                pcie_pads.rst_n.eq(self.rst_n_timer_pulse.done),
            ]
            self.comb += [
                platform.request("user_led", 1).eq(self.rst_n_timer_pulse.done),
                platform.request("user_led", 2).eq(self.pcie_phy._link_status.fields.status),
                platform.request("user_led", 3).eq(self.pcie_phy._link_status.fields.phy_down),
            ]

    # LiteScope Probes (Debug) -----------------------------------------------------------------

    # PCIe.
    def add_pcie_probe(self):
        self.pcie_phy.add_ltssm_tracer()
        analyzer_signals = [
            # Rst.
            self.pcie_phy.pcie_rst_n,

            # Link Status.
            self.pcie_phy._link_status.fields.rate,
            self.pcie_phy._link_status.fields.width,
            self.pcie_phy._link_status.fields.ltssm
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 4096,
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
