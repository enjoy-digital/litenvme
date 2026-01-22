#!/usr/bin/env python3

import os

from migen import *
from litex.gen import *

from usp_platform import Platform

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock import USMMCM

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.core.endpoint  import LitePCIeEndpoint

from litescope import LiteScopeAnalyzer

from gateware.cfg import LiteNVMePCIeCfgAccessor
from gateware.mem import LiteNVMePCIeMmioAccessor


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
    def __init__(self, sys_clk_freq=int(125e6), **kwargs):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteNVME Test SoC.", ident_version=True)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # PCIe -------------------------------------------------------------------------------------
        pcie_pads = platform.request("pcie_x4")
        pcie_pads_rst_n = pcie_pads.rst_n
        pcie_pads.rst_n = Signal()
        self.pcie_phy = USPPCIEPHY(platform, pcie_pads,
            speed      = "gen2",
            data_width = 128,
            ip_name    = "pcie4c_uscale_plus",
            bar0_size  = 0x20000,
            mode       = "RootPort",
        )

        # PCIe Endpoint (FIXME: Should be named Root here...) --------------------------------------
        self.pcie_endpoint = LitePCIeEndpoint(
            phy                  = self.pcie_phy,
            max_pending_requests = 8,
            endianness           = self.pcie_phy.endianness,
            address_width        = 64,
            with_configuration   = True,
            with_ptm             = False,
        )

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)

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

        # CFG/MMIO Accessors -----------------------------------------------------------------------
        endpoint = self.pcie_endpoint

        self.cfg_port = cfg_port = endpoint.crossbar.get_master_port()
        self.mem_port = mem_port = endpoint.crossbar.get_master_port()

        requester_id = getattr(self.pcie_phy, "requester_id", 0x0000)

        self.cfg = LiteNVMePCIeCfgAccessor(
            port         = cfg_port,
            requester_id = requester_id,
            tag          = 0x42,
            with_csr     = True,
        )
        self.add_module(name="pcie_cfg", module=self.cfg)

        self.mmio = LiteNVMePCIeMmioAccessor(
            port     = mem_port,
            tag      = 0x44,
            with_csr = True,
        )
        self.add_module(name="pcie_mmio", module=self.mmio)

    # LiteScope Probes (Debug) ---------------------------------------------------------------------

    def add_pcie_probe(self):
        # Minimal visibility:
        # - PHY reset/link + raw req/cmp handshake
        # - CFG and MEM user ports handshake + a few key fields

        cfg_req = self.cfg_port.source
        cfg_cmp = self.cfg_port.sink
        mem_req = self.mem_port.source
        mem_cmp = self.mem_port.sink

        analyzer_signals = [
            # PHY / Link
            self.pcie_phy.pcie_rst_n,
            self.pcie_phy._link_status.fields.status,
            self.pcie_phy._link_status.fields.phy_down,
            self.pcie_phy._link_status.fields.rate,
            self.pcie_phy._link_status.fields.width,
            self.pcie_phy._link_status.fields.ltssm,

            # PHY streams (coarse activity)
            self.pcie_phy.req_sink.valid,
            self.pcie_phy.req_sink.ready,
            self.pcie_phy.req_sink.last,
            self.pcie_phy.cmp_source.valid,
            self.pcie_phy.cmp_source.ready,
            self.pcie_phy.cmp_source.last,

            # CFG port (user-level)
            cfg_req.valid,
            cfg_req.ready,
            cfg_req.first,
            cfg_req.last,
            cfg_req.is_cfg,
            cfg_req.we,
            cfg_req.tag,
            cfg_req.channel,
            cfg_req.bus_number,
            cfg_req.device_no,
            cfg_req.func,
            cfg_req.ext_reg,
            cfg_req.register_no,

            cfg_cmp.valid,
            cfg_cmp.ready,
            cfg_cmp.first,
            cfg_cmp.last,
            cfg_cmp.err,
            cfg_cmp.tag,
            cfg_cmp.len,
            cfg_cmp.end,

            # MEM port (user-level)
            mem_req.valid,
            mem_req.ready,
            mem_req.first,
            mem_req.last,
            mem_req.we,
            mem_req.tag,
            mem_req.adr,
            mem_req.len,
            mem_req.channel,

            mem_cmp.valid,
            mem_cmp.ready,
            mem_cmp.first,
            mem_cmp.last,
            mem_cmp.err,
            mem_cmp.tag,
            mem_cmp.len,
            mem_cmp.end,
        ]

        self.analyzer = LiteScopeAnalyzer(
            analyzer_signals,
            depth        = 2048,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv",
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
