#!/usr/bin/env python3

import os

from migen import *
from litex.gen import *

from usp_platform import Platform

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import USMMCM, USIDELAYCTRL
from litex.soc.cores.led   import LedChaser

from litedram.modules import MT40A1G8
from litedram.phy     import usddrphy

from litepcie.phy.usppciephy import USPPCIEPHY

from litescope import LiteScopeAnalyzer

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()

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
