#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys

from migen import *
from litex.gen import *

from usp_platform import Platform

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock import USMMCM

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.core.rootport  import LitePCIeRootPort

from litescope import LiteScopeAnalyzer

from litenvme.cfg import LiteNVMePCIeCfgAccessor
from litenvme.mem import LiteNVMePCIeMmioAccessor
from litenvme.hostmem import LiteNVMeHostMemResponder


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

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(125e6), with_cpu=False, cpu_firmware=None, cpu_boot="rom", **kwargs):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        soc_kwargs = dict(ident="LiteNVME Test SoC.", ident_version=True, cpu_type=None, uart_name="stub")
        if with_cpu:
            soc_kwargs.update(dict(
                cpu_type      = "vexriscv",
                uart_name     = "serial",
                uart_baudrate = 2e6,
            ))
            if cpu_boot == "bios":
                soc_kwargs.update(dict(
                    integrated_rom_size      = 0x10000,  # BIOS
                    integrated_main_ram_size = 0x8000,   # App load
                ))
            else:
                soc_kwargs.update(dict(
                    integrated_rom_size = 0x8000,
                ))
            if cpu_boot == "rom" and cpu_firmware is not None:
                soc_kwargs["integrated_rom_init"] = cpu_firmware
        SoCCore.__init__(self, platform, sys_clk_freq, **soc_kwargs)

        # UARTBone ---------------------------------------------------------------------------------
        if not with_cpu:
            self.add_uartbone(uart_name="serial", baudrate=2e6)

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

        # PCIe RootPort ----------------------------------------------------------------------------
        self.pcie_endpoint = LitePCIeRootPort(
            phy                  = self.pcie_phy,
            max_pending_requests = 8,
            endianness           = self.pcie_phy.endianness,
            address_width        = 64,
            with_configuration   = True,
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

        # Host Memory Responder (NVMe -> RootPort DMA target) -------------------------------------

        hostmem_base = 0x10000000
        hostmem_size = 0x8000  # 32KB

        def hostmem_decoder(a):
            return (a >= hostmem_base) & (a < (hostmem_base + hostmem_size))

        self.hostmem_port = endpoint.crossbar.get_slave_port(address_decoder=hostmem_decoder)

        self.hostmem = LiteNVMeHostMemResponder(
            port       = self.hostmem_port,
            base       = hostmem_base,
            size       = hostmem_size,
            data_width = self.pcie_phy.data_width,
            with_csr   = True,
        )


    # LiteScope Probe (Overview) -------------------------------------------------------------------

    def add_pcie_probe(self):
        # Global visibility:
        # - PHY reset/link + raw req/cmp handshake
        # - CFG/MMIO master ports (RootPort initiated)
        # - HOSTMEM slave port (NVMe initiated DMA)
        # - Hostmem DMA counters

        cfg_req = self.cfg_port.source
        cfg_cmp = self.cfg_port.sink
        mem_req = self.mem_port.source
        mem_cmp = self.mem_port.sink

        hm_req  = self.hostmem_port.sink    # NVMe Requests arriving to our hostmem responder (MemRd/MemWr).
        hm_cmp  = self.hostmem_port.source  # Completions we generate back to NVMe for MemRd.

        analyzer_signals = []

        def add(sig):
            analyzer_signals.append(sig)

        def add_opt(obj, name):
            if hasattr(obj, name):
                add(getattr(obj, name))

        def add_stream(s, include_payload=False, include_req=False, include_cmp=False):
            add_opt(s, "valid")
            add_opt(s, "ready")
            add_opt(s, "first")
            add_opt(s, "last")
            if include_payload:
                add_opt(s, "dat")
                add_opt(s, "be")
            if include_req:
                add_opt(s, "we")
                add_opt(s, "adr")
                add_opt(s, "len")
                add_opt(s, "tag")
                add_opt(s, "req_id")
            if include_cmp:
                add_opt(s, "cmp_id")
                add_opt(s, "err")
                add_opt(s, "end")

        # Clocks / Reset / Link ------------------------------------------------------------
        add(self.pcie_phy.pcie_rst_n)
        add(self.pcie_phy._link_status.fields.status)
        add(self.pcie_phy._link_status.fields.phy_down)
        add(self.pcie_phy._link_status.fields.rate)
        add(self.pcie_phy._link_status.fields.width)
        add(self.pcie_phy._link_status.fields.ltssm)

        # PHY streams (coarse activity) -----------------------------------------------
        # - req_sink: TLPs we transmit toward the NVMe (MMIO reads/writes, cfg requests).
        # - cmp_source: TLPs we receive back from NVMe (completions for our reads).
        add_stream(self.pcie_phy.req_sink)
        add_stream(self.pcie_phy.cmp_source)
        add_stream(self.pcie_phy.cmp_sink)
        add_stream(self.pcie_phy.req_source)

        # CFG master port (RootPort initiated config cycles) ------------------------------
        add_stream(cfg_req, include_req=True)
        add_opt(cfg_req, "is_cfg")
        add_opt(cfg_req, "channel")
        add_opt(cfg_req, "bus_number")
        add_opt(cfg_req, "device_no")
        add_opt(cfg_req, "func")
        add_opt(cfg_req, "ext_reg")
        add_opt(cfg_req, "register_no")

        add_stream(cfg_cmp, include_cmp=True)
        add_opt(cfg_cmp, "tag")
        add_opt(cfg_cmp, "len")

        # MEM/MMIO master port (RootPort initiated MemRd/MemWr to NVMe BAR0) --------------
        # - writes to AQA/ASQ/ACQ/CC
        # - doorbell writes (BAR0+0x1000+...)
        # - reads of CAP/VS/etc
        add_stream(mem_req, include_req=True)
        add_opt(mem_req, "channel")

        add_stream(mem_cmp, include_cmp=True)
        add_opt(mem_cmp, "tag")
        add_opt(mem_cmp, "len")

        # HOSTMEM slave port (NVMe initiated DMA toward host memory window) ----------------
        # - NVMe MemRd: fetch ASQ entry from hostmem -> hm_req.we=0 + hm_cmp traffic.
        # - NVMe MemWr: write ACQ entry + Identify buffer -> hm_req.we=1 + payload beats.
        add_stream(hm_req, include_req=True)
        add_opt(hm_req, "channel")
        add_opt(hm_req, "be")

        # HOSTMEM completions (for NVMe MemRd).
        add_stream(hm_cmp, include_cmp=True)
        add_opt(hm_cmp, "tag")
        add_opt(hm_cmp, "len")

        # Hostmem responder internals (writes/reads counters) ------------------------------
        if hasattr(self.hostmem, "csr"):
            add(self.hostmem.csr._dma_wr_count.status)
            add(self.hostmem.csr._dma_rd_count.status)

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
    parser.add_argument("--sys-clk-freq",    default=125e6,       type=float,          help="System clock frequency.")
    parser.add_argument("--with-cpu",        action="store_true",                      help="Enable VexRiscv soft CPU.")
    parser.add_argument("--cpu-boot",        default="rom", choices=["rom", "bios"],  help="CPU boot mode: ROM firmware or LiteX BIOS.")
    parser.add_argument("--cpu-firmware",    default="auto",                           help="Integrated ROM init file for soft CPU (hex/bin or 'auto').")
    parser.add_argument("--litescope-probe", default="none", choices=["none", "pcie"], help="Select LiteScope probe set.")
    args = parser.parse_args()

    def build_soc(cpu_firmware=None):
        soc = BaseSoC(
            sys_clk_freq = args.sys_clk_freq,
            with_cpu     = args.with_cpu,
            cpu_firmware = cpu_firmware,
            cpu_boot     = args.cpu_boot,
            **parser.soc_argdict,
        )
        if args.litescope_probe == "pcie":
            soc.add_pcie_probe()
        builder = Builder(soc, **parser.builder_argdict)
        if args.build:
            builder.build(**parser.toolchain_argdict)
        return soc, builder

    if args.with_cpu and args.cpu_firmware == "auto":
        # First build to generate software headers.
        soc, builder = build_soc(cpu_firmware=None)

        # Compile firmware against generated headers.
        fw_dir = os.path.join(os.path.dirname(__file__), "firmware")
        build_dir = builder.output_dir
        os.system(f"make -C {fw_dir} BUILD_DIR={build_dir} BOOT={args.cpu_boot} clean all")

        # Second build with integrated ROM init (ROM boot only).
        fw_bin = os.path.join(fw_dir, "firmware.bin")
        if args.cpu_boot == "rom":
            soc, builder = build_soc(cpu_firmware=fw_bin)
    else:
        soc, builder = build_soc(cpu_firmware=args.cpu_firmware)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
