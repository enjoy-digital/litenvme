#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

# ./alibaba_xcku3p.py --with-etherbone --csr-csv=csr.csv --build --load
# ./alibaba_xcku3p.py --with-cpu --cpu-boot=bios --csr-csv=csr.csv --with-etherbone  --build --load

import os

from migen import *

from litex.gen import *

from litex_boards.platforms import alibaba_xcku3p

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import *
from litex.soc.cores.led   import LedChaser

from liteeth.phy.usp_gty_1000basex import USP_GTY_1000BASEX

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.core.rootport  import LitePCIeRootPort

from litescope import LiteScopeAnalyzer

from litenvme.cfg     import LiteNVMePCIeCfgAccessor
from litenvme.mem     import LiteNVMePCIeMmioAccessor
from litenvme.hostmem import LiteNVMeHostMemResponder
from litenvme.req     import LiteNVMeRequestCSR

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_eth = ClockDomain()

        # Clk.
        clk100 = platform.request("clk100")

        # PLL.
        self.pll = pll = USPMMCM(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_eth, 200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(125e6), with_cpu=False, cpu_firmware=None, cpu_boot="bios",
        with_etherbone  = False,
        eth_ip          = "192.168.1.50",
        **kwargs):
        # Platform ---------------------------------------------------------------------------------
        platform = alibaba_xcku3p.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        soc_kwargs = dict(ident="LiteNVME Test SoC.", ident_version=True, cpu_type=None, uart_name="stub")
        if with_cpu:
            soc_kwargs.update(dict(
                cpu_type      = "vexriscv",
                cpu_variant   = "minimal",
                uart_name     = "crossover",
                uart_baudrate = 115200,
            ))
            if cpu_boot == "bios":
                soc_kwargs.update(dict(
                    integrated_rom_size      = 0x10000,  # BIOS
                    integrated_main_ram_size = 0x8000,   # App load
                ))
            else:
                soc_kwargs.update(dict(
                    integrated_rom_size = 0x20000,
                ))
            if cpu_boot == "rom" and cpu_firmware is not None:
                soc_kwargs["integrated_rom_init"] = cpu_firmware
        SoCCore.__init__(self, platform, sys_clk_freq, **soc_kwargs)

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_etherbone:
            self.ethphy = USP_GTY_1000BASEX(self.crg.cd_eth.clk,
                data_pads    = self.platform.request("sfp", 0),
                sys_clk_freq = self.clk_freq,
                refclk_from_fabric = True)
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-1753]")
            self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, with_ethmac=False)

        # PCIe RootPort ----------------------------------------------------------------------------
        pcie_pads = platform.request("pcie_x4")
        pcie_pads_rst_n = pcie_pads.rst_n
        pcie_pads.rst_n = Signal()
        self.pcie_phy = USPPCIEPHY(
            platform,
            pcie_pads,
            speed      = "gen2",
            data_width = 128,
            ip_name    = "pcie4_uscale_plus",
            mode       = "RootPort",
        )
        self.pcie_endpoint = LitePCIeRootPort(
            phy        = self.pcie_phy,
            max_pending_requests = 8,
            endianness           = self.pcie_phy.endianness,
            address_width        = 64,
            with_configuration   = True,
        )

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)

        # Set manual locations to avoid Vivado to remap lanes.
        platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*GTYE4_CHANNEL_PRIM_INST}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTYE4_CHANNEL_X0Y7 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gtye4_channel_gen.gen_gtye4_channel_inst[3].GTYE4_CHANNEL_PRIM_INST}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTYE4_CHANNEL_X0Y6 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gtye4_channel_gen.gen_gtye4_channel_inst[2].GTYE4_CHANNEL_PRIM_INST}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTYE4_CHANNEL_X0Y5 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gtye4_channel_gen.gen_gtye4_channel_inst[1].GTYE4_CHANNEL_PRIM_INST}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTYE4_CHANNEL_X0Y4 [get_cells -hierarchical -filter {{NAME=~*pcie_usp_i/*gtye4_channel_gen.gen_gtye4_channel_inst[0].GTYE4_CHANNEL_PRIM_INST}}]")

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

        # NVMe Request CSR (firmware-driven).
        self.req = LiteNVMeRequestCSR()
        self.add_module(name="nvme_req", module=self.req)

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
    parser = LiteXArgumentParser(platform=alibaba_xcku3p.Platform, description="LiteX SoC on Alibaba Cloud KU3P board.")
    parser.add_target_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-etherbone", action="store_true",    help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",         default="192.168.1.50", help="Etherbone IP address.")
    parser.add_argument("--with-cpu",        action="store_true",                      help="Enable VexRiscv soft CPU.")
    parser.add_argument("--cpu-boot",        default="rom", choices=["rom", "bios"], help="CPU boot mode: ROM firmware or LiteX BIOS.")
    parser.add_argument("--cpu-firmware",    default="auto",                           help="Integrated ROM init file for soft CPU (hex/bin or 'auto').")
    parser.add_argument("--litescope-probe", default="none", choices=["none", "pcie"], help="Select LiteScope probe set.")
    args = parser.parse_args()

    def build_soc(cpu_firmware=None, force_run=None):
        soc = BaseSoC(
            sys_clk_freq   = args.sys_clk_freq,
            with_cpu       = args.with_cpu,
            cpu_firmware   = cpu_firmware,
            cpu_boot       = args.cpu_boot,
            with_etherbone = args.with_etherbone,
            **parser.soc_argdict,
        )
        if args.litescope_probe == "pcie":
            soc.add_pcie_probe()
        builder = Builder(soc, **parser.builder_argdict)
        if args.build:
            toolchain_kwargs = dict(parser.toolchain_argdict)
            if force_run is not None:
                toolchain_kwargs["run"] = force_run
            elif getattr(args, "no_compile_gateware", False):
                toolchain_kwargs["run"] = False
            else:
                toolchain_kwargs.setdefault("run", True)
            builder.build(**toolchain_kwargs)
        return soc, builder

    if args.with_cpu and args.cpu_firmware == "auto":
        # First build to generate software headers.
        soc, builder = build_soc(cpu_firmware=None, force_run=False)

        # Compile firmware against generated headers.
        fw_dir = os.path.join(os.path.dirname(__file__), "firmware")
        build_dir = builder.output_dir
        os.system(f"make -C {fw_dir} BUILD_DIR={build_dir} BOOT={args.cpu_boot} clean all")

        # Second build:
        # - ROM boot: integrate firmware.bin into ROM init.
        # - BIOS boot: rebuild gateware after firmware build.
        fw_bin = os.path.join(fw_dir, "firmware.bin")
        if args.cpu_boot == "rom":
            soc, builder = build_soc(cpu_firmware=fw_bin, force_run=True)
        else:
            soc, builder = build_soc(cpu_firmware=None, force_run=True)
    else:
        soc, builder = build_soc(cpu_firmware=args.cpu_firmware)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
