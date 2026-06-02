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

from litex.soc.integration.soc import *
from litex.soc.integration.soc_core import *  # SoCCore (moved out of .soc in recent LiteX).
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import *
from litex.soc.cores.led   import LedChaser

from liteeth.phy.usp_gty_1000basex import USP_GTY_1000BASEX

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.core.rootport  import LitePCIeRootPort

from litescope import LiteScopeAnalyzer

from litenvme.cfg       import LiteNVMePCIeCfgAccessor, LiteNVMeRootCfgMgmt
from litenvme.mem       import LiteNVMePCIeMmioAccessor
from litenvme.hostmem   import LiteNVMeHostMemResponder
from litenvme.io_engine   import LiteNVMeIOEngineAXI
from litenvme.request_gen import LiteNVMeRequestGen
from litenvme.block       import LiteNVMeBlockStreamer
from litenvme.core        import LiteNVMeCoreControl

# Block-streamer BIST ------------------------------------------------------------------------------

class _BlockBIST(LiteXModule):
    """CSR-driven counter source / checker around the block streamer's data ports.

    The board has no external data source for wr_sink, so to exercise the streamer end to
    end over Etherbone we feed wr_sink a per-beat counter and check rd_source against the
    same sequence: write the counter to an LBA range, read it back, and PASS when errors==0.
    Counters reset on each streamer `start` (so a write run and the following read run both
    start the sequence at 0; the SSD round-trip makes the readback match)."""
    def __init__(self, streamer, data_width):
        self._sel = CSRStorage(1, description="Engine front-end: 0=request_gen, 1=block streamer.")
        self.sel  = self._sel.storage
        self._control = CSRStorage(fields=[
            CSRField("wr_en", size=1, offset=0, description="Feed wr_sink with a counter pattern."),
            CSRField("rd_en", size=1, offset=1, description="Check rd_source against the counter."),
        ])
        self._errors   = CSRStatus(32, description="Read-back mismatches.")
        self._wr_beats = CSRStatus(32, description="Beats fed into wr_sink.")
        self._rd_beats = CSRStatus(32, description="Beats checked from rd_source.")

        # # #

        wr_cnt   = Signal(data_width)
        rd_cnt   = Signal(data_width)
        errors   = Signal(32)
        wr_beats = Signal(32)
        rd_beats = Signal(32)

        wr_fire = Signal()
        rd_fire = Signal()
        self.comb += [
            streamer.wr_sink.valid.eq(self._control.fields.wr_en),
            streamer.wr_sink.data.eq(wr_cnt),
            streamer.rd_source.ready.eq(self._control.fields.rd_en),
            wr_fire.eq(streamer.wr_sink.valid & streamer.wr_sink.ready),
            rd_fire.eq(streamer.rd_source.valid & streamer.rd_source.ready),
            self._errors.status.eq(errors),
            self._wr_beats.status.eq(wr_beats),
            self._rd_beats.status.eq(rd_beats),
        ]
        self.sync += [
            If(streamer.start,
                wr_cnt.eq(0), rd_cnt.eq(0), errors.eq(0), wr_beats.eq(0), rd_beats.eq(0),
            ).Else(
                If(wr_fire,
                    wr_cnt.eq(wr_cnt + 1),
                    wr_beats.eq(wr_beats + 1),
                ),
                If(rd_fire,
                    rd_cnt.eq(rd_cnt + 1),
                    rd_beats.eq(rd_beats + 1),
                    If(streamer.rd_source.data != rd_cnt, errors.eq(errors + 1)),
                ),
            ),
        ]

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
        with_io_engine  = False,
        io_engine_qd    = 32,
        with_block_streamer = False,
        staging_base    = 0x40000,
        staging_size    = 0x40000,
        with_rtl_init   = False,
        ep_max_pending_requests = 8,
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
                    integrated_main_ram_size = 0x10000,  # App load (room for diag/MPS commands)
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
        # Gen3 x4 @ 256-bit core datapath (256b @ sys 125MHz = 4.0 GB/s, the throughput target).
        # pcie_data_width == data_width (256) so the hard IP AXI-S is 256b @ 125MHz (axisten_freq
        # override) -- same-width, same-rate CDC, no StrideConverter. RQ-adapter 256b fix landed
        # (config works); MMIO-completion under LiteScope investigation.
        self.pcie_phy = USPPCIEPHY(
            platform,
            pcie_pads,
            speed           = "gen3",
            data_width      = 256,
            ip_name         = "pcie4_uscale_plus",
            mode            = "RootPort",
            with_cfg_mgmt   = True,  # Expose local cfg_mgmt to program the root-port DevCtl.MPS.
        )
        self.pcie_phy.update_config({
            "mode_selection"   : "Advanced",
            "en_gt_selection"  : "true",
            "select_quad"      : "GTY_Quad_225",
            "pcie_blk_locn"    : "X0Y0",
            "gen_x0y0"         : "true",
            "gen_x1y0"         : "false",
            "plltype"          : "QPLL0",  # Gen3 (8 GT/s) high-band PLL.
            # 256-bit Gen3 x4: user clock 125MHz (256b@125=4GB/s), core clock 250MHz.
            "axisten_freq"     : 125,
            "coreclk_freq"     : 250,
        })
        self.pcie_endpoint = LitePCIeRootPort(
            phy        = self.pcie_phy,
            max_pending_requests = ep_max_pending_requests,
            endianness           = self.pcie_phy.endianness,
            address_width        = 64,
            with_configuration   = True,
        )

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)

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

        # With the RTL init sequencer, cfg/mmio/rootcfg are driven by hardware (no firmware), so
        # they carry no CSRs; otherwise they are firmware-driven (the default path).
        acc_csr = not with_rtl_init

        self.cfg = LiteNVMePCIeCfgAccessor(
            port         = cfg_port,
            requester_id = requester_id,
            tag          = 0x42,
            with_csr     = acc_csr,
        )
        if acc_csr:
            self.add_module(name="pcie_cfg", module=self.cfg)

        # Root-port cfg management: lets firmware raise our own DevCtl.MPS (read path bottleneck).
        # (Auto-registers as "pcie_rootcfg" -> firmware CSRs pcie_rootcfg_*.)
        self.pcie_rootcfg = LiteNVMeRootCfgMgmt(self.pcie_phy, with_csr=acc_csr)

        self.mmio = LiteNVMePCIeMmioAccessor(
            port     = mem_port,
            tag      = 0x44,
            with_csr = acc_csr,
        )
        if acc_csr:
            self.add_module(name="pcie_mmio", module=self.mmio)

        # Host Memory Responder (NVMe -> RootPort DMA target) -------------------------------------

        hostmem_base = 0x10000000
        hostmem_size = 0x80000  # 512KB: admin/IO queues + per-slot 4KiB I/O buffers for QD>1.

        def hostmem_decoder(a):
            return (a >= hostmem_base) & (a < (hostmem_base + hostmem_size))

        # Optional hardware I/O command engine (steady-state I/O without per-command CPU
        # cost). Joins the hostmem AXI backend as an extra master and drives SQ/CQ
        # doorbells through its own dedicated MMIO accessor. Firmware still does admin
        # setup (enable controller, create IO SQ/CQ) and programs the engine's queue /
        # doorbell addresses + enable via CSR.
        io_engine_masters = None
        if with_io_engine:
            self.io_engine = io_engine = LiteNVMeIOEngineAXI(
                qid=1, qsize=64, qd=io_engine_qd,
                data_width=self.pcie_phy.data_width, with_csr=acc_csr,
                hostmem_base=hostmem_base)
            if acc_csr:
                self.add_module(name="nvme_engine", module=io_engine.engine)

            io_db_port = endpoint.crossbar.get_master_port()
            self.mmio_db = LiteNVMePCIeMmioAccessor(port=io_db_port, tag=0x45, with_csr=False)
            self.comb += io_engine.connect_mmio(self.mmio_db)
            io_engine_masters = [io_engine.axi]

            # Core control (init_done) — firmware raises it after NVMe bring-up so host scripts
            # over Etherbone can poll readiness before driving the engine.
            self.nvme_ctrl = LiteNVMeCoreControl()

            # Hardware request generator: drives the engine at full rate (no CPU in the
            # steady-state loop) and counts completions/cycles/errors, so the engine's
            # true throughput can be measured. Firmware programs it via nvme_gen_* CSRs.
            self.nvme_gen = nvme_gen = LiteNVMeRequestGen(with_csr=True)

            # Optional block streamer (the LiteSATA-style front-end) for correctness testing
            # over Etherbone. Both front-ends are present; a 1-bit CSR muxes which one owns the
            # engine streams (0 = request_gen / throughput, 1 = block streamer / correctness).
            if with_block_streamer:
                self.nvme_block = nvme_block = LiteNVMeBlockStreamer(
                    data_width=self.pcie_phy.data_width, hostmem_base=hostmem_base,
                    staging_base=staging_base, staging_size=staging_size, with_csr=True)
                io_engine_masters.append(nvme_block.dma.axi)
                self.nvme_block_bist = bist = _BlockBIST(nvme_block, self.pcie_phy.data_width)

                sel = bist.sel
                self.comb += If(sel,
                    *nvme_block.source.connect(io_engine.sink),
                    *io_engine.source.connect(nvme_block.sink),
                ).Else(
                    *nvme_gen.source.connect(io_engine.sink),
                    *io_engine.source.connect(nvme_gen.sink),
                )
            else:
                self.comb += [
                    nvme_gen.source.connect(io_engine.sink),
                    io_engine.source.connect(nvme_gen.sink),
                ]

            # Optional pure-RTL init sequencer (CPU-less bring-up). When enabled it drives the
            # cfg/mmio/rootcfg accessors + the engine config (firmware does none of the init), and
            # joins the hostmem arbiter with its own dword master for admin SQE/CQE access. A host
            # pulses nvme_init_ctrl.start over Etherbone and polls nvme_init_status.
            if with_rtl_init:
                from litenvme.init import LiteNVMeInitSequencer
                from litenvme.io_engine import LiteNVMeMemPortToAXI
                self.nvme_init = seq = LiteNVMeInitSequencer(
                    bar0_base=0xe0000000, hostmem_base=hostmem_base, io_q_entries=64, with_csr=True)
                self.nvme_init_bridge = init_bridge = LiteNVMeMemPortToAXI(
                    seq.mem, data_width=self.pcie_phy.data_width, address_width=32, base=hostmem_base)
                io_engine_masters.append(init_bridge.axi)
                self.comb += [
                    # CFG accessor.
                    self.cfg.start.eq(seq.cfg_start), self.cfg.we.eq(seq.cfg_we),
                    self.cfg.wdata.eq(seq.cfg_wdata), self.cfg.reg.eq(seq.cfg_reg),
                    self.cfg.ext_reg.eq(0), self.cfg.bus.eq(seq.cfg_bus),
                    self.cfg.device.eq(seq.cfg_dev), self.cfg.function.eq(seq.cfg_func),
                    seq.cfg_done.eq(self.cfg.done), seq.cfg_err.eq(self.cfg.err),
                    seq.cfg_rdata.eq(self.cfg.rdata),
                    # MMIO accessor.
                    self.mmio.start.eq(seq.mmio_start), self.mmio.we.eq(seq.mmio_we),
                    self.mmio.adr.eq(seq.mmio_adr), self.mmio.wdata.eq(seq.mmio_wdata),
                    self.mmio.wsel.eq(seq.mmio_wsel), self.mmio.len.eq(seq.mmio_len),
                    seq.mmio_done.eq(self.mmio.done), seq.mmio_err.eq(self.mmio.err),
                    seq.mmio_rdata.eq(self.mmio.rdata),
                    # Root cfg-mgmt (memory window).
                    self.pcie_rootcfg.start.eq(seq.root_start),
                    self.pcie_rootcfg.write.eq(seq.root_write),
                    self.pcie_rootcfg.addr.eq(seq.root_addr),
                    self.pcie_rootcfg.wdata.eq(seq.root_wdata),
                    self.pcie_rootcfg.be.eq(0xf), self.pcie_rootcfg.func.eq(0),
                    seq.root_done.eq(self.pcie_rootcfg.done),
                    # Engine config (from the sequencer instead of firmware CSRs).
                    io_engine.enable.eq(seq.eng_enable), io_engine.sq_base.eq(seq.eng_sq_base),
                    io_engine.cq_base.eq(seq.eng_cq_base), io_engine.sq_db_adr.eq(seq.eng_sq_db),
                    io_engine.cq_db_adr.eq(seq.eng_cq_db), io_engine.prp_list_base.eq(seq.eng_prp_list),
                ]

        self.hostmem_port = endpoint.crossbar.get_slave_port(address_decoder=hostmem_decoder)
        self.hostmem = LiteNVMeHostMemResponder(
            port              = self.hostmem_port,
            base              = hostmem_base,
            size              = hostmem_size,
            data_width        = self.pcie_phy.data_width,
            with_csr          = True,
            extra_axi_masters = io_engine_masters,
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
    parser.add_argument("--with-io-engine",  action="store_true",                      help="Enable the hardware NVMe I/O command engine.")
    parser.add_argument("--io-engine-qd",    default=32, type=int,                     help="I/O engine queue depth (commands outstanding).")
    parser.add_argument("--with-block-streamer", action="store_true",                  help="Add the block-streamer front-end + BIST (Etherbone correctness).")
    parser.add_argument("--with-rtl-init",       action="store_true",                  help="Pure-RTL NVMe bring-up (no firmware init; cfg/mmio/engine driven by the sequencer).")
    parser.add_argument("--ep-max-pending-requests", default=8, type=int,               help="LitePCIe outstanding-request buffers (control-plane); smaller saves BRAM.")
    args = parser.parse_args()

    def build_soc(cpu_firmware=None, force_run=None):
        soc = BaseSoC(
            sys_clk_freq   = args.sys_clk_freq,
            with_cpu       = args.with_cpu,
            cpu_firmware   = cpu_firmware,
            cpu_boot       = args.cpu_boot,
            with_etherbone = args.with_etherbone,
            with_io_engine = args.with_io_engine,
            io_engine_qd   = args.io_engine_qd,
            with_block_streamer = args.with_block_streamer,
            with_rtl_init       = args.with_rtl_init,
            ep_max_pending_requests = args.ep_max_pending_requests,
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

        # Compile firmware against generated headers. BUILD_DIR must be absolute: the firmware
        # make runs with `-C firmware`, so a relative output_dir would resolve under firmware/.
        fw_dir = os.path.join(os.path.dirname(__file__), "firmware")
        build_dir = os.path.abspath(builder.output_dir)
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
