#!/usr/bin/env python3

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
from litepcie.core.endpoint  import LitePCIeEndpoint

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

class BaseSoC(SoCMini):
    def __init__(self, sys_clk_freq=int(125e6), **kwargs):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteNVME Test SoC.", ident_version=True)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone(baudrate=2e6)

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
            address_mask         = 0,
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

    # LiteScope Probes (Debug) ---------------------------------------------------------------------

    def add_pcie_probe_identify(self):
        # What we want to see for Identify bring-up:
        # - RootPort -> NVMe MMIO (master port): writes to CC/AQA/ASQ/ACQ + doorbells, and reads of BAR0 regs.
        # - NVMe -> RootPort DMA (hostmem slave port): MemRd (ASQ fetch) + MemWr (ACQ + Identify buffer).
        # - Raw PHY activity to correlate.
        #
        # Notes:
        # - Field names can vary a bit across LitePCIe versions. The "activity only" probes are always safe.
        # - If some fields don't exist in your tree, just delete those lines; the structure below is the intended set.

        cfg_req = self.cfg_port.source
        cfg_cmp = self.cfg_port.sink
        mem_req = self.mem_port.source
        mem_cmp = self.mem_port.sink

        hm_req  = self.hostmem_port.sink    # NVMe Requests arriving to our hostmem responder (MemRd/MemWr).
        hm_cmp  = self.hostmem_port.source  # Completions we generate back to NVMe for MemRd.

        analyzer_signals = [
            # -----------------------------------------------------------------------------------------
            # Clocks / Reset / Link
            # -----------------------------------------------------------------------------------------
            self.pcie_phy.pcie_rst_n,
            self.pcie_phy._link_status.fields.status,
            self.pcie_phy._link_status.fields.phy_down,
            self.pcie_phy._link_status.fields.rate,
            self.pcie_phy._link_status.fields.width,
            self.pcie_phy._link_status.fields.ltssm,

            # -----------------------------------------------------------------------------------------
            # Raw PHY streams (coarse activity)
            #   - req_sink: TLPs we transmit toward the NVMe (MMIO reads/writes, cfg requests).
            #   - cmp_source: TLPs we receive back from NVMe (completions for our reads).
            # -----------------------------------------------------------------------------------------
            self.pcie_phy.req_sink.valid,
            self.pcie_phy.req_sink.ready,
            self.pcie_phy.req_sink.last,
            self.pcie_phy.cmp_source.valid,
            self.pcie_phy.cmp_source.ready,
            self.pcie_phy.cmp_source.last,

            self.pcie_phy.cmp_sink.valid,
            self.pcie_phy.cmp_sink.ready,
            self.pcie_phy.cmp_sink.last,

            self.pcie_phy.req_source.valid,
            self.pcie_phy.req_source.ready,
            self.pcie_phy.req_source.last,

            # -----------------------------------------------------------------------------------------
            # CFG master port (RootPort initiated config cycles)
            # -----------------------------------------------------------------------------------------
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

            # -----------------------------------------------------------------------------------------
            # MEM/MMIO master port (RootPort initiated MemRd/MemWr to NVMe BAR0)
            #   This is where you should see:
            #   - writes to AQA/ASQ/ACQ/CC
            #   - doorbell writes (BAR0+0x1000+...)
            #   - reads of CAP/VS/etc
            # -----------------------------------------------------------------------------------------
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

            # -----------------------------------------------------------------------------------------
            # HOSTMEM slave port (NVMe initiated DMA toward host memory window)
            #   This is the key for Identify:
            #   - NVMe MemRd: fetch ASQ entry from hostmem -> you'll see hm_req.we=0 + hm_cmp traffic.
            #   - NVMe MemWr: write ACQ entry + 4KB Identify buffer -> you'll see hm_req.we=1 + payload beats.
            # -----------------------------------------------------------------------------------------
            hm_req.valid,
            hm_req.ready,
            hm_req.first,
            hm_req.last,
            hm_req.we,
            hm_req.adr,
            hm_req.len,
            hm_req.channel,

            # Optional but very useful if present in your LitePCIe port definition:
            # (remove if your signals don't exist)
            hm_req.tag if hasattr(hm_req, "tag") else Signal(),
            hm_req.be  if hasattr(hm_req, "be")  else Signal(),

            # Payload visibility (for MemWr data into hostmem).
            # Name can be "dat" or "data" depending on LitePCIe version.
            #(hm_req.dat  if hasattr(hm_req, "dat")  else (hm_req.data if hasattr(hm_req, "data") else Signal())),

            # HOSTMEM completions (for NVMe MemRd).
            hm_cmp.valid,
            hm_cmp.ready,
            hm_cmp.first,
            hm_cmp.last,
            hm_cmp.err  if hasattr(hm_cmp, "err") else Signal(),
            hm_cmp.tag  if hasattr(hm_cmp, "tag") else Signal(),
            hm_cmp.len  if hasattr(hm_cmp, "len") else Signal(),
            hm_cmp.end  if hasattr(hm_cmp, "end") else Signal(),
            #(hm_cmp.dat  if hasattr(hm_cmp, "dat")  else (hm_cmp.data if hasattr(hm_cmp, "data") else Signal())),

            # -----------------------------------------------------------------------------------------
            # Hostmem responder internals (super handy to confirm writes/reads happen)
            # -----------------------------------------------------------------------------------------
            self.hostmem._dma_wr_count.status,
            self.hostmem._dma_rd_count.status,
        ]

        self.analyzer = LiteScopeAnalyzer(
            analyzer_signals,
            depth        = 2048,   # Identify is 4KB; deeper helps see the whole MemWr burst.
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv",
        )


    def add_pcie_probe_identify_minimal(self):
        # Convenience handles -----------------------------------------------------
        phy = self.pcie_phy
        ep  = self.pcie_endpoint

        # Pick the right depacketizer name (shared vs split PHY variants) --------
        # - shared: ep.depacketizer
        # - split : ep.req_depacketizer / ep.cmp_depacketizer
        if hasattr(ep, "req_depacketizer"):
            dp_req = ep.req_depacketizer
            dp_cmp = ep.cmp_depacketizer
        else:
            dp_req = ep.depacketizer
            dp_cmp = ep.depacketizer  # same block contains cmp_source too

        # Streams ----------------------------------------------------------------
        # What comes from NVMe into the RootPort request channel:
        phy_req = phy.req_source            # raw TLP stream from PHY (NVMe -> us)
        # What the depacketizer produced as "REQUEST" (decoded mem_rd/mem_wr):
        req_src = dp_req.req_source
        # What the crossbar sees on its slave side (should match req_src):
        xb_in   = ep.crossbar.phy_slave.sink

        # Your hostmem slave port (NVMe DMA target):
        hm_in   = self.hostmem_port.sink
        hm_out  = self.hostmem_port.source  # completions generated by hostmem

        # Completions leaving us back to the NVMe (through packetizer):
        # Depending on PHY style, packetizer completion sink name differs.
        if hasattr(ep, "cmp_packetizer"):
            # split channels
            cpl_sink = ep.cmp_packetizer.cmp_sink
        else:
            # shared channels
            cpl_sink = ep.packetizer.cmp_sink

        # Helper to safely add optional fields -----------------------------------
        analyzer_signals = []

        def add(sig):
            if sig is not None:
                analyzer_signals.append(sig)

        def add_opt(obj, name):
            add(getattr(obj, name, None))

        def add_stream(label, s):
            # Core handshake
            add_opt(s, "valid")
            add_opt(s, "ready")
            add_opt(s, "first")
            add_opt(s, "last")
            # Common payload for PHY raw stream:
            add_opt(s, "dat")
            add_opt(s, "be")
            # Decoded request fields (from depacketizer / crossbar):
            add_opt(s, "we")
            add_opt(s, "adr")
            add_opt(s, "len")
            add_opt(s, "tag")
            add_opt(s, "req_id")
            # Completion-ish:
            add_opt(s, "cmp_id")
            add_opt(s, "err")
            add_opt(s, "end")

        # Link status (useful for sanity) ----------------------------------------
        add(phy.pcie_rst_n)
        add(phy._link_status.fields.status)
        add(phy._link_status.fields.phy_down)
        add(phy._link_status.fields.ltssm)

        # 1) Raw incoming TLPs from NVMe (before decode) --------------------------
        #add_stream("phy_req_source", phy_req)

        # 2) Decoded requests from depacketizer -----------------------------------
        #add_stream("depacketizer_req_source", req_src)

        # 3) Crossbar slave input (what gets arbitrated to slaves) ----------------
        #add_stream("crossbar_phy_slave_sink", xb_in)

        # 4) Hostmem slave port input (what *you* expect to see) ------------------
        add_stream("hostmem_sink", hm_in)

        # 5) Hostmem completions output (CplD for MemRd) --------------------------
        add_stream("hostmem_source", hm_out)

        add(self.hostmem.fsm)

        # 6) Packetizer completion sink (what will go back to NVMe) ---------------
        #add_stream("packetizer_cmp_sink", cpl_sink)

        self.analyzer = LiteScopeAnalyzer(
            analyzer_signals,
            depth        = 1024,     # better chance to catch the 4KB Identify write burst
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
    #soc.add_pcie_probe()
    #soc.add_pcie_probe_identify()
    soc.add_pcie_probe_identify_minimal()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
