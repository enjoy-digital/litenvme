#!/usr/bin/env python3

#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
LiteNVMe standalone core generator

LiteNVMe aims to be used directly as a Python package when the SoC is created with LiteX.
For some use cases it is however convenient to generate a standalone Verilog file of the core:
- integration of the core in a SoC using a more traditional flow,
- need to version/package the core,
- avoid Migen/LiteX dependencies for the integrator,
- etc...

The standalone core is a PCIe *RootPort* NVMe host: it embeds the PCIe PHY + RootPort and the
LiteNVMe datapath (cfg/mmio/host-memory/I-O engine). Bring-up can run from firmware on a small
soft-CPU or from the pure-RTL init sequencer. Steady-state block I/O is exposed through a
LiteSATA-style block-streaming interface plus simple control/status pins. The configuration is
described by a YAML file, mirroring litedram_gen / litepcie_gen.

Current version targets Xilinx Ultrascale+ (USPPCIEPHY, pcie4_uscale_plus).
"""

import argparse
import os
import subprocess
import yaml

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder

from litex.build.generic_platform import *

from litepcie.core.rootport import LitePCIeRootPort

from litenvme.core import LiteNVMe, LiteNVMeCoreControl
from litenvme.cfg  import LiteNVMeRootCfgMgmt

# IOs / Interfaces ---------------------------------------------------------------------------------

def get_clk_ios():
    return [
        # Clk / Rst (internal-clk mode drives these out; external-clk mode drives them in).
        ("clk", 0, Pins(1)),
        ("rst", 0, Pins(1)),
    ]

def get_pcie_ios(phy_lanes=4):
    return [
        ("pcie", 0,
            Subsignal("rst_n", Pins(1)),  # RootPort drives PERST# to the downstream SSD.
            Subsignal("clk_p", Pins(1)),
            Subsignal("clk_n", Pins(1)),
            Subsignal("rx_p",  Pins(phy_lanes)),
            Subsignal("rx_n",  Pins(phy_lanes)),
            Subsignal("tx_p",  Pins(phy_lanes)),
            Subsignal("tx_n",  Pins(phy_lanes)),
        ),
    ]

def get_uart_ios():
    return [("serial", 0, Subsignal("tx", Pins(1)), Subsignal("rx", Pins(1)))]

def get_status_ios():
    return [
        ("status", 0,
            Subsignal("init_done",  Pins(1)),  # 1 once NVMe bring-up completed.
            Subsignal("init_error", Pins(1)),  # 1 if bring-up failed.
        ),
    ]

def get_block_ios(data_width):
    # LiteSATA-style block-streaming interface: a command/status port + two AXI-Stream data
    # ports (write payload in, read payload out). The integrator drives the command, streams
    # the write payload, and consumes the read payload — no knowledge of PRPs / host memory.
    return [
        ("block_ctrl", 0,
            Subsignal("start",  Pins(1)),
            Subsignal("write",  Pins(1)),   # 1 = write (host->SSD), 0 = read.
            Subsignal("sector", Pins(64)),  # start LBA.
            Subsignal("count",  Pins(32)),  # number of 512B sectors.
            Subsignal("nsid",   Pins(32)),
            Subsignal("done",   Pins(1)),
            Subsignal("busy",   Pins(1)),
            Subsignal("error",  Pins(1)),
        ),
        ("block_wr_axis", 0,  # write payload in.
            Subsignal("tvalid", Pins(1)),
            Subsignal("tready", Pins(1)),
            Subsignal("tlast",  Pins(1)),
            Subsignal("tdata",  Pins(data_width)),
        ),
        ("block_rd_axis", 0,  # read payload out.
            Subsignal("tvalid", Pins(1)),
            Subsignal("tready", Pins(1)),
            Subsignal("tlast",  Pins(1)),
            Subsignal("tdata",  Pins(data_width)),
        ),
    ]

# CRG ----------------------------------------------------------------------------------------------

class LiteNVMeCRG(LiteXModule):
    """Sys-clock from the PCIe user clock (internal) or from the user (external), like LitePCIe."""
    def __init__(self, platform, clk_external):
        self.cd_sys = ClockDomain()

        # # #

        platform.add_extension(get_clk_ios())
        clk = platform.request("clk")
        rst = platform.request("rst")

        pcie_clk = ClockSignal("pcie")
        pcie_rst = ResetSignal("pcie")

        # External Clk mode: the system clock is provided by the user logic.
        if clk_external:
            self.comb += self.cd_sys.clk.eq(clk)
            self.specials += AsyncResetSynchronizer(self.cd_sys, rst | pcie_rst)

        # Internal Clk mode: the core drives the system clock out from the PCIe user clock.
        else:
            self.comb += [
                clk.eq(pcie_clk),
                rst.eq(pcie_rst),
                self.cd_sys.clk.eq(pcie_clk),
                self.cd_sys.rst.eq(pcie_rst),
            ]

# LiteNVMeCore -------------------------------------------------------------------------------------

class LiteNVMeCore(SoCCore):
    def __init__(self, platform, core_config, **kwargs):
        platform.add_extension(get_pcie_ios(core_config.get("phy_lanes", 4)))
        platform.add_extension(get_status_ios())

        # Parameters -------------------------------------------------------------------------------
        init_mode    = core_config.get("init_mode", "firmware")
        sys_clk_freq = float(core_config.get("clk_freq", 125e6))
        cpu_type     = core_config.get("cpu", "vexriscv" if init_mode == "firmware" else None)
        cpu_variant  = core_config.get("cpu_variant", "minimal")
        data_width   = core_config.get("data_width", 256)
        uart_name    = core_config.get("uart", "serial" if cpu_type is not None else "stub")
        cpu_firmware = core_config.get("_cpu_firmware", None)  # injected by main() (2-pass build).
        hostmem_base = core_config.get("hostmem_base", 0x1000_0000)
        hostmem_size = core_config.get("hostmem_size", 0x8_0000)

        if uart_name == "serial":
            platform.add_extension(get_uart_ios())

        # SoCCore (CPU embedded, like litedram) ----------------------------------------------------
        soc_kwargs = dict(
            ident         = "LiteNVMe standalone core",
            ident_version = True,
            cpu_type      = cpu_type,
            cpu_variant   = cpu_variant,
            uart_name     = uart_name,
        )
        # firmware: "auto"/"minimal" bake the NVMe firmware into ROM (no main_ram); "none"
        # boots the LiteX BIOS (ROM + main_ram). The same memory map must hold across both
        # two-pass builds, so the ROM-boot layout is chosen from `firmware`, not from whether
        # the firmware binary is present yet.
        rom_boot = core_config.get("firmware", "none") in ("auto", "minimal")
        if cpu_type is not None:
            if rom_boot:
                soc_kwargs["integrated_rom_size"] = core_config.get("integrated_rom_size", 0x20000)
                if cpu_firmware is not None:
                    soc_kwargs["integrated_rom_init"] = cpu_firmware
            else:
                soc_kwargs["integrated_rom_size"]      = core_config.get("integrated_rom_size", 0x10000)
                soc_kwargs["integrated_main_ram_size"] = core_config.get("integrated_main_ram_size", 0x10000)
        SoCCore.__init__(self, platform, sys_clk_freq, **soc_kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.crg = LiteNVMeCRG(platform, core_config.get("clk_external", False))

        # PCIe PHY (RootPort) ----------------------------------------------------------------------
        pcie_pads       = platform.request("pcie")
        pcie_pads_rst_n = pcie_pads.rst_n
        pcie_pads.rst_n = Signal()

        phy_class = core_config["phy"]
        self.pcie_phy = phy_class(
            platform, pcie_pads,
            speed         = core_config.get("phy_speed", "gen3"),
            data_width    = data_width,
            ip_name       = core_config.get("phy_ip_name", "pcie4_uscale_plus"),
            mode          = "RootPort",
            with_cfg_mgmt = True,
        )
        self.pcie_phy.update_config(core_config.get("phy_config", {}))

        # PCIe RootPort one-shot reset (we drive PERST# to the SSD after link-up settle).
        self.pcie_rst_timer = WaitTimer(int(100e-3 * sys_clk_freq))  # 100ms.
        self.comb += self.pcie_rst_timer.wait.eq(1)
        self.comb += [
            pcie_pads_rst_n.eq(self.pcie_rst_timer.done),
            pcie_pads.rst_n.eq(self.pcie_rst_timer.done),
        ]

        # PCIe Endpoint (RootPort) -----------------------------------------------------------------
        self.pcie_endpoint = LitePCIeRootPort(
            phy                  = self.pcie_phy,
            max_pending_requests = core_config.get("ep_max_pending_requests", 8),
            endianness           = self.pcie_phy.endianness,
            address_width        = core_config.get("ep_address_width", 64),
            with_configuration   = True,
        )

        # Root-port cfg management (raise our own DevCtl.MPS / open memory window).
        accessor_csr = init_mode == "firmware"
        self.pcie_rootcfg = LiteNVMeRootCfgMgmt(self.pcie_phy, with_csr=accessor_csr)

        extra_axi_masters = []
        if init_mode == "rtl":
            from litenvme.init import LiteNVMeInitSequencer
            from litenvme.io_engine import LiteNVMeMemPortToAXI
            self.nvme_init = init_seq = LiteNVMeInitSequencer(
                bar0_base       = core_config.get("bar0_base", 0xe000_0000),
                hostmem_base    = hostmem_base,
                io_q_entries    = core_config.get("qsize", 64),
                io_qid          = core_config.get("qid", 1),
                mem_window_dw   = core_config.get("root_mem_window_dw", 0x20),
                mem_window_val  = core_config.get("root_mem_window_val", 0xe000_e000),
                with_csr        = False,
            )
            self.nvme_init_bridge = init_bridge = LiteNVMeMemPortToAXI(
                init_seq.mem, data_width=data_width, address_width=32, base=hostmem_base)
            extra_axi_masters.append(init_bridge.axi)

        # LiteNVMe core ----------------------------------------------------------------------------
        with_request_gen    = core_config.get("with_request_gen", True)
        with_block_streamer = core_config.get("with_block_streamer", False)
        # When the block streamer is the sole front-end, expose its data/command interface at
        # the top (pin-driven by external logic); otherwise it (if present) is CSR-driven.
        expose_block_pins   = with_block_streamer and not with_request_gen

        self.nvme = nvme = LiteNVMe(
            pcie_endpoint    = self.pcie_endpoint,
            hostmem_base     = hostmem_base,
            hostmem_size     = hostmem_size,
            data_width       = data_width,
            qid              = core_config.get("qid",   1),
            qsize            = core_config.get("qsize", 64),
            qd               = core_config.get("qd",    32),
            requester_id     = core_config.get("requester_id", 0x0000),
            with_request_gen = with_request_gen,
            with_block_streamer = with_block_streamer,
            block_streamer_csr  = not expose_block_pins,
            # The CSR memory-debug frontend adds a third BRAM port that replicates the whole
            # host-memory window; the pin-driven standalone core does not need it.
            with_hostmem_csr = core_config.get("with_hostmem_csr", (not expose_block_pins) and accessor_csr),
            staging_base     = core_config.get("staging_base", 0x4_0000),
            staging_size     = core_config.get("staging_size", 0x4_0000),
            with_csr         = accessor_csr,
            with_accessor_csr = accessor_csr,
            with_engine_csr  = accessor_csr,
            with_request_gen_csr = accessor_csr,
            extra_axi_masters = extra_axi_masters,
        )
        if accessor_csr:
            nvme.add_csrs(self)

        # Core control (init_done / init_error) wired to top-level status pins -----------------
        status_pads = platform.request("status")
        if init_mode == "firmware":
            self.nvme_ctrl = LiteNVMeCoreControl()
            init_done = self.nvme_ctrl.init_done.storage
            init_error = self.nvme_ctrl.init_error.storage
        else:
            seq = self.nvme_init
            init_done = seq.init_done
            init_error = seq.init_error
            self.comb += [
                # Auto-start once the downstream SSD is released from reset.
                seq.start.eq(self.pcie_rst_timer.done),
                # CFG accessor.
                nvme.cfg.start.eq(seq.cfg_start), nvme.cfg.we.eq(seq.cfg_we),
                nvme.cfg.wdata.eq(seq.cfg_wdata), nvme.cfg.reg.eq(seq.cfg_reg),
                nvme.cfg.ext_reg.eq(0), nvme.cfg.bus.eq(seq.cfg_bus),
                nvme.cfg.device.eq(seq.cfg_dev), nvme.cfg.function.eq(seq.cfg_func),
                seq.cfg_done.eq(nvme.cfg.done), seq.cfg_err.eq(nvme.cfg.err),
                seq.cfg_rdata.eq(nvme.cfg.rdata),
                # MMIO accessor.
                nvme.mmio.start.eq(seq.mmio_start), nvme.mmio.we.eq(seq.mmio_we),
                nvme.mmio.adr.eq(seq.mmio_adr), nvme.mmio.wdata.eq(seq.mmio_wdata),
                nvme.mmio.wsel.eq(seq.mmio_wsel), nvme.mmio.len.eq(seq.mmio_len),
                seq.mmio_done.eq(nvme.mmio.done), seq.mmio_err.eq(nvme.mmio.err),
                seq.mmio_rdata.eq(nvme.mmio.rdata),
                # Root cfg-mgmt (memory window).
                self.pcie_rootcfg.start.eq(seq.root_start),
                self.pcie_rootcfg.write.eq(seq.root_write),
                self.pcie_rootcfg.addr.eq(seq.root_addr),
                self.pcie_rootcfg.wdata.eq(seq.root_wdata),
                self.pcie_rootcfg.be.eq(0xf), self.pcie_rootcfg.func.eq(0),
                seq.root_done.eq(self.pcie_rootcfg.done),
                # Engine config.
                nvme.io_engine.enable.eq(seq.eng_enable),
                nvme.io_engine.sq_base.eq(seq.eng_sq_base),
                nvme.io_engine.cq_base.eq(seq.eng_cq_base),
                nvme.io_engine.sq_db_adr.eq(seq.eng_sq_db),
                nvme.io_engine.cq_db_adr.eq(seq.eng_cq_db),
                nvme.io_engine.prp_list_base.eq(seq.eng_prp_list),
            ]
        self.comb += [
            status_pads.init_done.eq(init_done),
            status_pads.init_error.eq(init_error),
        ]

        # Block-streaming interface to the top (when it is the sole front-end) ------------------
        if expose_block_pins:
            platform.add_extension(get_block_ios(data_width))
            bs   = nvme.block_streamer
            ctrl = platform.request("block_ctrl")
            wr   = platform.request("block_wr_axis")
            rd   = platform.request("block_rd_axis")
            self.comb += [
                # Command (start gated on init_done so no transfer runs before bring-up).
                bs.start.eq(ctrl.start & init_done),
                bs.write.eq(ctrl.write),
                bs.sector.eq(ctrl.sector),
                bs.count.eq(ctrl.count),
                bs.nsid.eq(ctrl.nsid),
                ctrl.done.eq(bs.done),
                ctrl.busy.eq(bs.busy),
                ctrl.error.eq(bs.error),
                # Write payload in.
                bs.wr_sink.valid.eq(wr.tvalid),
                wr.tready.eq(bs.wr_sink.ready),
                bs.wr_sink.last.eq(wr.tlast),
                bs.wr_sink.data.eq(wr.tdata),
                # Read payload out.
                rd.tvalid.eq(bs.rd_source.valid),
                bs.rd_source.ready.eq(rd.tready),
                rd.tlast.eq(bs.rd_source.last),
                rd.tdata.eq(bs.rd_source.data),
            ]

# Build --------------------------------------------------------------------------------------------

_CONFIG_KEYS = {
    "phy",
    "phy_device",
    "phy_lanes",
    "phy_speed",
    "phy_ip_name",
    "phy_config",
    "clk_freq",
    "clk_external",
    "cpu",
    "cpu_variant",
    "uart",
    "firmware",
    "init_mode",
    "integrated_rom_size",
    "integrated_main_ram_size",
    "data_width",
    "ep_max_pending_requests",
    "ep_address_width",
    "hostmem_base",
    "hostmem_size",
    "qid",
    "qsize",
    "qd",
    "requester_id",
    "bar0_base",
    "root_mem_window_dw",
    "root_mem_window_val",
    "with_request_gen",
    "with_block_streamer",
    "with_hostmem_csr",
    "staging_base",
    "staging_size",
}

def _normalize_config(core_config):
    if not isinstance(core_config, dict):
        raise ValueError("LiteNVMe config must be a YAML mapping.")

    normalized = {}
    replaces = {"False": False, "True": True, "None": None}
    for k, v in core_config.items():
        normalized[k] = replaces.get(v, v) if isinstance(v, str) else v

    if "clk_freq" in normalized:
        normalized["clk_freq"] = float(normalized["clk_freq"])

    normalized.setdefault("init_mode", "firmware")
    normalized.setdefault("firmware", "none")
    normalized.setdefault("with_block_streamer", False)
    if normalized["with_block_streamer"] and "with_request_gen" not in normalized:
        normalized["with_request_gen"] = False
    normalized.setdefault("with_request_gen", True)

    # CPU-less RTL init is the standalone default for init_mode="rtl". Users can still set a
    # CPU explicitly for debug, but the generated core does not require one.
    if normalized["init_mode"] == "rtl":
        normalized.setdefault("cpu", None)
        normalized.setdefault("uart", "stub")

    return normalized

def validate_config(core_config):
    unknown = sorted(set(core_config) - _CONFIG_KEYS)
    if unknown:
        raise ValueError("Unknown LiteNVMe config key(s): " + ", ".join(unknown))

    for key in ("phy", "phy_device"):
        if key not in core_config:
            raise ValueError(f"Missing required LiteNVMe config key: {key}")

    init_mode = core_config.get("init_mode", "firmware")
    if init_mode not in ("firmware", "rtl"):
        raise ValueError("init_mode must be 'firmware' or 'rtl'.")

    firmware = core_config.get("firmware", "none")
    if firmware not in ("none", "auto", "minimal"):
        raise ValueError("firmware must be 'none', 'auto' or 'minimal'.")

    if init_mode == "firmware" and core_config.get("cpu", "vexriscv") is None:
        raise ValueError("init_mode='firmware' requires a CPU; use init_mode='rtl' for CPU-less bring-up.")

    if init_mode == "rtl" and firmware != "none":
        raise ValueError("init_mode='rtl' does not use firmware; set firmware='none'.")

    data_width = core_config.get("data_width", 256)
    if data_width not in (64, 128, 256, 512):
        raise ValueError("data_width must be one of: 64, 128, 256, 512.")

    if core_config.get("with_request_gen", False) and core_config.get("with_block_streamer", False):
        raise ValueError("The standalone generator exposes one front-end; enable request_gen or block_streamer, not both.")

def load_config(filename):
    with open(filename, "r", encoding="utf-8") as f:
        core_config = yaml.safe_load(f)
    core_config = _normalize_config(core_config)
    validate_config(core_config)
    return core_config

# phy string -> (PHY class, platform factory).
def _resolve_platform(core_config):
    phy = core_config["phy"]
    if phy == "USPPCIEPHY":
        from litex.build.xilinx import XilinxPlatform
        from litepcie.phy.usppciephy import USPPCIEPHY
        platform = XilinxPlatform(core_config["phy_device"], io=[], toolchain="vivado")
        core_config["phy"] = USPPCIEPHY
        return platform
    raise ValueError(f"Unsupported PCIe PHY: {phy}")

def main():
    parser = argparse.ArgumentParser(description="LiteNVMe standalone core generator")
    parser.add_argument("config",       help="YAML config file")
    parser.add_argument("--output-dir", default="build", help="Build output directory.")
    parser.add_argument("--name",       default="litenvme_core", help="Standalone core/module name.")
    args = parser.parse_args()

    core_config = load_config(args.config)

    platform = _resolve_platform(core_config)
    firmware = core_config.get("firmware", "none")

    def build(cpu_firmware, build_name):
        cfg = dict(core_config, _cpu_firmware=cpu_firmware)
        soc     = LiteNVMeCore(platform, cfg)
        builder = Builder(soc, output_dir=args.output_dir, compile_gateware=False)
        builder.build(build_name=build_name, regular_comb=False)
        return soc, builder

    if core_config.get("cpu", "vexriscv") is not None and firmware in ("auto", "minimal"):
        # Two-pass build: (1) emit software headers, (2) compile firmware, (3) bake into ROM.
        # "minimal" -> code-only firmware (auto-init + idle, no console/prints) via MINIMAL=1.
        soc, builder = build(cpu_firmware=None, build_name=args.name)
        fw_dir   = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "bench", "firmware"))
        build_abs = os.path.abspath(builder.output_dir)
        make_args = "BOOT=rom" + (" MINIMAL=1" if firmware == "minimal" else "")
        subprocess.check_call(["make", "-C", fw_dir, f"BUILD_DIR={build_abs}", *make_args.split(), "clean", "all"])
        fw_bin = os.path.join(fw_dir, "firmware.bin")
        soc, builder = build(cpu_firmware=fw_bin, build_name=args.name)
    else:
        soc, builder = build(cpu_firmware=None, build_name=args.name)

if __name__ == "__main__":
    main()
