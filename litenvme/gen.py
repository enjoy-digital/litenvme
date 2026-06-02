#!/usr/bin/env python3

#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""
LiteNVMe standalone core generator

LiteNVMe aims to be used directly as a Python package when the SoC is created with LiteX.
For some use cases it is however convenient to generate a standalone Verilog file of the core:
- integration of the core in a SoC using a more traditional flow,
- need to version/package the core,
- avoid Migen/LiteX dependencies for the integrator,
- etc...

The standalone core is a PCIe *RootPort* NVMe host: it embeds the PCIe PHY + RootPort, the
LiteNVMe datapath (cfg/mmio/host-memory/I-O engine) and a small soft-CPU that runs the NVMe
bring-up firmware and raises `init_done`. Steady-state block I/O is exposed through a
LiteSATA-style block-streaming interface (added in litenvme/block.py) plus simple
control/status pins. The configuration is described by a YAML file, mirroring
litedram_gen / litepcie_gen.

Current version targets Xilinx Ultrascale+ (USPPCIEPHY, pcie4_uscale_plus).
"""

import os
import yaml
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder

from litex.build.generic_platform import *

from litepcie.core.rootport import LitePCIeRootPort

from litenvme.core import LiteNVMe
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
            Subsignal("init_done",  Pins(1)),  # 1 once firmware finished NVMe bring-up.
            Subsignal("init_error", Pins(1)),  # 1 if bring-up failed.
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

# LiteNVMeCoreControl ------------------------------------------------------------------------------

class LiteNVMeCoreControl(LiteXModule):
    """init_done / init_error: firmware sets them after NVMe bring-up; wired to top-level pins."""
    def __init__(self):
        self.init_done  = CSRStorage()
        self.init_error = CSRStorage()

# LiteNVMeCore -------------------------------------------------------------------------------------

class LiteNVMeCore(SoCCore):
    def __init__(self, platform, core_config, **kwargs):
        platform.add_extension(get_pcie_ios(core_config.get("phy_lanes", 4)))
        platform.add_extension(get_status_ios())

        # Parameters -------------------------------------------------------------------------------
        sys_clk_freq = float(core_config.get("clk_freq", 125e6))
        cpu_type     = core_config.get("cpu", "vexriscv")
        cpu_variant  = core_config.get("cpu_variant", "minimal")
        data_width   = core_config.get("data_width", 256)
        uart_name    = core_config.get("uart", "serial")
        cpu_firmware = core_config.get("_cpu_firmware", None)  # injected by main() (2-pass build).

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
        if cpu_type is not None:
            if cpu_firmware is not None:
                soc_kwargs.update(dict(
                    integrated_rom_size = core_config.get("integrated_rom_size", 0x20000),
                    integrated_rom_init = cpu_firmware,
                ))
            else:
                soc_kwargs.update(dict(
                    integrated_rom_size      = core_config.get("integrated_rom_size", 0x10000),
                    integrated_main_ram_size = core_config.get("integrated_main_ram_size", 0x10000),
                ))
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

        # LiteNVMe core ----------------------------------------------------------------------------
        self.nvme = nvme = LiteNVMe(
            pcie_endpoint    = self.pcie_endpoint,
            hostmem_base     = core_config.get("hostmem_base", 0x1000_0000),
            hostmem_size     = core_config.get("hostmem_size", 0x8_0000),
            data_width       = data_width,
            qid              = core_config.get("qid",   1),
            qsize            = core_config.get("qsize", 64),
            qd               = core_config.get("qd",    32),
            requester_id     = core_config.get("requester_id", 0x0000),
            with_request_gen = core_config.get("with_request_gen", True),
        )
        nvme.add_csrs(self)
        if core_config.get("with_request_gen", True):
            self.add_module(name="nvme_gen", module=nvme.request_gen)

        # Root-port cfg management (raise our own DevCtl.MPS) — firmware uses it for auto-MPS.
        self.pcie_rootcfg = LiteNVMeRootCfgMgmt(self.pcie_phy)

        # Core control (init_done / init_error) wired to top-level status pins -----------------
        self.nvme_ctrl = LiteNVMeCoreControl()
        status_pads = platform.request("status")
        self.comb += [
            status_pads.init_done.eq(self.nvme_ctrl.init_done.storage),
            status_pads.init_error.eq(self.nvme_ctrl.init_error.storage),
        ]

# Build --------------------------------------------------------------------------------------------

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

    core_config = yaml.load(open(args.config).read(), Loader=yaml.Loader)

    # Normalize YAML scalars (mirror litedram/litepcie).
    for k, v in core_config.items():
        replaces = {"False": False, "True": True, "None": None}
        if isinstance(v, str) and v in replaces:
            core_config[k] = replaces[v]
    if "clk_freq" in core_config:
        core_config["clk_freq"] = float(core_config["clk_freq"])

    platform = _resolve_platform(core_config)
    firmware = core_config.get("firmware", "none")

    def build(cpu_firmware, build_name):
        cfg = dict(core_config, _cpu_firmware=cpu_firmware)
        soc     = LiteNVMeCore(platform, cfg)
        builder = Builder(soc, output_dir=args.output_dir, compile_gateware=False)
        builder.build(build_name=build_name, regular_comb=False)
        return soc, builder

    if core_config.get("cpu") is not None and firmware == "auto":
        # Two-pass build: (1) emit software headers, (2) compile firmware, (3) bake into ROM.
        soc, builder = build(cpu_firmware=None, build_name=args.name)
        fw_dir = os.path.join(os.path.dirname(__file__), "..", "bench", "firmware")
        os.system(f"make -C {fw_dir} BUILD_DIR={builder.output_dir} BOOT=rom clean all")
        fw_bin = os.path.join(fw_dir, "firmware.bin")
        soc, builder = build(cpu_firmware=fw_bin, build_name=args.name)
    else:
        soc, builder = build(cpu_firmware=None, build_name=args.name)

if __name__ == "__main__":
    main()
