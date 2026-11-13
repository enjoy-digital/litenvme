#
# This file is part of LiteNVMe project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register appendix of the manual: which designs tools/csr_doc.py elaborates and which register banks
# it documents (the engine is generic, part of the manual kit).

import copy
import importlib.util
import json
import os
import sys
import tempfile

# Configurations: script (path from the repository root), command line and YAML keys changed on the
# generator example. The example of the repository makes the block streamer the sole front-end, driven
# by pins, which removes the request generator and the host-memory debug registers; the benchmark
# variant below (request generator instead of the block streamer, the variant named in the example's
# comments) has every register bank the generator can build. The firmware-mode CPU is elaborated but
# no software is compiled, so no RISC-V toolchain is needed.
CONFIGS = {
    "xcku3p_request_gen" : ("litenvme/gen.py", ["examples/alibaba_xcku3p.yml"],
        # No firmware: the registers do not depend on it, and its build needs the LiteX software.
        {"with_block_streamer": False, "with_request_gen": True, "firmware": "none"}),
}

# Banks documented in the appendix (prefix match on the bank name), in order, with their title and
# the Python class that defines them. The longest matching prefix owns a bank.
BANKS = [
    ("nvme_cfg",         "Configuration accessor",  "LiteNVMePCIeCfgAccessor",  "litenvme/cfg.py"),
    ("nvme_mmio",        "MMIO accessor",           "LiteNVMePCIeMmioAccessor", "litenvme/mem.py"),
    ("nvme_io_engine",   "I/O engine",              "LiteNVMeIOEngine",         "litenvme/io_engine.py"),
    ("nvme_hostmem",     "Host-memory window",      "LiteNVMeHostMemCSR",       "litenvme/hostmem.py"),
    ("nvme_request_gen", "Request generator",       "LiteNVMeRequestGen",       "litenvme/request_gen.py"),
    ("nvme_ctrl",        "Core control",            "LiteNVMeCoreControl",      "litenvme/core.py"),
    ("pcie_rootcfg",     "Root port configuration", "LiteNVMeRootCfgMgmt",      "litenvme/cfg.py"),
    ("pcie_phy",         "PCIe PHY (LitePCIe)",     "USPPCIEPHY",               "litepcie/phy/usppciephy.py"),
]

# Descriptions of the registers and fields that the gateware declares without one. A description
# declared in the gateware takes precedence. Keys: (bank, register) or (bank, register, field).
DESCRIPTIONS = {
    ("nvme_cfg", "cfg_ctrl", "start")         : "Write 1: send the configuration request (pulse).",
    ("nvme_cfg", "cfg_ctrl", "we")            : "0: CfgRd0, 1: CfgWr0.",
    ("nvme_mmio", "mem_ctrl", "start")        : "Write 1: send the memory request (pulse).",
    ("nvme_mmio", "mem_ctrl", "we")           : "0: MemRd, 1: MemWr (posted).",
    ("nvme_mmio", "mem_ctrl", "wsel")         : "Byte enables of a write; only `0xf` is accepted, another value ends the request with `err`.",
    ("nvme_io_engine", "engine_status", "busy")     : "1: the engine FSM is not idle.",
    ("nvme_io_engine", "engine_status", "inflight") : "Commands submitted and not yet reaped.",
    ("nvme_ctrl", "init_done")                : "Written to 1 by the bring-up firmware when the controller and the I/O queue are ready; drives `status_init_done` of the standalone core.",
    ("nvme_ctrl", "init_error")               : "Written to 1 by the bring-up firmware when the bring-up failed; drives `status_init_error`.",
    ("pcie_phy", "link_status", "status")     : "1: link up.",
    ("pcie_phy", "link_status", "phy_down")   : "1: PHY link down.",
    ("pcie_phy", "link_status", "phy_status") : "0: no receiver detected, 1: training, 2: link up and data link initialization in progress, 3: link up, data link initialized.",
    ("pcie_phy", "link_status", "rate")       : "0: 2.5 GT/s (Gen1), 1: 5.0 GT/s (Gen2), 2: 8.0 GT/s (Gen3).",
    ("pcie_phy", "link_status", "width")      : "0: x1, 1: x2, 2: x4, 3: x8, 4: x16.",
}

# Notes printed after the title of a bank family: what the register descriptions do not say.
NOTES = {
    "nvme_cfg": [
        "`LiteNVMe` collects its sub-blocks into one `nvme` bank, which this appendix splits per",
        "sub-block, keeping the offsets from the base of `nvme` (full names such as",
        "`nvme_cfg_cfg_ctrl`). The bank names given in this section and the next four",
        "(`nvme_cfg`, `nvme_mmio`, `nvme_io_engine`, `nvme_hostmem`, `nvme_request_gen`) are",
        "labels of these groups, not banks of `csr.csv`: add the offsets to the base of `nvme`.",
        "",
        "Software sets `cfg_bdf` and, for a write, `cfg_wdata`, then writes `start` and polls",
        "`cfg_stat` until `done` or `err` is set ([Configuration and register access](#pcie-access)).",
        "The register reached is at byte `(ext << 6 | reg) * 4`, within the first 2 KiB of the",
        "configuration space.",
    ],
    "nvme_mmio": [
        "Registers of bank `nvme`, offsets from its base (`nvme_mmio_mem_*`).",
        "",
        "The MMIO accessor reads and writes 32-bit registers of BAR0: software sets the 64-bit",
        "byte address in `mem_adr_h` and `mem_adr_l`, `len` at 1 and, for a write, `mem_wdata`,",
        "then writes `start` and polls `mem_stat`. Writes are posted. The I/O engine rings its",
        "doorbells through a second MMIO accessor that has no registers.",
    ],
    "nvme_io_engine": [
        "Registers of bank `nvme`, offsets from its base (`nvme_io_engine_*`).",
        "",
        "Software programs the queue bases (host-memory window addresses), the doorbell",
        "addresses (BAR0 byte addresses) and the PRP-list region, then sets `enable`",
        "([The I/O engine](#io-engine)).",
    ],
    "nvme_hostmem": [
        "Registers of bank `nvme`, offsets from its base (`nvme_hostmem_csr_*`).",
        "",
        "Present with `with_hostmem_csr`, which the generator turns on in `firmware` mode and off",
        "in `rtl` mode. `csr_adr` is a dword index from the base of the window; the",
        "read path uses a third port of the window memory. `csr_we` is a level: the firmware",
        "writes 1 then 0, and the write repeats while it stays at 1 (one dword written with",
        "byte enables each time).",
        "",
        "The counters observe the PCIe side",
        "of the window. `rd_gap_max` counts from reset, so it is meaningful only after the",
        "first beat ([The host-memory window](#host-memory)).",
    ],
    "nvme_request_gen": [
        "Registers of bank `nvme`, offsets from its base (`nvme_request_gen_*`).",
        "",
        "Present with `with_request_gen`. The generator issues `count` commands at",
        "`base_lba + k * lba_step`, with the buffer `buf_base + (k % qmod) * buf_stride`",
        "([Front-ends](#frontends)). The block streamer also has a register wrapper",
        "(`sector`, `count`, `nsid`, `ctrl`, `status`, `cycles`, in `litenvme/block.py`), which",
        "the generator does not build: it exposes the block streamer on pins.",
    ],
    "nvme_ctrl": [
        "Present with `init_mode` `firmware`. With `rtl`, the init sequencer drives the status",
        "pins and this bank does not exist ([Bring-up](#bringup)).",
    ],
    "pcie_rootcfg": [
        "Reaches the configuration space of the root port itself through the `cfg_mgmt` port of",
        "the hard IP (dword address), with a handshake to the `pcie` clock domain. The firmware",
        "uses it to raise the Max Payload Size of the root port and to program its memory",
        "window. Present with `init_mode` `firmware`.",
    ],
    "pcie_phy": [
        "Registers of the LitePCIe PHY, documented in the LitePCIe manual. `link_status` is the",
        "first register to read on a new bench ([Debugging](#debugging)).",
    ],
}

# Opening paragraph of the appendix and the sentence that ends the access paragraph.
INTRO = ["This appendix lists the registers of LiteNVMe: the configuration and MMIO accessors, the",
         "I/O engine, the host-memory window, the request generator, the core control and the",
         "root port configuration, with the registers of the LitePCIe PHY they work with. It is",
         "generated from the register definitions of the gateware, elaborated by the standalone",
         "generator `litenvme/gen.py` on its example `examples/alibaba_xcku3p.yml` in its",
         "benchmark variant, without running an FPGA toolchain or compiling the firmware.",
         "",
         "Offsets are in bytes from the base of the bank; the base addresses depend on the SoC",
         "and are in the `csr.csv` of a build. The bench SoC `bench/alibaba_xcku3p.py` wires",
         "the blocks itself and gives them other names (`cfg_cfg_*`, `mmio_mem_*`,",
         "`nvme_engine_engine_*`, `hostmem_csr_*`, `nvme_gen_*`), with the same registers in",
         "each block."]
USAGE = ["The firmware and the host scripts of `bench/` use these names through the generated",
         "`csr.h` and `csr.csv`, so they never need the offsets."]
MAIN_BANK = "Registers of the top-level `main` bank of the SoC."

# The `nvme` bank holds every sub-block of `LiteNVMe`: it is split into banks named after the
# sub-block, with their offsets kept from the base of `nvme`.
SPLITS = {"nvme": ["cfg", "mmio", "io_engine", "hostmem", "request_gen"]}


def _split_banks(soc, csr_json):
    for bank, subs in SPLITS.items():
        region = soc.csr.regions.get(bank)
        if region is None:
            continue
        keep = list(region.obj)
        for sub in subs:
            regs = []
            for csr in [c for c in keep if c.name.startswith(sub + "_")]:
                keep.remove(csr)
                c = copy.copy(csr)
                c.name = csr.name[len(sub) + 1:]
                regs.append(c)
            if regs:
                name = f"{bank}_{sub}"
                soc.csr.regions[name] = copy.copy(region)
                soc.csr.regions[name].obj = regs
                csr_json["csr_bases"][name] = csr_json["csr_bases"][bank]
        region.obj = keep
        if not keep:
            del soc.csr.regions[bank]


def _describe(soc):
    """Attach the descriptions of DESCRIPTIONS to the registers that have none in the gateware."""
    for bank, region in soc.csr.regions.items():
        for csr in region.obj:
            desc = DESCRIPTIONS.get((bank, csr.name))
            if desc and not (getattr(csr, "description", None) or "").strip():
                csr.description = desc
            fields = getattr(csr, "fields", None)
            for f in (fields.fields if fields else []):
                desc = DESCRIPTIONS.get((bank, csr.name, f.name))
                if desc and not (f.description or "").strip():
                    f.description = desc


def _load(repo_root, script):
    """Import a script of the repository under a unique module name."""
    name = "manual_" + script.replace("/", "_").replace(".py", "")
    spec = importlib.util.spec_from_file_location(name, os.path.join(repo_root, script))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def elaborate(name, repo_root):
    """Run the main() of the generator with its Builder replaced by one that elaborates without the
    toolchain; return (soc, csr_json, module, argv)."""
    import yaml
    script, argv, overrides = CONFIGS[name]
    sys.path.insert(0, repo_root)
    from litex.soc.integration.builder import Builder
    captured = {}
    old_cwd  = os.getcwd()

    with tempfile.TemporaryDirectory() as tmp:
        class CaptureBuilder(Builder):
            def __init__(self, soc, *args, **kwargs):
                kwargs.update(output_dir=tmp, csr_csv=os.path.join(tmp, "csr.csv"),
                    csr_json=os.path.join(tmp, "csr.json"), compile_software=False,
                    compile_gateware=False)
                Builder.__init__(self, soc, **kwargs)
                captured["soc"] = soc

            def build(self, *args, **kwargs):
                kwargs["run"] = False
                return Builder.build(self, *args, **kwargs)

        run_argv = list(argv)
        if overrides:
            # Variant of a generator example: the example with some keys changed, in a temporary file.
            config = yaml.safe_load(open(os.path.join(repo_root, argv[0])).read())
            config.update(overrides)
            path = os.path.join(tmp, os.path.basename(argv[0]))
            with open(path, "w") as f:
                yaml.safe_dump(config, f)
            run_argv = [path] + argv[1:]

        os.chdir(repo_root)
        mod = _load(repo_root, script)
        mod.Builder = CaptureBuilder
        old_argv = sys.argv
        sys.argv = [script] + run_argv + ["--output-dir", tmp]
        try:
            mod.main()
        finally:
            sys.argv = old_argv
            os.chdir(old_cwd)
        captured["json"] = json.load(open(os.path.join(tmp, "csr.json")))

    # Memories mapped on the CSR bus have no register definitions: keep the register banks only.
    soc, csr_json = captured["soc"], captured["json"]
    for bank, region in list(soc.csr.regions.items()):
        if not isinstance(region.obj, list):
            del soc.csr.regions[bank]
    _split_banks(soc, csr_json)
    _describe(soc)
    # Generator examples are shown by their file name (all in examples/), to fit the table.
    shown = [os.path.basename(a) for a in argv] + [f"{k}={v}" for k, v in overrides.items()]
    return soc, csr_json, "litenvme_gen", shown
