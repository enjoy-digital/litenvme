# Build options {#build-options}

This appendix lists the options of every tool of the project: the parameters of the `LiteNVMe`
class, the command line and the YAML keys of the generator, the options of the bench target and
the build flags of the firmware. The chapters explain when to use them; this appendix gives their
names and defaults.

## `LiteNVMe` parameters {#opt-core}

The `LiteNVMe` class (`litenvme/core.py`) takes the root port and the keyword parameters below.
[Integration](#integration) explains how to choose them.

| Parameter | Default | Meaning |
|---------------------------------|----------------|---------------------------------------------|
| `pcie_endpoint` | required | `LitePCIeRootPort` whose crossbar the core uses |
| `hostmem_base` | `0x10000000` | Bus address of the host-memory window |
| `hostmem_size` | `0x80000` | Size of the window in bytes (512 KiB) |
| `data_width` | PHY data width | Width of the window and of the engine bridge: 64, 128 or 256 |
| `qid` | `1` | Identifier of the I/O queue |
| `qsize` | `64` | Entries of the I/O SQ and CQ rings |
| `qd` | `32` | Commands in flight at most, at most `qsize - 1` |
| `requester_id` | `0x0000` | Requester ID of the configuration accessor |
| `hostmem_backend` | `None` | External AXI backend of the window; `None` builds block RAM |
| `with_request_gen` | `False` | Build the request generator |
| `with_block_streamer` | `False` | Build the block streamer |
| `block_streamer_csr` | `True` | Registers of the block streamer; `False` leaves its signals to the SoC |
| `with_hostmem_csr` | `True` | Debug registers and counters of the window |
| `staging_base` | `0x40000` | Offset of the staging region in the window |
| `staging_size` | `0x40000` | Size of the staging region in bytes (256 KiB) |
| `with_csr` | `True` | Default of the three options below |
| `with_accessor_csr` | `with_csr` | Registers of the configuration and MMIO accessors |
| `with_engine_csr` | `with_csr` | Registers of the I/O engine |
| `with_request_gen_csr` | `with_csr` | Registers of the request generator |
| `extra_axi_masters` | `None` | More AXI masters on the window, such as the bridge of the init sequencer |

With one front-end, the front-end owns the request and completion streams of the engine; with
none or both, the core exposes them as `request` and `completion` for the SoC
([Front-ends](#frontends)). `add_csrs(soc, name="nvme")` registers the blocks that have registers
on the SoC under the prefix `name` ([Register reference](#registers)).

## Generator command line {#opt-gen-cli}

`litenvme_gen` (or `python3 -m litenvme.gen`) takes one YAML file. It writes the Verilog core, the
script that creates the hard IP, the constraints and the register maps under the output directory,
and never runs Vivado: the core is synthesized in the user's project ([Integration](#integration)).

| Option | Default | Effect |
|--------------------|----------------------|------------------------------------------------------|
| `config` | required | YAML file of the core |
| `--output-dir` | `build` | Output directory |
| `--name` | `litenvme_core` | Name of the generated module and files |

## Generator YAML keys {#opt-gen}

The keys below are read by `litenvme/gen.py`; `examples/alibaba_xcku3p.yml` is the example. An
unknown key is an error. The strings `"True"`, `"False"` and `"None"` are converted to their
Python values, and `clk_freq` to a float.

### PHY and clocking {#opt-gen-phy}

| Key | Default | Meaning |
|--------------------------------------|----------------------------|------------------------------|
| `phy` | required | PCIe PHY class; only `USPPCIEPHY` is supported |
| `phy_device` | required | FPGA part, as `xcku3p-ffvb676-2-e` |
| `phy_lanes` | `4` | Lanes of the PCIe pads |
| `phy_speed` | `gen3` | Link speed given to the PHY |
| `phy_ip_name` | `pcie4_uscale_plus` | Vivado IP of the hard block |
| `phy_config` | `{}` | Settings passed to the hard IP (`update_config`) |
| `data_width` | `256` | Datapath width: 64, 128 or 256 |
| `clk_freq` | `125e6` | System clock frequency in Hz |
| `clk_external` | `False` | `False`: the core drives `clk` from the PCIe user clock; `True`: the user drives `clk` and `rst` |
| `ep_max_pending_requests` | `8` | Outstanding read buffers of the root port |
| `ep_address_width` | `64` | Address width of the root port |

A `data_width` of 512 passes the check of the generator, but the host-memory window accepts 64,
128 and 256 only, so the elaboration then fails ([Known issues](#intro-issues)).

### Bring-up {#opt-gen-bringup}

| Key | Default | Meaning |
|----------------------------------------|------------------------|--------------------------------|
| `init_mode` | `firmware` | `firmware`: a soft CPU does the bring-up; `rtl`: the init sequencer does |
| `cpu` | `vexriscv`, `None` with `rtl` | Soft CPU; `firmware` needs one |
| `cpu_variant` | `minimal` | Variant of the CPU |
| `uart` | `serial` with a CPU, else `stub` | UART of the CPU |
| `firmware` | `none` | `none`: LiteX BIOS; `auto` or `minimal`: firmware in ROM; `rtl` needs `none` |
| `integrated_rom_size` | `0x20000`, `0x10000` with `none` | ROM of the CPU |
| `integrated_main_ram_size` | `0x10000` | RAM of the BIOS (`firmware` `none` only) |
| `bar0_base` | `0xe0000000` | Bus address the init sequencer gives BAR0 (`rtl`) |
| `root_mem_window_dw` | `0x20` | Dword address, on the cfg_mgmt port, of the root port register that receives the memory window (`rtl`); Memory Base and Limit is dword `0x08` ([Known issues](#intro-issues)) |
| `root_mem_window_val` | `0xe000e000` | Value written to it (`rtl`) |

With `firmware` set to `auto` or `minimal`, the generator builds the SoC once for the software
headers, runs `make -C bench/firmware BOOT=rom` (with `MINIMAL=1` for `minimal`), then builds again
with the firmware in ROM; this needs a RISC-V toolchain.

The firmware uses the register names of the bench SoC, which the generated core does not have: its
compilation fails and the generator stops, so these two modes cannot be generated ([Known
issues](#intro-issues)).

### Core {#opt-gen-core}

| Key | Default | Meaning |
|----------------------------------|--------------------------|------------------------------------|
| `hostmem_base` | `0x10000000` | Bus address of the host-memory window |
| `hostmem_size` | `0x80000` | Size of the window in bytes |
| `qid` | `1` | Identifier of the I/O queue |
| `qsize` | `64` | Entries of the I/O rings, also used by the init sequencer |
| `qd` | `32` | Commands in flight at most |
| `requester_id` | `0x0000` | Requester ID of the configuration accessor |
| `with_request_gen` | `True`, `False` with the block streamer | Request generator, driven by registers |
| `with_block_streamer` | `False` | Block streamer on the ports of the core |
| `with_hostmem_csr` | see below | Debug registers of the window |
| `staging_base` | `0x40000` | Offset of the staging region |
| `staging_size` | `0x40000` | Size of the staging region in bytes |

`with_request_gen` and `with_block_streamer` exclude each other. With the block streamer, its
command, status and data streams become ports of the core, and its start input is gated by
`status_init_done`. `with_hostmem_csr` defaults to on in `firmware` mode without the block
streamer, off otherwise.

## Bench target {#opt-bench}

`bench/alibaba_xcku3p.py` builds the bench SoC for the Alibaba XCKU3P
([Bench, firmware and host tools](#examples)). It takes the usual LiteX options (`--build`,
`--load`, toolchain options) and the options below. Its window is 512 KiB, with 64-entry queues.

| Option | Default | Effect |
|------------------------------------------|--------------------|----------------------------------|
| `--sys-clk-freq` | `125e6` | System clock frequency in Hz |
| `--with-etherbone` | off | Etherbone on the SFP cage (1000BASE-X) |
| `--eth-ip` | `192.168.1.50` | IP address of Etherbone |
| `--with-cpu` | off | VexRiscv soft CPU (`minimal` variant) for the firmware, with a crossover UART |
| `--cpu-boot` | `rom` | `rom`: firmware in ROM; `bios`: LiteX BIOS, firmware loaded over the UART |
| `--cpu-firmware` | `auto` | ROM content: `auto` builds `bench/firmware` in a two-pass build, or a file |
| `--litescope-probe` | `none` | `pcie`: LiteScope on the PCIe ports ([Debugging](#debugging)) |
| `--with-io-engine` | off | I/O engine, request generator and core control |
| `--io-engine-qd` | `32` | Queue depth of the I/O engine |
| `--with-block-streamer` | off | Block streamer and its test pattern unit (with `--with-io-engine`) |
| `--with-rtl-init` | off | Init sequencer instead of the firmware bring-up |
| `--ep-max-pending-requests` | `8` | Outstanding read buffers of the root port |

The harness scripts build with `--with-cpu --cpu-boot=bios --with-etherbone --with-io-engine`
(`bench/hw_common.sh`).

## Firmware build flags {#opt-firmware}

The firmware is built with `make -C bench/firmware`, against the headers of a LiteX build.

| Variable | Default | Effect |
|----------------|--------------------------------------|------------------------------------------|
| `BUILD_DIR` | `../build/alibaba_xcku3p` | Build directory of the SoC (software headers) |
| `BOOT` | `bios` | `bios`: linked to run from main RAM; `rom`: linked for the ROM |
| `MINIMAL` | unset | `1`: bring-up then idle, no console and no prints (`CONFIG_LITENVME_MINIMAL`) |

The commands of the firmware depend on the registers of the SoC: `nvme_engine_bench` needs the
request generator and `nvme_setup` the I/O engine. [Bench, firmware and host tools](#examples)
lists them.
