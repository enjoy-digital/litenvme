# Bench, firmware and host tools {#examples}

This chapter describes the reference design of the repository and the tools around it: the bench SoC
of `bench/`, the firmware of its soft CPU, the bench PC tools, and the harness scripts that chain
them. It ends with the recorded runs of `bench/results`.

Use them to check a board, as a starting point for your own design, or to reproduce a result.

## At a glance {#ex-glance}

| Item | Path | Role |
|--------------|----------------------------------------------|--------------------------------|
| Bench SoC | `bench/alibaba_xcku3p.py` | Target: root port, core blocks, CPU, Etherbone |
| Firmware | `bench/firmware/` | Bring-up and test console, `litenvme>` prompt |
| Bench PC tools | `bench/*.py` | Etherbone tests, console driver, window access |
| Harness scripts | `bench/hw_*.sh` | Load, boot, run and log one test sequence |
| Recorded runs | `bench/results/` | Logs printed by the board |
| Generator example | `examples/alibaba_xcku3p.yml` | Standalone core, [Standalone core](#integ-standalone) |

## Boards {#ex-boards}

The repository has one target, for the Alibaba XCKU3P; no LiteX-Boards target uses LiteNVMe. The
platform file is the one of LiteX-Boards, `alibaba_xcku3p`, whose `pcie_x4` resource carries the
lanes, the reference clock and PERST#. The [status table](#intro-status) gives the status of each
configuration.

| Board | FPGA | PHY | Link | Width |
|----------------------------|------------|-----------------|----------|------------|
| Alibaba XCKU3P | XCKU3P | `USPPCIEPHY` | Gen3 x4 | 256 bits |
| Alibaba XCKU3P, earlier runs | XCKU3P | `USPPCIEPHY` | Gen2 x4 | 128 bits |
| A Xilinx 7-Series board | 7-Series | `S7PCIEPHY` | Gen2 x4 | unknown |

The 7-Series row comes from the description of the commit that added the byte count of the
completions of the host-memory window. The repository holds no target and no log for it.

## Bench SoC {#ex-soc}

The bench SoC does not use the `LiteNVMe` class: it builds the accessors, the host-memory window,
the I/O engine and the front-ends one by one, so that each can be probed and selected.

Its system clock is 125 MHz, from an MMCM on the 100 MHz oscillator; its window is 512 KiB at
`0x10000000`, and its I/O queue has 64 entries.

[Bench target](#opt-bench) lists the options of the target with their defaults. Some of them need
a word more:

- With `--with-cpu` and `--cpu-firmware auto`, the target elaborates the SoC twice and runs `make`
  on `bench/firmware` in between, even for `--load` alone; with `--build`, the second pass runs
  Vivado. A failed `make` stops the target, and `--no-compile` (or `--no-compile-gateware`)
  skips Vivado in both passes.
- `--with-block-streamer` adds a test block on the streamer data ports, `_BlockBIST`. It writes a
  counter, one value per beat, and checks it on the read path. A register (`nvme_block_bist_sel`)
  selects the front-end of the I/O engine: 0 for the request generator, 1 for the block streamer.
- `--with-rtl-init` removes the registers of the accessors: the init sequencer drives them, and a
  bench PC script starts it through `nvme_init_ctrl`.

A smaller `--ep-max-pending-requests` saves block RAM in the root port and lowers the read
throughput. The harness scripts build with `--with-cpu --cpu-boot=bios --with-etherbone
--with-io-engine`, plus the option of the test they run.

The bench SoC registers its blocks both as attributes and with `add_module()`, so some register
banks exist twice: `cfg` and `pcie_cfg`, `mmio` and `pcie_mmio`, `io_engine` and `nvme_engine`.
The firmware and the bench PC tools use the names of the first bank of each pair, plus
`nvme_engine`, `nvme_gen`, `hostmem` and `pcie_rootcfg`.

## Firmware {#ex-firmware}

The firmware, `bench/firmware/main.c`, runs on the VexRiscv of the bench SoC. It brings the SSD
up through the accessors, drives the admin and I/O queues itself for single commands, and programs
the I/O engine for benchmarks. Its console is the crossover UART, reached from the bench PC with
`litex_term crossover`.

| Command | Action |
|--------------------------|--------------------------------------------------------------|
| `help` | List the commands |
| `reboot` | Reboot the CPU |
| `status` | PCIe link and window counters |
| `cmd_enable` | Set MEM and BME of the SSD |
| `cmd_disable` | Clear MEM and BME |
| `nvme_reset` | Clear the cached state, disable the controller |
| `nvme_fill` | Set the fill pattern used by `nvme_write`, `nvme_verify` and the benchmark |
| `nvme_identify` | Assign BAR0, set MEM and BME, send Identify Controller |
| `nvme_read` | Read sectors into the window and print the first dwords |
| `nvme_write` | Write sectors from the window, with the fill pattern |
| `nvme_verify` | Write the fill pattern to sectors, read them back and compare |
| `nvme_engine_bench` | Benchmark the I/O engine with the request generator |
| `nvme_setup` | Bring-up, engine enabled, `init_done` set |

The arguments are optional, except the pattern of `nvme_fill`, and take the defaults of the
bench: BAR0 at `0xe0000000`, namespace 1; `nvme_engine_bench` reads by default.

```console
nvme_fill <pattern>
nvme_identify [bar0] [cid]
nvme_read [bar0] [nsid] [slba] [nlb]
nvme_write [bar0] [nsid] [slba] [nlb]
nvme_verify [bar0] [nsid] [slba] [nlb] [dwords]
nvme_engine_bench [read|write] [bar0] [nsid] [slba] [nlb] [count] [step]
nvme_setup [bar0]
```

`nvme_engine_bench` needs the request generator and `nvme_setup` the I/O engine; the firmware
leaves them out of a SoC without them. `nvme_engine_bench` accepts 1 to 16 sectors per command,
so each command fits two PRP entries.

The single commands poll their CQE in the firmware, which reads the phase dword first and the
other dwords once the phase matches, as the I/O engine does.

The Makefile takes the build directory of the SoC (`BUILD_DIR`), the boot mode (`BOOT=bios` or
`BOOT=rom`) and `MINIMAL=1`. A minimal build removes the console and every print: it runs the
bring-up of `nvme_setup` once, then idles. The generator uses it for `firmware: minimal`.

## Bench PC tools {#ex-host}

The tools of `bench/` run on the bench PC and reach the board through one `litex_server` over
Etherbone. Most of them take the register map from `--csr-csv`; their defaults are `csr.csv` or
`build/alibaba_xcku3p/csr.csv`.

| Tool | Use |
|------------------------------|------------------------------------------------------------|
| `engine_console.py` | Upload the firmware, then send one console command and capture its reply |
| `test_cfg.py` | Configuration reads and writes of the SSD through the accessor |
| `test_bar0.py` | MMIO reads, writes and dumps of BAR0 |
| `test_nvme.py` | Step-by-step NVMe: registers, Identify, I/O queues, read, write |
| `nvme_io.py` | Read or write sectors, with an optional full setup |
| `nvme_host.py` | Library of these two: command builders and an NVMe host class |
| `hostmem_tool.py` | Dump, fill and compare the window through its debug port |
| `test_block.py` | Request generator throughput, then block streamer BIST |
| `test_rtlinit.py` | Start the init sequencer, then read through the request generator |
| `test_ltssm_tracer.py` | History of LTSSM states, for PHYs with an LTSSM tracer |
| `qd_sweep.py` | Queue-depth sweep over a console command |

`engine_console.py` exists because of the crossover UART. `litex_term crossover` keeps reading
the receive FIFO of the UART for as long as it runs. A second reader, such as a script, then gets
only part of the bytes. The harness therefore uploads the firmware with `litex_term`, stops it,
and leaves `engine_console.py` as the only reader.

`qd_sweep.py` sends the `nvme_bench` command, which the firmware no longer has; its recorded
results come from an earlier firmware ([Known issues](#intro-issues)).

## Harness scripts {#ex-harness}

The harness scripts chain the steps of [Getting started](#getting-started) and write a log. They
run from `bench/` and share the functions of `hw_common.sh`.

| Function | Action |
|-----------------------|----------------------------------------------------------------|
| `hw_load [probe]` | Stop old servers, load the bitstream, optional LiteScope probe |
| `hw_serve` | Start one `litex_server` over UDP |
| `hw_link_decode` | Print rate, width, LTSSM state, MPS and MRRS of the root port |
| `hw_boot` | Upload the firmware with `litex_term`, then stop it |
| `hw_cmd <name> <s> <cmd>` | Run one console command, wait, append the reply to the log |
| `hw_stop` | Stop the server |

| Script | Build option added | Sequence |
|----------------------------------|----------------------------------|--------------------------|
| `hw_gen3.sh` | none | Link, boot, 8 KiB and 4 KiB engine benchmarks |
| `hw_integrity.sh` | none | Boot, fill a pattern, write and verify an LBA range |
| `hw_block.sh` | `--with-block-streamer` | Boot, `nvme_setup`, then `test_block.py` |
| `hw_rtlinit.sh` | `--with-rtl-init`, no CPU | Link, then `test_rtlinit.py` |
| `run_engine_session.sh` | none | Boot only, for interactive use |

Each script expects a bitstream built beforehand with its options; it loads it, but does not
build it. The logs go to `/tmp`, one file per script. Run one script at a time: each one stops the
servers and terminals left by another.

## Recorded runs {#ex-results}

`bench/results/gen3_256b_2026-06-01/` holds the runs of the Gen3 x4, 256-bit design, printed by
the board through the harness. The SSD answers with vendor and device ID `0x5427c0a9`. The
benchmark logs are quoted in [Getting started](#gs-firmware) and their figures in
[Performance](#performance); this section quotes the two other logs.

| File | Content |
|----------------------------------------|--------------------------------------------------|
| `throughput_8kib_default.log` | `hw_gen3.sh` (plus `rootmps`): 8 KiB and 4 KiB reads, 8 KiB writes, QD 32 |
| `throughput.log` | 4 KiB reads and writes, and an 8 KiB read |
| `nvme_init.log` | BAR0 discovery, MMIO self-test, one read by the firmware |
| `integrity.log` | Pattern written, read back and verified |
| `SUMMARY.md` | Table of the throughput runs |

These runs used a firmware build with more commands than the one in the repository:
`nvme_mmiotest`, `nvme_debug`, `nvme_write_readback` and `rootmps` appear in the logs but are not
in `bench/firmware/main.c`. The harness lines that name capture files are cut from the captures
below.

### BAR0 and MMIO self-test {#ex-mmiotest}

The capture below is from `nvme_init.log`. The firmware assigns BAR0, sets MEM and BME, raises
the MPS, opens the memory window of the root port, then reads and writes controller registers.

```console
[22:59:50] CMD mmiot: nvme_mmiotest
[...]
nvme_mmiotest
VID/DID      = 0x5427c0a9
BAR0 orig    = 0x00000004 (64b=1)
BAR0 size    = 0x4000
BAR0/BAR1    = 0xe0000004 / 0x00000000
MPS auto-set: enc=2 (512B)  [root_sup=3 ssd_sup=2]
bar0_base=0xe0000000  CMD/STS=0x00100006 (MEM=1 BME=1)
root MemBase/Limit(0x20)=0xe000e000
MMIO RD: cap=0x0000000000000000(e0) csts=0x00000000(e0) cc=0x00000000(e0)
MMIO WR32: AQA<=0x003f003f(e0) readback=0x003f003f(e0) MATCH
MMIO WR64: ASQ<=0x12340000 (e0) readback=0x0000000012340000 (e0/0)
MMIO post-write CSTS: 0x00000000 (e0) ok
litenvme>
```

`BAR0 orig` is the value read before the assignment: bit 2 set marks a 64-bit memory BAR, so
BAR1 holds its upper half. Writing all ones and reading back gives the size, 16 KiB.

`CMD/STS` shows MEM and BME set, and the root port forwards `0xe0000000` to `0xe00fffff` to the
SSD. The `(e0)` after each value is the error flag of the MMIO accessor, 0 here.

In this run, the CAP read returned 0 while AQA and ASQ read back what was written; the firmware
reads CAP several times for this reason.

### Data integrity {#ex-integrity}

The capture below is from `integrity.log`. The firmware fills its write buffer with a pattern,
writes 8 sectors, reads them back, then compares the first 128 dwords of each sector. The bytes of
the pattern are distinct, so a byte swap inside a dword would show; the pattern is the same in
every dword, so a dword swap would not.

```console
[23:06:01] CMD fill: nvme_fill 0xDEADBEEF
[...]
nvme_fill 0xDEADBEEF
nvme_fill = 0xdeadbeef
litenvme>
[...]
nvme_write_readback 0xe0000000 1 5600000 8 16
[...]
Write completed. Readback:
Read data (first 16 dwords):
  deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef
  deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef deadbeef
litenvme>
[...]
nvme_verify 0xe0000000 1 5600000 8 128
VERIFY OK: LBA 5600000 .. 5600007 (pattern 0xdeadbeef)
litenvme>
```

`hw_integrity.sh` runs `nvme_fill` then `nvme_verify`: the verify command writes the pattern to
each sector, one Write command per sector, then reads the sectors back and compares the first
`dwords` of each. `nvme_write` writes the fill pattern too; its default, `0xa5a5a5a5`, is the
data that the read of `nvme_init.log` at sector 0 returns.

### Earlier runs {#ex-archive}

`bench/results/archive/` holds the runs of the Gen2 x4, 128-bit design and the steps that led to
the I/O engine. `bench/qd_sweep_result.txt` is a firmware queue-depth sweep at 4 KiB on that
design. [Performance](#performance) summarizes them.

## LiteScope probe {#ex-litescope}

`--litescope-probe pcie` adds a LiteScope analyzer on the PCIe ports, described in
[LiteScope](#debug-litescope). The harness loads such a bitstream with `hw_load pcie`.

## Where to go next {#ex-next}

[Integration](#integration) adapts these designs to your board. [Debugging](#debugging) starts
from the registers that these tools read.
