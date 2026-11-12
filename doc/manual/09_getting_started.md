```{=latex}
\part{Use it}
```

```{=html}
<h1 class="part">Part III. Use it</h1>
```

# Getting started {#getting-started}

This chapter takes you from a fresh machine to an SSD that answers commands from FPGA logic. It
starts without hardware: the unit tests, then a standalone core generated without any vendor tool.

It then builds the bench SoC for the Alibaba XCKU3P, boots its firmware and runs a first benchmark.
Each step gives the commands, what you should see, and what to check when you do not.

## Prerequisites {#gs-install}

LiteNVMe is a Python package built on LiteX and LitePCIe; it needs Python 3.8 or newer. Install
LiteX and its cores by following the installation page of the LiteX wiki (`litex_setup.py`).
LiteNVMe is not among the repositories that `litex_setup.py` clones, so clone it and install it in
development mode:

```sh
git clone https://github.com/enjoy-digital/litenvme
cd litenvme
pip3 install -e .
```

The installation adds the `litenvme_gen` command. When it is not on the `PATH`,
`python3 -m litenvme.gen` runs the same generator. The other tools depend on the step, as the
table below shows.

| Task | Tools |
|------------------------------------|------------------------------------------------------------|
| Unit tests | Python 3, pytest |
| Standalone core, `init_mode: rtl` | LiteX, LitePCIe, PyYAML |
| Standalone core with a CPU | The above, plus the RISC-V GCC toolchain for the BIOS |
| Bench SoC | LiteX-Boards, LiteEth, LiteScope, RISC-V GCC, Vivado |
| Loading the board | openFPGALoader and a Digilent HS2 JTAG cable |
| Bench PC tools | `litex_server`, `litex_term`, a 1000BASE-X link to the SFP cage |

## Step 1: run the tests {#gs-tests}

The unit tests check the accessors, the host-memory window, the I/O engine, the front-ends, the
init sequencer and the generator configuration in Migen simulation, with no board and no FPGA
tool. The selection below is the fastest part: the accessors, the generator configuration and the
host-memory window.

```sh
python3 -m pytest -q test/test_cfg_accessor.py test/test_mem_accessor.py \
    test/test_gen_config.py test/test_hostmem.py
```

**What you should see.** All selected tests pass. The capture below is a run on the repository as
this manual describes it.

```console
.....................                                                    [100%]
21 passed in 4.11s
```

`python3 -m pytest -q test` runs the whole suite of 50 tests in one process. Its run for this
manual ended with `50 passed in 530.21s (0:08:50)`, most of it in the I/O engine and block
streamer simulations; [Simulation and
verification](#simulation) describes each test file.

**If not.** An import error names the missing package: `litex` or `litepcie` for the LiteX
installation, `yaml` for PyYAML. The suite turns off the assertion rewriting of pytest in
`pytest.ini`, because it confuses the signal naming of Migen; keep that setting.

## Step 2: generate a standalone core {#gs-generate}

The generator turns a YAML configuration into a Verilog core: PCIe PHY, root port and LiteNVMe in
one module. It never runs Vivado. With `init_mode: rtl`, the core has no CPU, so no software is
compiled either and the step needs no toolchain at all.

The example configuration, `examples/alibaba_xcku3p.yml`, uses a CPU for bring-up. The commands
below derive a CPU-less copy of it, as the continuous integration of the repository does, then
generate the core.

```sh
python3 - <<'EOF'
from pathlib import Path
import yaml
from litenvme.gen import load_config
cfg = load_config("examples/alibaba_xcku3p.yml")
cfg.update(init_mode="rtl", cpu=None, uart="stub", firmware="none")
Path("build").mkdir(exist_ok=True)
Path("build/litenvme_rtl.yml").write_text(yaml.safe_dump(cfg))
EOF
python3 -m litenvme.gen build/litenvme_rtl.yml --output-dir build/litenvme_rtl
ls build/litenvme_rtl/gateware
```

**What you should see.** The generator runs in a few seconds. The LiteX log names the device and
the system clock, then the register banks; the last command lists the generated files. The
capture below is trimmed with `[...]`.

```console
INFO:SoC:FPGA device : xcku3p-ffvb676-2-e.
INFO:SoC:System clock: 125.000MHz.
[...]
INFO:SoCCSRHandler:ctrl CSR allocated at Location 0.
INFO:SoCCSRHandler:identifier_mem CSR allocated at Location 1.
INFO:SoCCSRHandler:pcie_phy CSR allocated at Location 2.
INFO:SoCCSRHandler:timer0 CSR allocated at Location 3.
INFO:SoCCSRHandler:uart CSR allocated at Location 4.
[...]
build_litenvme_core.sh
litenvme_core_mem_1.init
litenvme_core_sram.init
litenvme_core.tcl
litenvme_core.v
litenvme_core.xdc
```

In this mode the register banks hold no NVMe register: the init sequencer drives the core
directly. `litenvme_core.v` holds one module, `litenvme_core`, with the PCIe pins, the clock and
reset, two status pins and the block interface. The hard IP is a black box named `pcie_usp`, which
`litenvme_core.tcl` creates. [Standalone core](#integ-standalone) describes the ports.

**If not.** `Unknown LiteNVMe config key` or `init_mode='rtl' does not use firmware` come from
the checks of the generator on the YAML file. The example configuration itself, in firmware mode,
also compiles the LiteX BIOS and fails without the RISC-V GCC toolchain.

## Step 3: build the bench SoC {#gs-build}

The bench SoC runs on the Alibaba XCKU3P, a Kintex UltraScale+ card, as a PCIe Gen3 x4 root port
with a 256-bit datapath. The SSD sits on an M.2 adapter: the card plugs into the PCIe slot of the
adapter, which takes its power from its own DC jack. The [introduction](#introduction) shows this
bench.

The bench PC reaches the board through Etherbone over the first SFP cage (1000BASE-X) and through
JTAG.

The command below builds the SoC that the harness scripts of `bench/` use: a VexRiscv CPU booted
by the LiteX BIOS, Etherbone, and the I/O engine with the request generator.

```sh
cd bench
./alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone \
    --with-io-engine --csr-csv=csr.csv --build
```

**What you should see.** The target builds in two passes. The first writes the software headers,
then runs `make` on the firmware of `bench/firmware`; the second pass runs Vivado. The bitstream
is `build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit`, and `csr.csv` lists the registers that
the bench PC tools use. `make` compiles the firmware of `bench/firmware` into
`firmware/firmware.bin`, which step 5 uploads; if it fails, the target stops before the second
pass.

**If not.** A firmware compilation error comes from the RISC-V toolchain or from register names
missing in the generated headers. Vivado reports its own errors in the logs of
`build/alibaba_xcku3p/gateware`. [Bench SoC](#ex-soc) lists the other options of the target.

## Step 4: load the board and check the link {#gs-load}

Power the M.2 adapter first, then load the bitstream over JTAG: `--load`, with the options of the
build, calls openFPGALoader. The design holds PERST# of the SSD low for 100 ms after
configuration, then releases it, and the link trains on its own. Then start one `litex_server`
for Etherbone; the IP address of the board is `192.168.1.50`.

```sh
./alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone \
    --with-io-engine --load
litex_server --udp --udp-ip 192.168.1.50
```

`litex_cli` reads the registers of the board by name through the server: the first command
below checks the connection, the second reads the link status register of the PHY,
`pcie_phy_link_status`.

```sh
litex_cli --csr-csv csr.csv --ident
litex_cli --csr-csv csr.csv --regs --filter link_status
```

The harness decodes the same register from the bench PC (`hw_link_decode` in
`bench/hw_common.sh`). The lines below are its output in a recorded run of the repository,
`bench/results/gen3_256b_2026-06-01/throughput_8kib_default.log`.

```console
[01:00:12] link decode:
  link_status=0x20ad  up=1  rate=Gen3  width=x4  ltssm=0x10
  RC MPS=128B MRRS=512B
```

**What you should see.** `up=1`, with the rate and the width of the link (bit 0 of
`link_status` at 1 with `litex_cli`). The MPS of the root port is 128 bytes at this point; the
firmware raises it later on both ends.

**If not.** `up=0` points to PERST#, the reference clock or the power of the adapter;
[Debugging](#debugging) goes through them. An Etherbone timeout means that the bench PC has no
link on the SFP cage or no address on the subnet of the board.

## Step 5: boot the firmware and run a benchmark {#gs-firmware}

The firmware runs on the CPU of the bench SoC and talks to the bench PC through a crossover UART,
a UART carried by Etherbone registers. `litex_term` uploads the firmware through the BIOS, then
gives its console, with the `litenvme>` prompt.

```sh
litex_term crossover --csr-csv csr.csv --kernel firmware/firmware.bin
```

The captures below come from a recorded run of the repository.

At the prompt, `nvme_identify` assigns BAR0, sets the MEM and BME bits of the SSD, and sends the
Identify Controller command. It prints the vendor ID, the model, the firmware revision, the
specification version and MDTS of the SSD. The figure below shows its decoded output in an
earlier session of the bench, with a Micron 2200S SSD.

![`nvme_identify` on the firmware console: completion status and decoded Identify Controller
fields of a Micron 2200S SSD (earlier session).](litenvme_identify.png){width=75%}

```console
nvme_identify 0xe0000000
nvme_engine_bench read 0xe0000000 1 1000000 16 1000 16
```

`nvme_engine_bench` creates the I/O queues, programs the I/O engine, then lets the request
generator issue 1000 reads of 16 sectors (8 KiB) each. The capture below is the first command of
`bench/hw_gen3.sh` in the recorded run above; the harness lines around the reply are cut.

```console
[01:00:24] CMD rd8k_a: nvme_engine_bench read  0xe0000000 1 1000000 16 1000 16
[...]
nvme_engine_bench read  0xe0000000 1 1000000 16 1000 16
VID/DID      = 0x5427c0a9
BAR0 orig    = 0x00000004 (64b=1)
BAR0 size    = 0x4000
BAR0/BAR1    = 0xe0000004 / 0x00000000
MPS auto-set: enc=2 (512B)  [root_sup=3 ssd_sup=2]
Engine bench read: count=1000 nlb=16 start_lba=1000000 step=16
completed: 1000
submitted: 1000
errors: 0
last_cqe_status: 0x0000
cycles: 381111
max_inflight: 32
latency_avg: 3.048 us
throughput: 2686.881 MB/s
iops: 327988.433 IOPS
payload_bytes: 8192000
hostmem_rd_present_cycles: 259255
hostmem_rd_accept_cycles: 257000
hostmem_rd_stall_cycles: 2255
hostmem_rd_tlps: 17000
hostmem_rd_gap_max: 3244950875
hostmem_rd_duty_pct: 68.026 %
litenvme>
```

**What you should see.** The BAR0 lines show a 64-bit BAR of 16 KiB assigned at `0xe0000000`,
and the MPS line the payload size that both ends now use, 512 bytes. `completed` equals `count`
and `errors` is 0. The figures read as follows:

- `cycles` is counted by the request generator, from the start to the last completion, at the
  125 MHz system clock; `throughput` and `iops` derive from it.
- `latency_avg` is that time divided by `count`, not the latency of one command.
- `max_inflight` is the most commands in flight that the firmware saw, here the queue depth of 32.
- `hostmem_rd_gap_max` is not meaningful: its counter runs before the first read.

**If not.** A firmware error line names the step that failed, such as BAR0, CSTS.RDY or an admin
CQE. A non-zero `errors` comes with the status of the first failing CQE. [Debugging](#debugging)
lists the causes of each one.

## Where to go next {#gs-next}

[Integration](#integration) adds LiteNVMe to your own SoC or flow. [Bench, firmware and host
tools](#examples) describes the other commands and harness scripts. [Performance](#performance)
collects the recorded results.
