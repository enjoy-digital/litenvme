```
                               __   _ __      _  ___   ____  ___
                              / /  (_) /____ / |/ / | / /  |/  /__
                             / /__/ / __/ -_)    /| |/ / /|_/ / -_)
                            /____/_/\__/\__/_/|_/ |___/_/  /_/\__/

                           Small footprint LiteX based NVMe core.
                           Copyright (c) 2025-2026 Enjoy-Digital.
```

![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)

> **Status: functional, hardware-validated.**
>
> LiteNVMe is an open-source **NVMe host core**: it brings up a commercial NVMe SSD over PCIe
> and lets FPGA logic read/write it through a simple block interface — no host CPU, OS or driver
> on the data path. It runs end-to-end on real hardware at PCIe Gen3 x4 (~2.7 GB/s), and can be
> generated as a **standalone Verilog core** for use outside LiteX. Active work: Gen4/Gen5 scaling
> and a pure-RTL init sequencer (see *Roadmap*).

[> Intro
--------

LiteNVMe is a small-footprint, configurable **NVMe host/initiator core** for LiteX. It is the
PCIe *RootPort* side: it discovers and initializes an off-the-shelf NVMe SSD (controller enable,
Identify, create the I/O queues) and then issues read/write commands to it, keeping many commands
in flight in hardware. This is the opposite of the more common NVMe *device* cores (which *are*
the SSD controller) — to our knowledge LiteNVMe is the only **open-source NVMe host** core.

It can be used two ways:

* **As a LiteX/Migen package** — instantiate `LiteNVMe` (`litenvme/core.py`) in your LiteX SoC
  on top of a LitePCIe RootPort.
* **As a standalone Verilog core** — `litenvme_gen` produces a self-contained `litenvme_core.v`
  (+ C headers) from a YAML config, for integration in a traditional FPGA flow with no
  Migen/LiteX dependency. See `doc/STANDALONE_CORE.md`.

[> Features
-----------

* **PCIe RootPort NVMe host**, Gen3 x4, 256-bit datapath @ 125 MHz (LitePCIe USPPCIEPHY,
  Ultrascale+ `pcie4_uscale_plus` hard IP).
* **Hardware I/O command engine** (`LiteNVMeIOEngine`): builds SQEs, rings SQ/CQ doorbells, reaps
  CQEs by phase bit in a tight FSM, keeping up to `qd` (default 32) commands outstanding with **no
  per-command CPU cost**. PRP1 / PRP2 / PRP-list (multi-page transfers).
* **LiteSATA-style block interface**: a `{write, sector, count}` command + AXI-Stream data in/out
  (`wr_sink` / `rd_source`) — no knowledge of PRPs or host memory needed.
* **Self-contained bring-up**: a small embedded soft-CPU runs the NVMe init firmware and raises
  `init_done`; steady-state I/O is then hardware-driven.
* **Configurable & portable**: parameterized data width / queue depth / host-memory window;
  **0 URAM, 0 DSP** (LUT/FF + BRAM only). Pluggable host-memory backend (on-FPGA BRAM by default,
  or a DDR/LiteDRAM AXI port for large buffers).
* **Standalone core generator** (`litenvme_gen`) + an Etherbone-driven test SoC for
  throughput/correctness.
* Validated end-to-end in simulation (`pytest`, 39 tests) and on real hardware.

[> Performance & resources
--------------------------

Measured on an Alibaba KU3P (XCKU3P) with a Crucial CT500P310SSD8, Gen3 x4 / 256-bit @ 125 MHz:

| Workload                 | Result                              |
|--------------------------|-------------------------------------|
| 8 KiB reads (QD32)       | **~2.69 GB/s**, errors = 0          |
| 8 KiB writes (QD32)      | **~2.74 GB/s**, errors = 0          |
| Data integrity           | write → read-back bit-exact         |

That is ~80 % of the usable Gen3 x4 link; the remaining gap is the SSD's 512-byte MaxPayloadSize
(read TLP fragmentation), i.e. the design is device/link-bound, not core-bound.

Standalone core resource usage (out-of-context synthesis of `litenvme_core`, default block-streamer
config, 256 KiB window + bring-up CPU):

| LUT    | FF     | BRAM tiles | URAM | DSP |
|-------:|-------:|-----------:|-----:|----:|
| 11,577 | 14,408 | 133 (37 %) | 0    | 0   |

BRAM is the dominant, tunable cost (the host-memory window + the bring-up CPU's ROM/RAM); shrink
`hostmem_size`, use a DDR backend, or drop the CPU to cut it. See `doc/STANDALONE_CORE.md` §5.

[> How it compares
------------------

Compared with the (closed-source) commercial NVMe **host** IP cores:

* On **Gen3 x4**, LiteNVMe reaches a comparable, near-link-ceiling throughput and sits in a
  similar resource/area class — while being **open source** with exact, reproducible numbers.
* The higher absolute figures some cores advertise (7–11 GB/s) come from **Gen4/Gen5/x8 PCIe**, a
  PHY/lane scaling story rather than a different host architecture.
* One real difference: some commercial cores perform NVMe init entirely in **pure RTL** (no CPU
  anywhere), whereas LiteNVMe does init in firmware on a small embedded CPU (a pure-RTL init
  sequencer is on the roadmap).

A sourced, caveated comparison is in `doc/NVME_CORE_COMPARISON.md`.

[> The block interface
----------------------

The generated standalone core exposes (Gen3 x4, `data_width = 256`):

```
  pcie_clk_p/n, pcie_rx_p/n[3:0], pcie_tx_p/n[3:0], rst_n   # PCIe pads (PERST# out to the SSD)
  clk, rst                                                  # sys clock/reset (from PCIe user clk)
  status_init_done, status_init_error                       # 1 once NVMe bring-up completed

  block_ctrl_{start, write, sector[63:0], count[31:0], nsid[31:0]}   # command in
  block_ctrl_{done, busy, error}                                     # status out
  block_wr_axis_{tvalid, tready, tlast, tdata[255:0]}               # write payload in
  block_rd_axis_{tvalid, tready, tlast, tdata[255:0]}              # read payload out
```

Usage: wait for `status_init_done`; then for each transfer drive `{write, sector, count}`, pulse
`block_ctrl_start`, stream `count*512` bytes on `block_wr_axis` (write) or consume them on
`block_rd_axis` (read), and wait for `block_ctrl_done` (check `block_ctrl_error`). 1 sector =
512 B; the 256-bit bus carries 32 B/beat. See `doc/STANDALONE_CORE.md` for the full protocol.

[> Getting started
------------------

Install Python 3.8+, your FPGA vendor tools, and LiteX (per the LiteX wiki). Then:

**Generate a standalone core**

```sh
litenvme_gen examples/alibaba_xcku3p.yml --output-dir build --name litenvme_core
# -> build/gateware/litenvme_core.v + litenvme_core.tcl/.xdc + software/include/generated/*.h
```

Integrate `litenvme_core.v` into your project, generate the `pcie4_uscale_plus` IP via the emitted
`.tcl`, wire the PCIe pads + block interface, and build. See `doc/STANDALONE_CORE.md` §4 for the
traditional-flow steps.

**Build the reference test SoC (Alibaba KU3P) and drive it over Etherbone**

```sh
# build + load the test SoC (CPU + Etherbone + I/O engine + block streamer)
./bench/alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone \
    --with-io-engine --with-block-streamer --csr-csv=csr.csv --build --load

# bring up + run throughput (request generator) and correctness (block streamer BIST)
./bench/hw_block.sh                                   # load -> boot -> nvme_setup -> host test
python3 bench/test_block.py --csr-csv build/alibaba_xcku3p/csr.csv
python3 bench/test_block.py --isolated               # write/scrub/read-back persistence proof
```

**Use it as a LiteX package** — instantiate `LiteNVMe(pcie_endpoint, ...)` (`litenvme/core.py`) on
a `LitePCIeRootPort`; `bench/alibaba_xcku3p.py` is the worked example.

[> Documentation
----------------

* `doc/ARCHITECTURE.md`        — the as-built Gen3-256b architecture (datapath, accessors, engine).
* `doc/STANDALONE_CORE.md`     — generating, the interface, usage, integration flow, resources.
* `doc/NVME_PERFORMANCE.md`    — throughput study, the 512 B MPS ceiling, transfer-size sweep.
* `doc/NVME_CORE_COMPARISON.md`— comparison with other public NVMe cores (sourced, with caveats).
* `doc/PROGRESS.md`            — development log.

[> Roadmap
----------

* PCIe Gen4 / Gen5 and wider lane counts (PHY/lane + datapath scaling) for higher throughput.
* A **pure-RTL init sequencer** to remove the bring-up CPU for fully autonomous, software-free
  operation (matching CPU-less commercial cores).
* Multi-chunk block transfers larger than the staging window; CSR-name alignment so the generated
  core self-inits from the minimal firmware.
* DDR/LiteDRAM-backed host-memory window for very large buffers.

If you want to support or accelerate these, contact us at florent [AT] enjoy-digital.fr.

[> Tests
--------

Simulation tests live in `./test/` (host-memory backend, I/O engine + PRP, request generator,
block streamer, MMIO/CFG accessors, NVMe sequencing):

```sh
$ pytest -q          # full suite (39 tests)
```

[> License
----------

LiteNVMe is released under the very permissive two-clause BSD license. Under
the terms of this license, you are authorized to use LiteNVMe for closed-source
proprietary designs.

Even though we do not require you to do so, those things are awesome, so please
do them if possible:

* tell us that you are using LiteNVMe
* cite LiteNVMe in publications related to research it has helped
* send feedback and suggestions for improvements
* send bug reports when something goes wrong
* send modifications and improvements you have done to LiteNVMe.

[> Support and consulting
-------------------------

LiteNVMe is developed and maintained by EnjoyDigital.

If you would like to know more about LiteNVMe or extend it for your needs,
EnjoyDigital can provide standard commercial support as well as consulting services.

So feel free to contact us, we'd love to work with you!

[> Contact
----------

E-mail: florent [AT] enjoy-digital.fr
