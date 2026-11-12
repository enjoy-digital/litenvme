```{=latex}
\part{The core}
```

```{=html}
<h1 class="part">Part II. The core</h1>
```

# Architecture {#architecture}

This chapter describes how the core is built: its blocks, how they share the LitePCIe root port,
which clock they run on and how a block write travels from user logic to the SSD.

It is the map for the chapters that follow, which describe each block in detail. The NVMe
concepts used here are explained in [NVMe over PCIe in a nutshell](#nvme-primer).

## At a glance {#arch-glance}

| Item | Value |
|---------------------|-------------------------------------------------------------------|
| Role | NVMe host: the FPGA is the root complex, the SSD its only device |
| PCIe dependency | A LitePCIe root port with configuration requests enabled |
| Data widths | 64, 128 or 256 bits (the PCIe datapath width) |
| Clock | One domain, `sys`; LitePCIe crosses to its `pcie` domain |
| Queues | The admin queue (set-up) and one I/O queue (Read and Write) |
| Completion | Polling of the CQ phase bit, no interrupts |
| Host memory | A window of on-FPGA BRAM that the SSD reads and writes |
| Top-level class | `LiteNVMe` in `litenvme/core.py` |

## Blocks {#arch-blocks}

The core has three kinds of blocks, as the figure below shows. Accessors send requests from the
FPGA to the SSD: configuration reads and writes, and reads and writes of the controller registers
in BAR0. The host-memory window answers the requests of the SSD.

The I/O engine turns block requests into NVMe commands, and an optional front-end gives user
logic a simpler interface.

![Blocks of the core. Look at the direction of the arrows at the root port: the accessors send
requests to the SSD, while the host-memory window receives the requests of the SSD (cyan arrow).
The I/O engine writes commands into the window and rings doorbells through its own
accessor.](arch_blocks.svg)

| Block | Class | Role |
|----------------------|----------------------------------------|--------------------------------|
| Configuration accessor | `LiteNVMePCIeCfgAccessor` | CfgRd0 and CfgWr0 to the SSD |
| MMIO accessor | `LiteNVMePCIeMmioAccessor` | Controller register access |
| Doorbell accessor | `LiteNVMePCIeMmioAccessor` | Doorbell writes of the I/O engine |
| Host-memory window | `LiteNVMeHostMemResponder` | Memory the SSD reads and writes |
| I/O engine | `LiteNVMeIOEngineAXI` | Read and Write commands |
| Block streamer | `LiteNVMeBlockStreamer` | Sector streams through staging |
| Request generator | `LiteNVMeRequestGen` | Sequential requests for benchmarks |

Two more blocks live outside the `LiteNVMe` class because they belong to the SoC. The root
configuration accessor (`LiteNVMeRootCfgMgmt`) needs the PHY, and the init sequencer
(`LiteNVMeInitSequencer`) replaces the firmware. The generator instantiates them next to the
core; the bench SoC builds all the blocks itself, without the `LiteNVMe` class. See
[Bring-up](#bringup).

## Control plane and data plane {#arch-planes}

The traffic of the core splits into two planes, as the figure below shows. The control plane
sets the SSD up once: configuration space, controller registers, admin commands. It is slow and
runs from the firmware or the init sequencer through the accessors. The data plane carries the
commands and data of every transfer and runs in hardware.

![Control plane and data plane. Look at the arrows to the SSD: the control plane reaches the SSD
through the accessors, the data plane is the SSD reading and writing the host-memory window. The
I/O engine only enters the control plane to ring doorbells.](arch_planes.svg)

The split explains the sizes of the blocks. The accessors handle one 32-bit request at a time and
need no throughput. The host-memory window sees every byte of every transfer and is as wide as
the PCIe datapath. The I/O engine writes small commands into the window and lets the SSD move
the data itself.

## Crossbar ports {#arch-ports}

The core takes four ports on the crossbar of the LitePCIe root port: three master ports and one
slave port. The section *Crossbar* of the
[Core architecture chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/03_core_architecture.md)
of the LitePCIe manual describes the crossbar; the table below lists what the core puts on it.

| Port | Kind | Block | Requests | Tag given |
|-----------------|--------|------------------------|---------------------------|-----------|
| Configuration | Master | Configuration accessor | CfgRd0, CfgWr0 | `0x42` |
| MMIO | Master | MMIO accessor | MemRd, MemWr to BAR0 | `0x44` |
| Doorbell | Master | Doorbell accessor | MemWr to doorbells | `0x45` |
| Host memory | Slave | Host-memory window | MemRd, MemWr from the SSD | none |

The tags the accessors give are labels only. The three master ports can read, so their requests
pass through the TLP controller of the crossbar. The controller puts its own tag on every request
it sends and returns the completions, in request order, to the port that sent the read.

The slave port has an address decoder: a request from the SSD goes to the host-memory window when
its address is inside the window, from `hostmem_base` to `hostmem_base + hostmem_size`.

Requests outside the window go to the other slave ports of the crossbar, if any. With no other slave
port, the crossbar drops them; a MemRd then gets no completion and the SSD reports a completion
timeout.

## Clocking {#arch-clocking}

The whole core runs in the `sys` clock domain. The PCIe hard IP runs in its own `pcie` domain,
and the LitePCIe PHY crosses between the two with asynchronous FIFOs. Among the LiteNVMe blocks,
only the root configuration accessor has logic in the `pcie` domain; see
[Root configuration](#acc-rootcfg).

On the reference design, an Alibaba XCKU3P running PCIe Gen3 x4, the datapath is 256 bits at
125 MHz in both domains, and the hard IP core clock is 250 MHz. A 256-bit beat at 125 MHz moves
4 GB/s, more than a Gen3 x4 link carries.

The bench SoC declares the `sys` and `pcie` clocks as a false path. A standalone core in
internal-clock mode runs `sys` on the PCIe user clock and needs none; with `clk_external`, the
integrator must add it, since the root configuration accessor relies on it
([Known issues](#intro-issues)).

## The four traffic classes {#arch-traffic}

Because the FPGA is the root complex, the four kinds of PCIe traffic all occur, each with its own
interface on the UltraScale+ hard IP. The section *Root port* of the
[Core architecture chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/03_core_architecture.md)
of the LitePCIe manual describes how LitePCIe maps them; the table below says what each one
carries for LiteNVMe.

| Interface | Direction | Packets | Carries |
|-----------|-------------|-----------------|----------------------------------------------|
| RQ | FPGA to SSD | Cfg, MemRd, MemWr | Configuration, controller registers, doorbells |
| RC | SSD to FPGA | Cpl, CplD | Completions of configuration accesses and register reads |
| CQ | SSD to FPGA | MemRd, MemWr | SQE and PRP-list fetches, read data, CQEs |
| CC | FPGA to SSD | CplD | Answers to the reads of the window |

## The `LiteNVMe` class {#arch-class}

`LiteNVMe` builds the blocks of the first table, takes the crossbar ports and connects the I/O
engine to its doorbell accessor and to the host-memory window. Its parameters are below; the
complete list with the generator keys is in [Build options](#build-options).

| Parameter | Default | Meaning |
|---------------------------|----------------|-----------------------------------------|
| `pcie_endpoint` | none | The `LitePCIeRootPort` instance |
| `hostmem_base` | `0x10000000` | Bus address of the host-memory window |
| `hostmem_size` | `0x80000` | Size of the window (512 KiB) |
| `data_width` | PHY width | Width of the window and of the engine bridge |
| `qid` | 1 | I/O queue identifier |
| `qsize` | 64 | Entries of the I/O SQ and CQ |
| `qd` | 32 | Commands in flight, at most `qsize - 1` |
| `requester_id` | `0x0000` | Requester ID of the configuration requests |
| `with_block_streamer` | `False` | Build the block streamer |
| `with_request_gen` | `False` | Build the request generator |
| `staging_base` | `0x40000` | Offset of the staging region in the window |
| `staging_size` | `0x40000` | Size of the staging region (256 KiB) |
| `hostmem_backend` | `None` | External memory for the window |
| `with_csr` | `True` | Registers for accessors, engine, generator |

The window base must be a bus address that the SSD uses for nothing else, and the staging region
must lie inside the window. [The host-memory window](#host-memory) gives the layouts used by the
firmware and the generator.

### Front-end ownership {#arch-frontend}

The I/O engine has one request stream and one completion stream, so one front-end drives it.
`LiteNVMe` connects them according to the front-ends it builds:

- the block streamer alone: the block streamer owns the engine streams;
- the request generator alone: the request generator owns them;
- none, or both: the core leaves the engine streams free and exports them as `request` and
  `completion`, for user logic or for a multiplexer in the SoC.

When both front-ends are built, the core connects neither of them and the SoC adds a
multiplexer; the bench SoC, which assembles the blocks itself, selects the front-end with a
register. [Front-ends](#frontends) describes the three ways of
driving the engine.

### Registers {#arch-registers}

`add_csrs(soc, name="nvme")` adds the register banks of the sub-blocks to a SoC under one prefix:
`nvme_cfg`, `nvme_mmio`, `nvme_engine`, `nvme_hostmem`, `nvme_gen` and `nvme_block`.

A LiteX SoC also collects the registers of a module assigned as an attribute, such as
`self.nvme`, under the attribute name. A core assigned as an attribute and passed to `add_csrs`
therefore has some banks twice, under two names; the generator does this
([Known issues](#intro-issues)). The register list is in [Register reference](#registers).

## A block write, end to end {#arch-write}

The figure below follows one write through the block streamer, from the data of user logic to the
`done` flag. It shows every block of this part at work.

![A block write through the core. Read it top to bottom: the data is copied into the staging
region first, then the I/O engine writes the command and rings the doorbell, the SSD fetches the
command and reads the data from the window, and the engine reaps the completion.](arch_write.svg)

1. User logic pulses `start` with the sector, the count and `write` set, then sends the data
   beats. The block streamer copies them into the staging region of the window.
2. The block streamer sends one Write request to the I/O engine, pointing at the staging region.
3. The I/O engine writes the 64-byte SQE into the I/O SQ of the window, then rings the SQ doorbell
   with a MemWr to BAR0 through its doorbell accessor.
4. The SSD fetches the SQE with a MemRd to the window, then reads the data with more MemRd. The
   window answers each read with a completion.
5. The SSD writes the CQE into the I/O CQ of the window. The I/O engine sees its phase bit change,
   emits a completion and rings the CQ doorbell.
6. The block streamer checks the status of the completion and raises `done`, with `error` set if
   the status is not success.

A read is the same exchange in the other direction: the SSD writes the data into the staging
region with MemWr, and the block streamer copies it out after the completion. No CPU takes part
in either transfer.

## Where to go next {#arch-next}

[Configuration and register access](#pcie-access) describes the accessors,
[The host-memory window](#host-memory) the memory the SSD sees, [The I/O engine](#io-engine) the
command engine, [Front-ends](#frontends) the interfaces for user logic and [Bring-up](#bringup)
the set-up sequence. The status of each part is in the [status table](#intro-status).
