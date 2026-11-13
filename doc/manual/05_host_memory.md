# The host-memory window {#host-memory}

This chapter describes the host-memory window: the memory that the SSD sees as host memory. In
a PC, the queues and the data buffers of an NVMe driver live in DRAM; here they live in a block of
FPGA memory that answers the PCIe requests of the SSD.

Read it to size the window, to place the
queues and buffers in it, or to understand what the SSD can and cannot do with it.

## At a glance {#mem-glance}

| Item | Value |
|---------------------|-------------------------------------------------------------------|
| Role | Completer for the MemRd and MemWr of the SSD |
| Contents | Admin and I/O queues, PRP lists, data buffers, staging region |
| Address range | `hostmem_base` to `hostmem_base + hostmem_size` on the PCIe bus |
| Default size | 512 KiB (`LiteNVMe`), 256 KiB (generator example) |
| Storage | BRAM, one word per PCIe beat, byte write enables |
| Data widths | 64, 128 or 256 bits |
| Inside access | AXI arbiter shared by the I/O engine, the block streamer, the init sequencer and a debug port |
| Top-level class | `LiteNVMeHostMemResponder` in `litenvme/hostmem.py` |

## Role {#mem-role}

NVMe moves almost everything through host memory. The SSD fetches commands from the SQ, reads
PRP lists, reads the data of writes, writes the data of reads and posts completions into the CQ,
all with its own MemRd and MemWr.

The host only writes doorbells. The window is therefore the busy side of the core: every command
and every data byte crosses it.

The SSD knows the window only by its bus addresses: the base of the admin queues in ASQ and ACQ,
the queue bases given by the admin commands, and the PRP entries of each command. Any address
outside the window that the SSD is given leads to a request that the window does not answer.

## Structure {#mem-blocks}

The figure below shows the blocks of the window. On the PCIe side, a responder turns the requests
of the SSD into memory accesses. On the inside, an AXI arbiter shares the memory between the
responder and the blocks of the core that write commands or copy data.

![Blocks of the host-memory window. Follow the cyan arrow: the requests of the SSD go through the
responder and the arbiter to the BRAM backend. The other masters reach the same memory through
the arbiter, one single-beat access at a time.](mem_blocks.svg)

| Block | Class | Role |
|----------------------|--------------------------------|------------------------------------|
| Responder | `LiteNVMeHostMemDMA` | MemRd and MemWr of the SSD |
| BRAM backend | `LiteNVMeHostMemAXIRAM` | The memory, as an AXI slave |
| Register debug port | `LiteNVMeHostMemCSR` | Dword access and counters |
| Arbiter | `AXIArbiter` (LiteX) | Shares the backend between masters |

## BRAM backend {#mem-backend}

The backend is a memory one PCIe beat wide: 32 bytes at 256 bits, so 16 384 words for 512 KiB.
Each byte has its own write enable, so a write can change one dword of a beat and keep the others.
The size must be a multiple of the beat.

The backend is an AXI slave that accepts single-beat transactions only. It takes a read address
only when its burst length is 0, and returns the data one cycle later through a 64-entry FIFO.

Writes go through one address slot and one data slot, and a beat is committed when both are full,
so the backend takes one beat per cycle.

With the register debug port, the backend has a third memory port for debug reads. A BRAM has two
ports, so a third one makes the synthesis tool duplicate the whole memory. The responder builds
the debug port only when it has registers (`with_csr=True`); the generator leaves it out of a
standalone core whose front-end is driven from pins.

## The responder {#mem-responder}

The responder (`LiteNVMeHostMemDMA`) sits on the slave port of the crossbar. For each request, it
first checks that the address is inside the window, then serves a write or a read.

- **MemWr** (the SSD writes data or a CQE): the first beat is placed at its address, then the
  following beats go to the next words. A request of `len` dwords takes `len` divided by the dwords
  per beat, rounded up. Beats go through a 64-entry FIFO and are written to the backend without
  waiting for each write response.
- **MemRd** (the SSD fetches an SQE, a PRP list or the data of a write): the responder reads the
  beats one by one and sends them back as one completion with data (CplD) carrying all `len`
  dwords. It does not split a completion at MPS, so the MRRS of the SSD must not exceed the MPS
  of the link ([Known issues](#intro-issues)). It serves one MemRd at a time; the next request
  waits until the completion has left.

A request outside the window is dropped: its beats are accepted and discarded, and the request
that follows is served. A dropped MemRd gets no completion, so the SSD sees a completion timeout.
In the core, the address decoder of the slave port already keeps such requests away from the
responder ([Architecture](#architecture)).

The responder ignores the first and last dword byte enables of a request: every write covers
whole dwords, which is what an SSD sends for queues and data.

The completion carries the tag, the requester ID, the length, a Byte Count of `len` times 4 and
the Lower Address of the request. Every LitePCIe PHY sends these fields as given, the UltraScale+
adapter included; a Byte Count of 0 would mean 4096 bytes and stall the SSD.

### Sub-beat writes {#mem-subbeat}

A CQE is 16 bytes, and a 256-bit beat is 32 bytes, so every second CQE of a queue starts in the
middle of a beat. The PCIe port delivers its data from dword 0 of the beat, whatever the address.

The responder shifts the first beat of a write to the dword offset of its address and sets the
byte enables of the written dwords only, as the figure below shows.

![A 16-byte CQE written at ACQ + 16 on a 256-bit window. Look at the byte enables: the four
dwords of the CQE move to dwords 4 to 7 and only they are written, so the CQE in dwords 0 to 3
stays intact.](mem_subbeat.svg)

The shift is a case statement with one constant shift per offset. A shift by a variable amount
gave the same result in simulation but was implemented as no shift at all on hardware; the code
keeps the constant form for that reason.

Only the first beat of a write is realigned. A write that spans several beats must therefore
start on a beat boundary, and its later beats are written whole.

A read is always served from the start of the beat that holds its address.

The core places SQEs, PRP-list pages and data buffers on beat boundaries, and the SSD of the
bench sends aligned requests for them; the only unaligned writes it sends are 16-byte CQEs. An
SSD that fetches a PRP list or moves data in TLPs that start inside a beat would get wrong data:
this is a limitation of the window.

## AXI arbiter and masters {#mem-arbiter}

Every block that touches the window from the inside is an AXI master on the arbiter. The arbiter
of LiteX serves the read and write channels separately and keeps a master until all its requests
have been answered. All the masters below use single-beat transactions.

| Master | Present when | Accesses |
|--------------------|---------------------------------|---------------------------------------------|
| Responder | Always | Requests of the SSD |
| Register debug port | `with_csr=True` | Dword reads and writes by software |
| I/O engine bridge | Always (`LiteNVMe`) | SQE and PRP-list writes, CQE reads |
| Staging DMA | Block streamer built | Copies between the streams and staging |
| Init bridge | Init sequencer built | Admin SQE writes, admin CQE reads |

The I/O engine and the init sequencer access memory one dword at a time through a bridge
(`LiteNVMeMemPortToAXI`) that turns each dword into a single-beat AXI access on its enclosing
beat, with the byte enables of that dword.

The bridge subtracts the window base from the address, since the backend starts at 0; it must be
built with the same base as the window.

## Window layouts {#mem-layout}

The window has no fixed layout in the gateware: the firmware, the init sequencer and the
parameters of the core each place their regions. The firmware and the init sequencer share the
layout of the figure below for the queues, at fixed offsets from the window base.

![Layout of the host-memory window used by the firmware and the init sequencer. Read the offsets
on the left: the queues sit in the first 20 KiB, the data region starts at `+0x10000`, and the
PRP-list region, one 4 KiB page per I/O queue entry, follows the data region unless the design
places it elsewhere.](mem_map.svg)

The table below gives the variable regions of the two reference configurations. The data
region holds the per-slot buffers of the firmware benchmarks or the staging region of the block
streamer.

| Configuration | Window | Entries | Staging region | PRP-list region |
|---------------------------|---------|----------|--------------------------|--------------------------|
| Bench SoC, either bring-up | 512 KiB | 64 | `+0x40000` to `+0x80000` | `+0x50000` to `+0x90000` |
| Generator example, either bring-up | 256 KiB | 8 | `+0x10000` to `+0x40000` | `+0x7000` to `+0xf000` |

In the bench SoC, the firmware also places 64 slot buffers of 4 KiB from `+0x10000` to `+0x50000`
for its benchmarks. By default, the PRP-list region starts at `+0x10000` plus one page per queue
entry, as the firmware and the init sequencer compute it; the I/O engine only uses it for
transfers of more than two pages.

The generator places the region itself and gives its address, with the base of the window and the
queue size, to the init sequencer or to the firmware. The example sets it with `prp_list_offset`,
between the queues and the staging region. Without this key, the generator takes the first place
that fits among the default one, `+0x7000` and the end of the staging region. It refuses a
configuration whose PRP-list region leaves the window, starts below `+0x7000` or overlaps the
staging region ([Build options](#opt-gen-core)).

The values of the bench SoC overlap in two ways. Its PRP-list region lies inside the staging
region: a block streamer transfer long enough to reach the list page of its slot, 64 KiB, has
that page written over its data.

The region also ends past the window, so the list pages of slots 48 to 63 are outside it. The
firmware benchmarks stay clear of both by limiting transfers to two pages;
[The I/O engine](#io-engine) gives the PRP rules.

## Register debug port {#mem-debug}

The register debug port (`LiteNVMeHostMemCSR`) lets software read and write the window one dword
at a time, and exposes counters of the responder. The firmware uses it to build admin commands,
to read their completions and to fill or check data buffers.

| Register | Meaning |
|-----------------------|-------------------------------------------------------------|
| `csr_adr` | Dword index in the window (byte offset divided by 4) |
| `csr_wdata` | Data of the next write |
| `csr_we` | Write request, on its rising edge: write 1, then 0 |
| `csr_rdata` | The dword at `csr_adr` |
| `dma_wr_count` | Beats written by the responder |
| `dma_rd_count` | Beats read by the responder |
| `wr_present_cycles` | Cycles a MemWr beat was offered at the port |
| `wr_accept_cycles` | Cycles a MemWr beat was accepted |
| `wr_stall_cycles` | Cycles a MemWr beat was offered but not accepted |
| `rd_tlp_count` | MemWr requests accepted |
| `rd_gap_max` | Longest run of cycles without a MemWr beat offered |

The port issues one write each time `csr_we` goes from 0 to 1, so software writes 1 then 0. A 1
left in the register does not repeat the write.

The read data follows the address through the debug memory port with a short delay, so the
firmware reads `csr_rdata` twice after changing `csr_adr` and keeps the second value.

The MemWr counters describe the data of NVMe reads arriving from the SSD. Comparing
`wr_stall_cycles` with `wr_present_cycles` tells whether the window slows the SSD down, or the
SSD and the link deliver with gaps.

The counters run from reset and never clear, so software
reads them before and after a run and takes the difference. `rd_gap_max` also counts the idle time
before the first read, so it is large until it is read after real traffic.

## External backend {#mem-external}

`LiteNVMe` takes an optional `hostmem_backend`: any module with an AXI slave named `axi`, of the
data width of the window, that accepts single-beat transactions. Such a backend, a LiteDRAM port
for example, would give a window larger than the BRAM of the FPGA.

Without a debug read port in the
backend, the register debug port writes but its reads are not connected. The status of this
option is in the [status table](#intro-status).

## In the code {#mem-code}

All the classes of this chapter are in `litenvme/hostmem.py`, except the dword bridge
`LiteNVMeMemPortToAXI` in `litenvme/io_engine.py`. The firmware layout is defined at the top of
`bench/firmware/main.c`, the init sequencer layout at the top of `LiteNVMeInitSequencer` in
`litenvme/init.py`.

## Where to go next {#mem-next}

[The I/O engine](#io-engine) describes the commands written into the window;
[Front-ends](#frontends) the staging region; [Debugging](#debugging) the use of the counters.
