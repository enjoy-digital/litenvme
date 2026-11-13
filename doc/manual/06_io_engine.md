# The I/O engine {#io-engine}

This chapter describes the I/O engine, the block that runs NVMe Read and Write commands without a
CPU. It takes block requests on a stream, writes the commands into the I/O submission queue,
rings the doorbells and returns the completions on another stream, with many commands in flight.

Read it to drive the engine from your own logic or to understand its throughput.

## At a glance {#eng-glance}

| Item | Value |
|-------------------|-------------------------------------------------------------------|
| Queue | One I/O SQ and its CQ, created by bring-up |
| Commands | Read (opcode `0x02`) and Write (opcode `0x01`) |
| Commands in flight | Up to `qd`, at most `qsize - 1` |
| Command ID | The SQ slot index of the command |
| Data pointers | PRP1, PRP2 and one PRP-list page per slot; no SGL |
| Sector size | 512 bytes; memory page size 4 KiB |
| Completion | Polling of the phase bit, no interrupts |
| Doorbells | One SQ doorbell per burst, one CQ doorbell per batch |
| Top-level class | `LiteNVMeIOEngineAXI` in `litenvme/io_engine.py` |

## Request and completion streams {#eng-streams}

User logic or a front-end gives the engine one request per command on its `sink` stream. The
engine takes the request (raises `ready`) once it has written the command into the SQ, which can
be many cycles after `valid`.

| Request field | Width | Meaning |
|---------------|-------|--------------------------------------------------------------|
| `op` | 1 | 0 for Read, 1 for Write |
| `nsid` | 32 | Namespace, 1 on the reference SSD |
| `lba` | 64 | First sector |
| `nlb` | 16 | Number of sectors, from 1 (the engine writes `nlb - 1` in the SQE) |
| `buf` | 64 | Bus address of the data buffer in the host-memory window |

The engine returns one completion per command on its `source` stream, in the order the SSD posts
them in the CQ. An SSD may complete commands out of submission order, so user logic matches a
completion to its command by the command identifier.

| Completion field | Width | Meaning |
|------------------|-------|---------------------------------------------------------------|
| `cid` | 16 | Command identifier, the SQ slot of the command |
| `status` | 16 | Upper half of CQE dword 3: phase bit (bit 0), SC (bits 8:1), SCT (bits 11:9), CRD (bits 13:12), M (bit 14), DNR (bit 15) |
| `sqhd` | 16 | SQ head reported by the SSD |

A command succeeded when SC and SCT are both 0. The engine passes the status on without acting on
it: a failed command is reported like any other.

## Command identifiers {#eng-cid}

The engine gives each command the index of its SQ slot as its command identifier (CID). The SQ
tail starts at 0 after reset and wraps at `qsize`, so the CIDs of successive commands are 0, 1,
2 and so on, modulo `qsize`. User logic that counts its requests knows the CID of each one.

A slot is reused after `qsize` commands, and the SSD may hold the command of that slot longer if
it completes out of order. Reusing its CID may make the SSD reject the new command, with the
status Command ID Conflict (the bench SSD does).

The engine therefore keeps a busy bit per slot, set at submission and cleared
when the CQE of that CID is reaped, and waits until the slot it is about to use is free.

## State machine {#eng-fsm}

The engine is one state machine that alternates between submitting and reaping. In IDLE, it picks
the first path that has work, in the priority order of the figure below; every path returns to
IDLE, so submissions and completions interleave.

![State machine of the I/O engine. Read the conditions from top to bottom: the engine submits
while it can, rings the SQ doorbell once it cannot submit more, reaps completions while commands
are in flight, and rings the CQ doorbell once a batch is reaped.](eng_fsm.svg)

1. **Submit.** A request is waiting, fewer than `qd` commands are in flight and the slot at the
   SQ tail is free. The engine latches the request (LATCH-REQ), writes a PRP list if the transfer
   needs one (PRP-LIST-WRITE), writes the 16 dwords of the SQE (SUBMIT-WRITE), then takes the
   request and advances the SQ tail (SUBMIT-ADVANCE).
2. **SQ doorbell.** The SQ tail has moved since the last SQ doorbell. The engine writes the tail to
   the SQ doorbell through its doorbell accessor and waits for the accessor to finish.
3. **Reap.** Commands are in flight. The engine reads dword 3 of the CQE at the CQ head
   (REAP-READ) and compares its phase bit with the expected phase (REAP-CHECK). On a match, it
   reads dword 2 for the SQ head, emits the completion and advances the CQ head, then reads the
   next CQE. On a mismatch, the batch is over.
4. **CQ doorbell.** The CQ head has moved since the last CQ doorbell. The engine writes the head to
   the CQ doorbell.

The engine reads dword 3 before dword 2 on purpose. The SSD writes the whole CQE, and its phase
bit changes with the last write. Reading the phase first guarantees that the rest of the entry is
new.

Reading the dwords in order could mix the SQ head of the old entry with the phase of the new one
at the moment the SSD overwrites it.

The expected phase starts at 1 and flips each time the CQ head wraps to slot 0. The CQ must
therefore hold no phase bit at 1 when the queue is created. The firmware clears it; the init
sequencer relies on the window being zero after the FPGA is configured.

## Submission queue entry {#eng-sqe}

The engine builds the SQE from the latched request. Only the dwords below are set; the others are
written as 0.

| Dword | Content |
|-------|---------------------------------------------------------------|
| 0 | Opcode (bits 7:0), CID (bits 31:16) |
| 1 | NSID |
| 6, 7 | PRP1: `buf` |
| 8, 9 | PRP2: 0, the second page or the PRP-list page |
| 10, 11 | Starting LBA: `lba` |
| 12 | Number of logical blocks minus 1 (bits 15:0) |

The SQE of slot `n` is at `sq_base + 64 * n`. The engine writes it one dword at a time, so an SQE
costs 16 single-beat writes into the window.

## Data pointers (PRP) {#eng-prp}

NVMe describes the data buffer of a command with physical region page (PRP) entries, each the
address of one memory page of 4 KiB. The engine assumes that the buffer at `buf` is contiguous
and starts on a page boundary, and derives the PRP entries from it.

| Transfer size | Pages | PRP1 | PRP2 |
|---------------------|-------|-------|--------------------------------------------|
| Up to 4 KiB | 1 | `buf` | 0 |
| More than 4 KiB, up to 8 KiB | 2 | `buf` | `buf + 4 KiB` |
| More than 8 KiB | 3 or more | `buf` | The PRP-list page of the slot |

The number of pages is the transfer size, `nlb * 512` bytes, divided by 4 KiB and rounded up.
For three pages or more, PRP2 points to a list page that the engine writes before the SQE, as the
figure below shows.

Each slot has its own list page at `prp_list_base + 4 KiB * slot`, so the lists of commands in
flight never collide.

![PRP list of a 16 KiB transfer. Follow the arrows: PRP1 points to the first page of the buffer,
PRP2 to the list page of the slot, and each list entry to one of the following pages.](eng_prp.svg)

A list page holds 512 entries of 8 bytes. The engine writes one entry per page after the first
and never chains a second list page, so a transfer is limited to 513 pages, 2052 KiB.

Above that, the SSD would also read entry 511 of the page as a pointer to a next list page, and
the entries of a longer transfer overflow into the list page of the next slot. The SSD sets its
own limit, the MDTS field of Identify Controller, which is often lower.

The PRP-list region takes `qsize` pages of the window, at the address written in the `prp_list`
registers. [Window layouts](#mem-layout) shows where the firmware and the init sequencer put it.

## Doorbell coalescing {#eng-doorbells}

A doorbell is a MemWr through the doorbell accessor, much slower than a write into the window.
The engine therefore rings each doorbell once per group of entries, with the latest pointer
value, as the figure below shows.

![Doorbell coalescing. Look at the accent arrows: three SQEs are announced by one SQ doorbell,
and two CQEs are acknowledged by one CQ doorbell, rung when the next CQE is not ready
yet.](eng_pipeline.svg)

The engine remembers the last values written to each doorbell. A doorbell is due when the pointer
has moved since; because no more than `qsize - 1` commands are in flight, the tail cannot travel a
whole ring between two doorbells and the comparison is unambiguous.

Submitting has priority, so the SQ doorbell rings when the request stream pauses or the engine
reaches `qd` commands in flight.

## Queue depth {#eng-qd}

The number of commands in flight sets the throughput
([Queue depth and throughput](#primer-qd)).

`qd` sets the maximum number of commands in flight; the engine asserts at build time that it is at
most `qsize - 1`. Each command in flight needs its own data buffer in the window, so a high `qd`
also needs a larger window. The request generator gives each slot its own buffer for that reason;
see [Front-ends](#frontends).

## Host-memory port {#eng-memport}

The engine reaches the window through a dword port, `LiteNVMeMemPort`, which keeps it independent
of the window width. `LiteNVMeIOEngineAXI` connects this port to the AXI arbiter of the window
through the dword bridge described in [AXI arbiter and masters](#mem-arbiter).

| Signal | Width | Meaning |
|---------|-------|--------------------------------------------------------------|
| `adr` | 32 | Dword address (bus byte address divided by 4) |
| `we` | 1 | 1 for a write |
| `dat_w` | 32 | Write data |
| `stb` | 1 | Access requested; held with the other inputs until `ack` |
| `ack` | 1 | One-cycle pulse when the access at `adr` is done |
| `dat_r` | 32 | Read data, valid in the cycle of `ack` |

`ack` must belong to the address on `adr`: a backend raises it only when the access it presents
is done. The bridge to AXI makes each dword access a full single-beat transaction, with one idle
cycle between accesses.

## Registers {#eng-registers}

The engine is configured through registers when it is built with `with_csr=True`, or through
signals driven by the init sequencer otherwise. The addresses are written once, before `enable`.

| Register | Meaning |
|-----------------------|--------------------------------------------------------------|
| `enable` | 1: the engine may submit and reap |
| `sq_base_lo`, `_hi` | Bus address of the I/O SQ |
| `cq_base_lo`, `_hi` | Bus address of the I/O CQ |
| `sq_db_lo`, `_hi` | Bus address of the SQ doorbell in BAR0 |
| `cq_db_lo`, `_hi` | Bus address of the CQ doorbell in BAR0 |
| `prp_list_lo`, `_hi` | Bus address of the PRP-list region |
| `status` | `busy` (bit 0), `inflight` (bits 15:8) |
| `submitted` | Commands submitted since reset |
| `completed` | Completions reaped since reset |

The engine itself does not use the queue identifier: it knows the queue only by the addresses of
its rings and doorbells. Clearing `enable` stops new submissions and reaping, but keeps the ring
pointers; they return to 0 only with a reset of the core. The full register list is in
[Register reference](#registers).

## Limits {#eng-limits}

- One I/O queue, Read and Write only: no Flush, Dataset Management or other I/O command.
- Data buffers contiguous in the window and starting on a 4 KiB boundary; sectors of 512 bytes.
- `nlb` from 1. A request with `nlb` of 0 is refused: the engine consumes it without writing an
  SQE and returns a completion with the status Invalid Field in Command (`0x8004`) and the CID
  of the slot it would have used.
- At most 513 pages per command (one PRP-list page per slot).
- The error flag of the doorbell accessor is not checked.

## In the code {#eng-code}

`LiteNVMeIOEngine`, the dword port `LiteNVMeMemPort`, the bridge `LiteNVMeMemPortToAXI` and the
wrapper `LiteNVMeIOEngineAXI` are in `litenvme/io_engine.py`. The firmware that programs the
engine registers is `nvme_autoinit` in `bench/firmware/main.c`.

## Where to go next {#eng-next}

[Front-ends](#frontends) describes the blocks that drive the engine;
[Bring-up](#bringup) creates the queue the engine uses.
