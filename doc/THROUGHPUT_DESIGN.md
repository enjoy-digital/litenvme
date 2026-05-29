# LiteNVMe — Throughput Design Notes (P1/P2/P4)

Implementation-ready design for the throughput effort. Companion to
`doc/NVME_ARCHITECTURE.md` (roadmap), `doc/NVME_PERFORMANCE.md` (measured numbers),
and `doc/PROGRESS.md` (live log).

## 0. The governing model

NVMe steady-state throughput obeys Little's Law:

```
in_flight ≈ throughput × latency
IOPS      ≈ queue_depth / per_command_latency      (when QD < bandwidth limit)
MB/s      ≈ IOPS × transfer_size
```

Measured today (QD=1, PRP1, 4 KiB): ~32 µs/command → ~31k IOPS → ~130 MB/s, and
per-command time is *independent of payload size* (nlb 1→8 leaves latency flat). That
is the signature of a control-path-bound, single-outstanding pipeline.

Two independent levers, multiplicative:

1. **Queue depth (QD):** keep N commands in flight. Ideal IOPS scales ~linearly with
   QD until something saturates (device internal parallelism, or the PCIe link).
   Expectation: QD=8–32 should move 4 KiB throughput from ~130 MB/s toward the link
   ceiling, *if* the device overlaps commands (most NVMe SSDs do).
2. **Transfer size per command:** PRP1 caps a command at one 4 KiB page. PRP2 extends
   to 8 KiB; a PRP list extends to 128 KiB+ (page list). Bigger transfers amortize the
   fixed per-command cost and reach the bandwidth limit at lower QD.

Link ceiling: PCIe Gen2 x4 ≈ 2.0 GB/s raw, ~1.5–1.6 GB/s usable after TLP overhead.
Stop optimizing when measurements are clearly bandwidth-bound (throughput flat vs QD
and vs transfer size, near the ceiling).

## 1. P1 — Firmware queue-depth pipelining

Goal: prove the QD ceiling on real hardware cheaply, before committing to RTL. Convert
the firmware I/O benchmark from submit-one→poll-one into a sliding window of QD
outstanding commands.

### Host-memory layout (extend current bring-up window)

Current single-buffer layout (from firmware): one IO SQ, one IO CQ, one RD buffer, one
WR buffer. For QD>1 we need QD distinct data buffers and a deeper SQ/CQ.

```
HOSTMEM_BASE                = 0x1000_0000
ASQ / ACQ / ID buffer       : unchanged (admin)
IO_CQ                       : depth >= QD+1 entries (16 B each), phase-tracked
IO_SQ                       : depth >= QD+1 entries (64 B each)
IO_BUF_BASE                 : QD data buffers, 4 KiB stride (page-aligned, PRP1-safe)
                              buf[i] = IO_BUF_BASE + i*0x1000
```

Make the IO queue depth a compile-time constant `IO_QD` (e.g. 32) and size IO_SQ/IO_CQ
to `IO_QD+1` so the ring never reports full-vs-empty ambiguously. The NVMe queue size
is set when creating the IO SQ/CQ (CDW10 qsize field) — update Create IO CQ/SQ to use
the larger size, and program the matching doorbell strides.

### Submission ring (firmware-owned, mirrors device SQ)

State:
```
sq_tail   : next SQ slot to fill (firmware → device); doorbell value
sq_head   : last slot the device has consumed (from CQE SQHD field), for free-slot calc
cq_head   : next CQE slot to inspect
cq_phase  : expected phase bit (toggles each time CQ wraps)
inflight  : number submitted but not yet completed
```

Per-command context (parallel arrays indexed by `cid`, cid == SQ slot index):
```
cmd_lba[i], cmd_active[i], (cmd_start_tick[i] if per-cmd latency wanted)
```

### Core loop (one measurement window of `count` I/Os)

```
submitted = 0
completed = 0
while completed < count:
    # 1. Fill the window.
    while inflight < IO_QD and submitted < count:
        i   = sq_tail
        lba = base_lba + submitted*step          # step=0 fixed-LBA, step=nlb sequential
        build_sqe(IO_SQ + i*64, op, nsid, lba, nlb, prp1=buf[i], cid=i)
        submitted += 1
        inflight  += 1
        sq_tail    = (sq_tail + 1) % SQ_SIZE
    ring_sq_doorbell(sq_tail)                     # ONE doorbell per batch (coalesced)

    # 2. Reap everything currently complete (phase-bit scan).
    while cqe_phase(IO_CQ + cq_head*16) == cq_phase:
        st   = cqe_status(IO_CQ + cq_head*16)
        sqhd = cqe_sqhd(IO_CQ + cq_head*16)
        accumulate_status(st)                      # OR errors, count completions
        sq_head    = sqhd
        completed += 1
        inflight  -= 1
        cq_head    = (cq_head + 1) % CQ_SIZE
        if cq_head == 0: cq_phase ^= 1
    ring_cq_doorbell(cq_head)                      # batch CQ head update
```

Key differences vs current code:
- Doorbell is rung **once per batch**, not once per command → fewer MMIO writes.
- Completion is **drained in bulk** by scanning consecutive phase-matching CQEs →
  amortizes the polling and the per-command bookkeeping.
- `inflight` lets up to `IO_QD` commands overlap the device latency.

### `nvme_bench` CLI

Add a trailing `qd` argument (default 1 = current behavior):
```
nvme_bench <read|write> <bar0> <nsid> <slba> <nlb> <count> <step> <warmup> [qd]
```
Keep the existing warmup + clean measurement window (snapshot counters after warmup).
Report, in addition to current fields: `qd`, effective `inflight` high-water,
doorbell count, cq poll loops (already tracked).

### Validation / sweep

For read and write, sweep `qd ∈ {1,2,4,8,16,32}` at nlb=8 sequential and fixed-LBA.
Expected: throughput rises with QD then plateaus. Record the curve in
`doc/NVME_PERFORMANCE.md`. The plateau identifies whether we are device-limited or
link-limited, which decides how hard P3 (bigger transfers) is needed.

### Gotchas
- CID must be unique among in-flight commands; using the SQ slot index as CID and
  sizing buffers per-slot guarantees that.
- The device reports `SQHD` in each CQE — use it (not an assumption) to know how many
  SQ slots are free, so we never overwrite an unconsumed SQE.
- Phase bit starts at 1 for a freshly created CQ; toggles on each wrap.
- Keep SQ/CQ in a memory region the device DMAs to coherently (the hostmem window).

## 2. P2 — RTL I/O command engine (`litenvme/io_engine.py`)

Goal: remove the CPU from the steady-state I/O path so QD can be sustained at line
rate without firmware polling. Firmware/admin still does controller enable + Create IO
queues; the engine owns the IO SQ/CQ ring at runtime.

### Reused existing blocks (confirm exact ports before wiring)
- `litenvme/mem.py::LiteNVMePCIeMmioAccessor` — doorbell writes (and any MMIO). Ports
  (per architecture notes): `start` (pulse), `we`, `adr[64]`, `wdata[32]`, `len`,
  `rdata[32]`, `done`, `err`. Doorbells are 32-bit posted writes.
- `litenvme/hostmem.py` responder — stores SQ/CQ + data buffers; has an AXI backend, a
  PCIe DMA frontend (the SSD's MemRd/MemWr land here), and a debug/CSR read port. The
  engine needs to (a) write 64 B SQEs into the SQ region and (b) read 16 B CQEs from
  the CQ region. Prefer driving these through the hostmem AXI backend port (beat-wide)
  rather than the narrow CSR debug port.

### Engine interfaces (proposed)
```
Request  (stream/sink, from user logic or CSR/AXI-Lite):
    op[8], nsid[32], lba[64], nlb[16], buf_addr[64]   # buf_addr = PRP1 (data location)
Completion (stream/source, to user logic):
    cid[16], status[16], (bytes[32] optional)
Config (CSR at elaboration / setup):
    bar0_base[64], io_sq_base[64], io_cq_base[64],
    sq_doorbell_off, cq_doorbell_off, qd, sqe phase init
```

### FSM sketch (single IO queue, QD outstanding)
```
state regs: sq_tail, sq_head, cq_head, cq_phase, inflight, free-CID allocator
- IDLE: if request available and inflight < qd and CID free -> SUBMIT
- SUBMIT:
    allocate cid; write 64B SQE to hostmem at io_sq_base + sq_tail*64
      (build 16 dwords: CDW0 op|cid, NSID, ..., PRP1=buf_addr, CDW10/11=lba,
       CDW12=nlb-1 | flags)
    sq_tail++ (mod SQ_SIZE); inflight++
    ring SQ doorbell (MMIO write bar0+sq_doorbell_off = sq_tail)  [posted]
    -> IDLE (allows back-to-back submits) and concurrently run COMPLETE scan
- COMPLETE (runs whenever inflight>0):
    read 16B CQE at io_cq_base + cq_head*16 from hostmem
    if cqe.phase == cq_phase:
        emit completion {cqe.cid, cqe.status}
        free cid; sq_head = cqe.sqhd; inflight--
        cq_head++ (mod CQ_SIZE); on wrap cq_phase ^= 1
        ring CQ doorbell (MMIO write bar0+cq_doorbell_off = cq_head) [posted]
```
Submit and complete should be concurrent (two cooperating FSMs sharing the ring
counters, or one FSM that interleaves) so submission isn't stalled by completion
scanning. The CQE poll is a periodic hostmem read of the next expected slot; gate its
rate so it doesn't starve the SSD's DMA write path into hostmem.

### Simulation
Extend `test/test_nvme_sequence.py` style: a model SSD that, on SQ doorbell, reads the
SQE from hostmem, performs the data MemWr/MemRd, writes a CQE (correct phase + sqhd),
and the test feeds K requests and checks K completions with right CIDs/status and that
`inflight` never exceeds `qd`.

### Bring-up in SoC
Wire behind a CSR `enable`. Keep firmware path available (mutually exclusive owner of
the IO queue) for A/B comparison. Measure IOPS/throughput vs the firmware QD sweep.

## 3. P3 — Larger transfers (PRP2 + PRP list)

- PRP2: if transfer crosses one page, CDW8/9 = PRP2 = second page address. Covers
  8 KiB (two pages) directly.
- PRP list: for >2 pages, PRP2 points to a page-aligned list of 64-bit page addresses
  in hostmem; the engine (or firmware) builds that list from the buffer base. Supports
  up to 512 entries/page → 2 MiB with one list page.
- Implement in firmware first (validate), then in the RTL engine's SQE builder. Buffers
  must be page-granular; the list build is a simple counted loop writing addresses.

## 4. P4 — Public core API (`litenvme/core.py`)

Make it drop-in for a LiteX/LitePCIe SoC, exposing SSD block access like other Lite
cores.

```python
class LiteNVMe(Module, AutoCSR):
    def __init__(self, pcie_endpoint, hostmem, *, qd=32, with_axi_stream=True):
        # - config/control via CSR (auto AXI-Lite): submit requests, read status,
        #   admin/setup trigger.
        # - data via AXI-Stream: write-data sink, read-data source, bridged to the
        #   hostmem data buffers by an internal DMA so users never touch PRP details.
        # - hostmem backend selectable: BRAM (litenvme.hostmem) or LiteDRAM (P5).
    # Helper methods for a block-style facade:
    #   read(lba, nlb, dst)  / write(lba, nlb, src)
```

Design intent (per user): "integrate into a litex/litex-pcie design to use an SSD and
have read/write access like a wishbone/axi interface." So the facade is a block engine:
the user issues (op, lba, length) and supplies/consumes data via AXI-Stream (or points
at a host-memory address); the core handles queues, doorbells, PRP, completion.

## 5. P5 — LiteDRAM host memory

Replace/augment the BRAM AXI backend in `litenvme/hostmem.py` with a LiteDRAM AXI/
native port so the SSD DMAs into DDR (large buffers, sustained high-QD streaming).
Keep BRAM as a fallback/sim backend. On the Alibaba KU3P demo, instantiate LiteDRAM
and point the hostmem responder's backend at it.

## 6. P6 — Demos

- Self-check: firmware/Etherbone command that writes a known pattern over an LBA range,
  reads it back into a separate buffer, verifies equality. Proves correctness of the
  QD/PRP/stream paths end to end.
- Max-perf: drive the engine at best QD + transfer size through the LiteX system,
  report sustained read and write GB/s, and identify the limiting factor.
