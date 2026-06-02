# LiteNVMe Performance Notes

This document tracks the measured performance and the bottlenecks. The
**current** results are at the top; the older firmware-driven bring-up notes
below are kept for history.

---

## Current status — Gen3 x4 + 256-bit, RTL engine (2026-06)

Board Alibaba KU3P + Crucial CT500P310SSD8. Link **Gen3 x4**, datapath
**256-bit @ 125 MHz (4.0 GB/s)**, MPS **512 B** (SSD-capped), QD **32**,
hardware I/O engine. All numbers below are **board-printed, reproduced,
errors=0, integrity VERIFY OK** (distinctive 0xDEADBEEF pattern). See
`ARCHITECTURE.md` for the design and `bench/results/gen3_256b_2026-06-01/`.

| Transfer | Read | Write | read duty |
|---|---|---|---|
| 4 KiB | ~2.2 GB/s (1.2–2.2, SSD/LBA-variant) | ~1.2–2.5 GB/s | ~56% |
| **8 KiB (default)** | **~2.69 GB/s** (very consistent) | **~1.6–2.6 GB/s** | **~68%** |

This is **~2.4–2.7× the prior 128-bit Gen2 path** (reads ~1.0, writes ~1.5 GB/s).
8 KiB is the default transfer because it is both faster *and* far more consistent
than 4 KiB. Writes vary run-to-run (SSD-side cache/GC), reads at 8 KiB are stable.

### Where the read ceiling is (and what does NOT help)

- The 256-bit datapath is **4.0 GB/s** and is **not** the limit: the read-path
  duty cycle is ~56–68% with **<2% of cycles stalled by us** — we keep up with
  the data; the gaps are the SSD/PCIe completion cadence.
- **MPS cannot be raised:** the SSD's DevCap caps MaxPayloadSize at **512 B**, so
  read-data arrives in 512-byte MemWr TLPs. Programming DevCtl.MPS to 1024 B is
  clamped back to 512 B by the device; throughput is unchanged. MRRS is the same story.
- **512-bit datapath would give ~0 gain** on this link: width × user-clock is
  pinned by the link, so 512-bit @ 62.5 MHz = 256-bit @ 125 MHz = 4.0 GB/s, and
  the Gen3 x4 link itself caps at **~3.4 GB/s** usable. 512-bit only pays off with
  a faster link (Gen4 x4 / Gen3 x8).
- The one live lever on this link is **transfer size** (amortizes per-TLP /
  per-command overhead). Measured + modeled (per-cmd ≈ 83 cyc, per-512B-TLP ≈
  18.8 cyc):

  | Transfer | Throughput | duty | note |
  |---|---|---|---|
  | 4 KiB | 2.20 GB/s (meas) | 56% | |
  | 8 KiB | 2.69 GB/s (meas) | 68% | **default** |
  | 16 KiB | ~3.0 GB/s (est) | ~75% | needs URAM hostmem (see below) |
  | 32 KiB | ~3.2 GB/s (est) | ~80% | needs URAM hostmem, ~100% URAM |
  | ∞ | ~3.4 GB/s | ~85% | link ceiling |

### Getting >8 KiB transfers (deferred)

The PRP-list path for >2-page transfers **works in simulation**
(`test/test_io_engine_prp.py`, nlb=24/64). It does **not** run on HW today purely
because the **512 KiB BRAM hostmem window** cannot hold QD=32 large buffers
(16 KiB × 32 = 512 KiB) **plus** the engine's 64-slot PRP-list region (256 KiB).
BRAM is 90% full (325/360) so the window cannot grow there. The fix is to move
the hostmem backing store to **URAM** (0/48 used) and enlarge it — a hostmem
backend refactor (3-port → 2-port for URAM + URAM primitive + layout), worth
~+12% (16 KiB ≈ 3.0 GB/s). Deferred by decision; 8 KiB is the shipped default.

---

## Historical bring-up notes (firmware-driven, superseded)

## Current configuration

- Firmware-driven queue management on VexRiscv.
- Single outstanding I/O.
- PRP1 only.
- Host memory backed by the bring-up `hostmem` window.
- NVMe queue setup and completions handled through firmware polling.

This is not the target high-performance architecture. It is a bring-up path
used to validate protocol correctness and expose the slow parts before moving
queue ownership and DMA management into more capable hardware blocks.

## Baseline measurements

Measured with the firmware-side benchmark. Use a warmup run so setup/admin
overhead is not mixed into steady-state I/O timing:

```sh
nvme_bench read 0xe0000000 1 0 1 100 0 1
nvme_bench read 0xe0000000 1 0 8 100 0 1
```

Representative post-fix results before the benchmark window cleanup showed:

- steady-state latency on the order of `55-61 us`
- steady-state throughput around `67-75 MB/s` for 4 KiB reads
- steady-state IOPS around `16k-18k`

The benchmark now snapshots counters after setup and warmup, so the next rerun
should be used to refresh the detailed per-counter baseline without setup/admin
pollution.

Clean steady-state baselines (`warmup=1`):

```sh
nvme_bench read 0xe0000000 1 0 8 100 0 1
nvme_bench read 0xe0000000 1 0 8 100 8 1
nvme_bench write 0xe0000000 1 1024 8 100 8 1
nvme_bench read 0xe0000000 1 0 1 100 0 1
nvme_bench read 0xe0000000 1 0 1 100 1 1
```

### 4 KiB read, fixed LBA (`step=0`)

- `ticks_setup`: about `12,751,823` on the first run, then about `569`
- `ticks_io`: about `625,067`
- `latency_avg`: about `50.0 us`
- `throughput`: about `81.9 MB/s`
- `iops`: about `20.0k`
- `io_cq_poll_loops`: about `834`

### 4 KiB read, sequential (`step=8`)

- `ticks_setup`: about `569`
- `ticks_io`: about `490,296`
- `latency_avg`: about `39.2 us`
- `throughput`: about `104.4 MB/s`
- `iops`: about `25.5k`
- `io_cq_poll_loops`: about `400`

### 4 KiB write, sequential (`step=8`)

- `ticks_setup`: about `569`
- `ticks_io`: about `396,184`
- `latency_avg`: about `31.7 us`
- `throughput`: about `129.2 MB/s`
- `iops`: about `31.6k`
- `io_cq_poll_loops`: about `100`

### 512 B read, fixed LBA (`step=0`)

- `ticks_setup`: about `569`
- `ticks_io`: about `823,472`
- `latency_avg`: about `65.9 us`
- `throughput`: about `7.77 MB/s`
- `iops`: about `15.2k`
- `io_cq_poll_loops`: about `1472`

### 512 B read, sequential (`step=1`)

- `ticks_setup`: about `569`
- `ticks_io`: about `410,823`
- `latency_avg`: about `32.9 us`
- `throughput`: about `15.6 MB/s`
- `iops`: about `30.4k`
- `io_cq_poll_loops`: about `145`

The current best steady-state baseline for the firmware-driven,
single-outstanding, PRP1-only bring-up path is sequential 4 KiB write:

- about `31.7 us`
- about `31.6k IOPS`
- about `129.2 MB/s`

## Queue-depth sweep (firmware, PRP1, 4 KiB sequential) — measured 2026-05-29

The firmware `nvme_bench` was extended with a sliding-window queue-depth path
(`nvme_io_bench_window`): submit up to `qd` commands with distinct per-slot 4 KiB PRP1
buffers, ring the SQ doorbell once per batch (coalesced), then drain all phase-matching
CQEs and ring the CQ doorbell once per batch; refill as slots free. Driven from the host
with `bench/qd_sweep.py` (single Etherbone process over the crossover UART).

Measured on the Alibaba KU3P board (PCIe Gen2 x4), 4 KiB sequential, `count=1000`, all
runs `errors=0`:

| QD | read MB/s | read kIOPS | write MB/s | write kIOPS |
|----|-----------|------------|------------|-------------|
| 1  | 115.2     | 28.1       | 141.7      | 34.6        |
| 2  | 160.5     | 39.2       | 167.8      | 41.0        |
| 4  | 190.7     | 46.6       | 184.9      | 45.1        |
| 8  | 207.5     | 50.6       | 197.1      | 48.1        |
| 16 | 217.7     | 53.1       | 210.9      | 51.5        |
| 32 | 221.7     | 54.1       | 218.1      | 53.2        |
| 63 | 223.6     | 54.6       | 221.9      | 54.2        |

(Run-to-run variance is a few %, e.g. a repeat measured QD=16 read at ~263 MB/s; the
table above is one full sweep.)

### What the QD sweep shows

1. **Queue depth helps, ~1.6–1.9×, then plateaus.** Read rises 115 → 224 MB/s and write
   142 → 222 MB/s across QD 1→63, flattening by QD≈16–32 at ~220 MB/s.

2. **It overlaps device latency (so QD=1 was partly latency-bound).** Detailed read
   counters (`count=1000`, 4 KiB → 32 beats/cmd at 128-bit → ~25,700 beats expected;
   `dma_wr_beats` is reported as the firmware's per-run counter delta, 257,000):

   | QD | ticks_io  | io_submit | io_cq_poll | mmio_wr32 | dma_wr_beats |
   |----|-----------|-----------|------------|-----------|--------------|
   | 1  | 4,893,844 | 1000      | 9022       | 2000      | 257,000      |
   | 4  | 2,685,663 | 1000      | 1511       | 1005      | 257,000      |
   | 16 | 2,358,879 | 1000      | 1139       | 253       | 257,000      |
   | 63 | 2,294,842 | 1000      | 1064       | 80        | 257,000      |

   `ticks_io` drops ~2.1× from QD=1 (4.89M) to QD=63 (2.29M) — queue depth overlaps the
   device completion latency. Two effects compound: CQ-poll spins collapse
   (`io_cq_poll` 9022 → ~1064, i.e. ~9 spins/cmd waiting for a completion at QD=1 down to
   ~1/cmd once commands are pipelined), and doorbell writes coalesce hard
   (`mmio_wr32` 2000 → 80, from ~2/cmd to far less than 1/cmd as the batch fills). Payload
   moved (`dma_wr_beats`) is constant, confirming identical work per run.

3. **But it plateaus ~7× below the link.** ~220 MB/s is far from the usable PCIe Gen2 x4
   ceiling (~1.5–1.6 GB/s). At QD=63, `ticks_io ≈ 2.29M / 1000 ≈ 2295` cycles/command
   (~18.4 µs) — and CQ polling and doorbells are no longer the cost (both collapsed
   above). What remains is the firmware's serial per-command submit work: building each
   64-byte SQE in host memory one dword at a time through the **CSR debug
   read-modify-write port** (`hostmem_csr_csr_*`, 16 writes/SQE). The soft CPU cannot
   construct SQEs fast enough to fill the link.

### Conclusion / next lever

Firmware queue depth is a real but limited win (~2×). To approach the link, the
steady-state path must build SQEs / manage the SQ-CQ rings **in hardware at bus rate**,
removing the per-command CPU cost. That is the role of the RTL I/O command engine
(`litenvme/io_engine.py`, sim-validated in `test/test_io_engine.py`): it writes SQEs
through the hostmem AXI backend and reaps CQEs in a tight FSM. A secondary lever is
larger transfers per command (PRP2 / PRP list, >4 KiB) to amortize the fixed
per-command construction cost.

Reproduce (board up, `litex_server --udp` running, firmware booted), from `bench/`:

```sh
python3 qd_sweep.py            # full read+write sweep over qd in {1,2,4,8,16,32,63}
```

## Read ceiling: link/protocol-bound, not the engine or the SSD (measured 2026-06-01)

After the pipelined-write fix took 4 KiB reads to ~1.0-1.15 GB/s, on-chip read-path duty-cycle
counters were added to the hostmem DMA (`wr_present/accept/stall_cycles`, `rd_tlp_count`) to
settle where the remaining gap to the ~1.5 GB/s Gen2 x4 ceiling lives. NVMe read data arrives
as MemWr TLPs, so these measure how the device/PCIe feeds us vs whether we backpressure it.
Two reproduced passes, errors=0, max_inflight=32 (evidence:
`bench/results/engine_hw_2026-06-01_readgap_pass{1,2}.log`):

- **We never backpressure reads:** `hostmem_rd_stall_cycles` ≈ 0 (tens-to-low-thousands out of
  250k-600k cycle windows). Our datapath keeps up; the earlier write-path pipelining holds.
- **The port is idle ~half the time:** `hostmem_rd_duty_pct` ≈ 40-58% for 4 KiB, ≈ 55-57% for
  8 KiB. Read-data beats are present only about half the run — the SSD/PCIe delivers completions
  with gaps, not back-to-back.
- **The SSD is not the limit:** Identify reports `CT500P310SSD8` (Crucial P310 500 GB, native
  Gen4 x4), rated **6,600 MB/s** sequential read / 520K random-4K IOPS (~2.1 GB/s) — ~4× our
  Gen2 x4 usable ceiling. The media can far exceed 1.5 GB/s.

**Conclusion:** the ~1.0-1.15 GB/s 4 KiB read figure is **link/protocol-bound** — PCIe Gen2 x4
(2.0 GB/s raw, ~1.5 GB/s after TLP overhead) plus read round-trip latency and per-TLP
completion-header overhead at 4 KiB. Writes are posted and already saturate the link; reads are
latency-exposed. The remaining levers are protocol-level (larger MaxReadRequest/MaxPayload,
more outstanding read bytes), with diminishing returns against the hard ~1.5 GB/s wall — there
is no throughput left to recover in our RTL datapath. (Caveat: the `hostmem_rd_gap_max` counter
printed a garbage value — it free-runs before the first beat; the duty/stall metrics the
conclusion rests on are sound.)

## RTL I/O engine on hardware (measured 2026-05-31)

The `LiteNVMeIOEngine` now runs on the Alibaba KU3P board (PCIe Gen2 x4) with a real NVMe
SSD attached, driven by `nvme_engine_bench` (firmware kicks the engine + on-chip generator
and reads back the engine's own counters — no host-channel arithmetic). Captured with the
single-reader crossover-UART console (`bench/engine_console.py`, ANSI-stripped prompt gate)
so every line below was literally printed by the board at a verified `litenvme>` prompt.
Bring-up gate for both passes: `ping 0% loss`, `pcie_phy_link_status=0x209d`, CSR
write/read roundtrip exact (double-read).

**Functional result — rock solid (this is the headline).** Across BOTH passes and ALL six
configs (read/write × 512 B / 4 KiB / 8 KiB), every single run reported:

    completed: 1000    errors: 0    last_cqe_status: 0x0000/0x0001 (valid phase bit)

That is 16/16 clean runs at `count=1000`. The engine builds SQEs, rings the doorbell, and
reaps CQEs entirely in RTL, and the SSD completes every command with no errors. The diag
command corroborates: `final sub=N cmp=N err=0`, `seqread selftest: OK`, `sentinel CSR path
OK`. (Note: the engine's `submitted` CSR is a free-running cumulative counter — it reads
1020, 2020, … 16020 across the session, i.e. exactly +1000 per run after the 20 left from a
prior diag+bench; `completed`/`errors` are per-run. There is no over-submission.)

**Throughput — two literal passes, shown raw.** Firmware computes throughput as
`payload_bytes / (cycles / 125e6)`, i.e. engine command-processing time over real
completions. Both passes, exactly as printed (MB/s):

| config            | pass 1 (MB/s)        | pass 2 (MB/s)        | reproducible? |
|-------------------|----------------------|----------------------|---------------|
| read  512 B  nlb1 | 197.703              | 67.646               | NO (≈3×)      |
| read  4 KiB  nlb8 | 463.153 / 453.624    | 453.172 / 438.104    | yes (~440–463)|
| read  8 KiB nlb16 | 480.723              | 479.719              | yes (~480)    |
| write 512 B  nlb1 | 602.835              | 601.673              | yes (~602)    |
| write 4 KiB  nlb8 | 1521.554 / 1518.508  | 1522.545 / 1196.742  | mostly (~1520, 1 outlier) |
| write 8 KiB nlb16 | 1012.066             | 1577.892             | NO (≈1.5×)    |

**How to read these numbers honestly:**

1. **They are engine throughput over real CQEs, not validated sustained media bandwidth.**
   Writes far exceeding reads (4 KiB write ~1520 vs read ~455 MB/s) and several points at or
   above the usable Gen2 x4 ceiling (~1.5 GB/s) mean the write path is write-cache-acked and
   the repeated sequential-from-LBA-0 reads are partly read-cache-served. This benchmarks the
   engine's control path against a (partly cached) device, which is the thing we are
   optimizing — but it is NOT a claim about the SSD's steady-state NAND bandwidth.

2. **Two configs are not reproducible** (512 B read 67–198; 8 KiB write 1012–1578). The
   firmware-computed throughput is sensitive to SSD completion jitter within the measurement
   window. Do not quote a single headline number for those.

3. **The one clean comparative win:** 4 KiB read is reproducibly ~440–463 MB/s, ~2× the
   firmware queue-depth plateau (~224 MB/s, table above) and ~4× firmware QD=1 (115 MB/s).
   The RTL engine builds and reaps commands faster than the soft CPU, as intended.

**End-to-end data integrity: VERIFIED (round-trip, 2026-05-31).** A host-driven round-trip
proves the engine moves bytes correctly through the SSD, not just that commands complete
(`bench/hw_integrity2.sh` + `bench/hostmem_tool.py`; raw evidence
`bench/results/engine_hw_2026-05-31_integrity.log`):

1. Engine writes LBA 0 (`completed=1, errors=0`). The firmware bench fills the host buffer
   with a uniform `0xa5a5a5a5` (it overwrites any host pre-seed), so that is the pattern
   sent to the SSD — confirmed by reading the buffer back over the `hostmem_csr` debug port.
2. Host clobbers the buffer to `0xDEAD0000+i` and confirms the clobber took.
3. Engine reads LBA 0 back (`completed=1, errors=0`); the buffer returns to `0xa5a5a5a5`.
4. Read-back == written (`VERDICT_EQUAL=PASS`, 0/38 sampled dwords mismatched) AND read-back
   != clobber (`VERDICT_DIFFER=PASS`, 38/38 differ).

Because the buffer was clobbered *between* the write and the read (and the clobber was
confirmed), the recovered pattern could only have come from the SSD — so the engine's write
(host→SSD) and read (SSD→host) data paths are both real and correct. *Caveat:* the written
pattern is uniform, so this proves round-trip correctness but not per-offset/LBA addressing;
a distinct-per-block pattern needs a firmware-bench change.

**Still TODO:** distinct-per-block write data, a cache-busting LBA spread (so reads aren't
partly cache-served), and an engine-side QD sweep. Trustworthy statements today: *the RTL
engine is functional and data-correct on hardware (1000/1000, errors=0, reproduced;
integrity round-trip PASS), and its 4 KiB read command throughput is ~2× the firmware path.*

Reproduce (board up, firmware already at `litenvme>` prompt, from `bench/`):

```sh
./hw_measure.sh        # 8-command read+write battery, count=1000 -> /tmp/meas.log
```

## What the current data says

### 1. The old MMIO write-timeout bottleneck is gone

Steady-state latency dropped from about `16.8 ms` per I/O to about `55-61 us`
per I/O after fixing posted writes and ordering in firmware.

### 2. Access pattern now clearly matters

Sequential traffic is substantially faster than fixed-LBA traffic.

Examples:
- 4 KiB read: about `81.9 MB/s` fixed-LBA vs about `104.4 MB/s` sequential
- 512 B read: about `15.2k IOPS` fixed-LBA vs about `30.4k IOPS` sequential

So the measured limit is no longer purely firmware overhead. Device-side
completion behavior and access pattern now affect the results.

### 3. There is still meaningful fixed per-request overhead

`nlb=1`, `2`, `4`, and `8` all complete in almost exactly the same I/O time.
Only throughput changes, because payload size changes while request time stays
flat.

That means the current bottleneck is not payload transfer bandwidth. It is a
fixed control-path cost paid once per I/O.

### 4. Completion polling is now a visible cost

In the clean steady-state baseline, `io_cq_poll_loops` is about `834` for
`100` I/Os, or about `8.3` polls per request. That makes firmware CQ polling a
visible part of the remaining control-path cost.

The poll count also tracks the access pattern:

- 4 KiB read fixed-LBA: about `834`
- 4 KiB read sequential: about `400`
- 512 B read fixed-LBA: about `1472`
- 512 B read sequential: about `145`

That strongly suggests completion arrival timing, not just firmware instruction
count, is contributing to the observed differences.

### 5. The data path is still not the dominant cost

Host memory DMA beat counts scale with `nlb`, but total I/O time does not.

For reads:
- `hostmem_dma_wr_beats` grows from about `3300` to `25700`
- `ticks_io` stays essentially unchanged

So moving more payload data does not materially change request time in the
current range.

### 6. MMIO is no longer dominant in steady state

Post-fix measurements showed that MMIO timing was only a fraction of the total
steady-state I/O timing. MMIO still costs something, but it is no longer the
main source of the measured latency.

The main remaining fixed cost is now more likely in:

- firmware CQ polling
- hostmem polling reads
- queue bookkeeping / command build / submit overhead

## Recommended comparison runs

To compare steady-state access patterns:

```sh
nvme_bench read 0xe0000000 1 0 8 100 0 1
nvme_bench read 0xe0000000 1 0 8 100 8 1
nvme_bench write 0xe0000000 1 1024 8 100 8 1
nvme_bench read 0xe0000000 1 0 1 100 0 1
nvme_bench read 0xe0000000 1 0 1 100 1 1
```

- `step=0` keeps hitting the same LBA range
- `step=8` walks sequentially in 4 KiB steps
- `step=1` walks sequentially in 512 B steps

This is useful to confirm whether the current limit is still dominated by the
firmware/control path rather than by media or caching effects.

## Most likely improvement path

### Short term

1. Refresh the baseline with the cleaned-up benchmark window
   - Re-run with explicit warmup.
   - Record counters that now exclude setup/admin overhead.

2. Reduce firmware-side fixed overhead
   - Avoid repeated work in the submit path where possible.
   - Keep queue state hot and avoid unnecessary debug/clear operations.

3. Reduce CQ polling cost
   - Consider less aggressive firmware polling patterns.
   - Consider interrupt or hardware-owned completion handling later.

4. Batch or decouple request submission
   - Add a small request FIFO.
   - Let firmware consume queued requests without CSR submission gaps.

### Medium term

1. Move queue ownership out of firmware
   - SQ/CQ head/tail handling in RTL.
   - Doorbell generation in hardware.

2. Keep firmware for setup/debug only
   - Admin init and bring-up are acceptable in firmware.
   - Steady-state I/O should move to dedicated logic if higher IOPS is required.

3. Add larger transfer support
   - PRP2
   - PRP lists
   - multiple outstanding requests

## Practical takeaway

The current benchmark is doing its job:

- protocol flow looks correct
- the benchmark numbers are now self-consistent
- the measured limit is no longer a fake MMIO timeout artifact
- the present limit is still the bring-up control path, but now much closer to the real one

This is a good point to treat the firmware path as the correctness baseline and
start reducing the control-path cost, first by measuring MMIO transaction time
directly and then by migrating the hot I/O path toward hardware ownership.
