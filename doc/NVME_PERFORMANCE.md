# LiteNVMe Performance Notes

This document tracks the current measured performance of the bring-up design and
the most likely bottlenecks.

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
