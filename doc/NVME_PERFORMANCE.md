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

Measured with the firmware-side benchmark:

```sh
nvme_bench read 0xe0000000 1 0 1 100 0
nvme_bench read 0xe0000000 1 0 2 100 0
nvme_bench read 0xe0000000 1 0 4 100 0
nvme_bench read 0xe0000000 1 0 8 100 0
```

Representative results:

### 1 block (512 B)

- `ticks_io`: about `210,001,357`
- `latency_avg`: about `16,800 us`
- `throughput`: about `0.030 MB/s`
- `iops`: about `59.5`
- `mmio_wr32`: `200`
- `io_submit`: `100`
- `io_cq_poll_loops`: `100`

### 8 blocks (4096 B)

- `ticks_io`: about `210,001,355`
- `latency_avg`: about `16,800 us`
- `throughput`: about `0.243 MB/s`
- `iops`: about `59.5`
- `mmio_wr32`: `200`
- `io_submit`: `100`
- `io_cq_poll_loops`: `100`

## What the current data says

### 1. The bottleneck is fixed per-request overhead

`nlb=1`, `2`, `4`, and `8` all complete in almost exactly the same I/O time.
Only throughput changes, because payload size changes while request time stays
flat.

That means the current bottleneck is not payload transfer bandwidth. It is a
fixed control-path cost paid once per I/O.

### 2. Completion polling is not the dominant cost

`io_cq_poll_loops = 100` for `100` I/Os means the completion queue is observed
as ready on the first poll iteration for nearly every request.

So the firmware is not spending most of its time spinning on CQE polling.

### 3. The data path is not the dominant cost

Host memory DMA beat counts scale with `nlb`, but total I/O time does not.

For reads:
- `hostmem_dma_wr_beats` grows from about `3300` to `25700`
- `ticks_io` stays essentially unchanged

So moving more payload data does not materially change request time in the
current range.

### 4. The bottleneck is the MMIO write path

In steady state, each I/O uses:
- one SQ tail doorbell write
- one CQ head doorbell write

The benchmark shows `mmio_wr32 = 200` for `100` I/Os, which matches those two
doorbell writes per request.

Since:
- CQ polling is not dominant,
- payload size is not dominant,
- and the fixed work per I/O is mostly doorbell handling,

the bottleneck is the latency of the current MMIO path:

- firmware -> CSR accessor
- CSR accessor -> PCIe MMIO transaction
- completion of that accessor transaction

The measured counters show that almost all steady-state time is spent inside
`mmio_wr32()`. Investigation of the MMIO accessor confirmed the root cause:

- MMIO reads correctly wait for a completion TLP.
- MMIO writes are posted transactions and should not wait for a completion.
- The accessor was waiting for a completion even on writes.
- As a result, each posted write ran until its watchdog timeout before returning.

Since each I/O performs two posted doorbell writes, the benchmark ended up
paying roughly one write-timeout for SQ tail and one write-timeout for CQ head
on every request.

The posted-write fix also exposed a second issue:

- CQ timeout was based on a fixed CPU loop count rather than elapsed time.
- Once the accidental MMIO-write delay disappeared, CQ polling started much earlier.
- The old loop-count timeout could now expire before the SSD completed the command.

The firmware submit path therefore needs a time-based CQ timeout, not just a raw
poll-loop budget.

## Most likely improvement path

### Short term

1. Use a time-based CQ timeout
   - Poll CQEs against elapsed timer ticks, not just a raw loop count.
   - Keep the loop counter as a diagnostic, not as the real timeout source.

2. Re-run the benchmark after the posted-write fix
   - Confirm that steady-state latency drops sharply.
   - Re-check whether another fixed cost becomes dominant.

3. Reduce firmware-side fixed overhead
   - Avoid repeated work in the submit path where possible.
   - Keep queue state hot and avoid unnecessary debug/clear operations.

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
- the measured limit is clearly not SSD bandwidth
- the present limit is the bring-up control path

This is a good point to treat the firmware path as the correctness baseline and
start reducing the control-path cost, first by measuring MMIO transaction time
directly and then by migrating the hot I/O path toward hardware ownership.
