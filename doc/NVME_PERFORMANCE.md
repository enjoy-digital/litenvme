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

Representative post-fix results:

### 1 block (512 B), first measured run after setup

- `ticks_setup`: about `12,928,991`
- `ticks_io`: about `693,171`
- `latency_avg`: about `55.5 us`
- `throughput`: about `9.23 MB/s`
- `iops`: about `18.0k`
- `mmio_rd_ticks`: about `6,330,764`
- `mmio_wr_ticks`: about `30,033`
- `admin_submit`: `3`
- `io_submit`: `100`
- `io_cq_poll_loops`: about `1053`

### 8 blocks (4096 B), steady-state

- `ticks_setup`: about `565`
- `ticks_io`: about `763,438`
- `latency_avg`: about `61.1 us`
- `throughput`: about `67.1 MB/s`
- `iops`: about `16.4k`
- `mmio_rd_ticks`: about `27,000`
- `mmio_wr_ticks`: about `28,200`
- `mmio_wr32`: `200`
- `io_submit`: `100`
- `io_cq_poll_loops`: about `1279`

## What the current data says

### 1. The old MMIO write-timeout bottleneck is gone

Steady-state latency dropped from about `16.8 ms` per I/O to about `55-61 us`
per I/O after fixing posted writes and ordering in firmware.

### 2. There is still meaningful fixed per-request overhead

`nlb=1`, `2`, `4`, and `8` all complete in almost exactly the same I/O time.
Only throughput changes, because payload size changes while request time stays
flat.

That means the current bottleneck is not payload transfer bandwidth. It is a
fixed control-path cost paid once per I/O.

### 3. Completion polling is now a visible cost

In steady state, `io_cq_poll_loops` is about `1279` for `100` I/Os, or about
`12.8` polls per request. That is no longer negligible.

### 4. The data path is still not the dominant cost

Host memory DMA beat counts scale with `nlb`, but total I/O time does not.

For reads:
- `hostmem_dma_wr_beats` grows from about `3300` to `25700`
- `ticks_io` stays essentially unchanged

So moving more payload data does not materially change request time in the
current range.

### 5. MMIO is no longer dominant in steady state

In the steady-state `nlb=8` case:

- `mmio_rd_ticks + mmio_wr_ticks` is about `55,200`
- `ticks_io` is about `763,438`

So MMIO still costs something, but it is no longer the main source of the
measured I/O latency.

The main remaining fixed cost is now more likely in:

- firmware CQ polling
- hostmem polling reads
- queue bookkeeping / command build / submit overhead

## Most likely improvement path

### Short term

1. Make warmup explicit in benchmark usage
   - Exclude setup/admin overhead from steady-state reporting.
   - Compare `nlb=1` and `nlb=8` with the same warmup count.

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
