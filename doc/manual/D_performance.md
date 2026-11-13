# Performance and resources {#performance}

This appendix gathers the performance and resource figures of the project: the bounds of the
datapath, computed, the throughputs measured on the bench and logged in the repository, and the
resource figures recorded in its documents. Computed values are labelled as such; every measured
value names its log. The status of each feature is in the [status table](#intro-status).

## Bounds {#perf-bounds}

Two bounds limit the throughput of one SSD on the reference configuration. The table computes
them; neither is a measurement.

| Bound | Computed from | Value |
|----------------------------------|----------------------------------------------|----------------|
| Datapath of the core | 256 bits at 125 MHz (`examples/alibaba_xcku3p.yml`) | 4000 MB/s |
| Gen3 x4 link, raw | 8 GT/s per lane, 4 lanes, 128b/130b coding | 3938 MB/s |

The link carries TLP headers and completions besides the data, so the usable rate is lower than
the raw rate. With an MPS of 512 bytes, a 4 KiB read reaches the host-memory window as eight
MemWr TLPs of data, plus one for the CQE.

## How the bench measures {#perf-method}

The firmware command `nvme_engine_bench` programs the request generator and the I/O engine, starts
the run and prints the counters of the hardware ([Bench, firmware and host tools](#examples)).
The request generator counts the system clock cycles from the start to the last completion; the
firmware derives the other figures from that count.

| Printed value | Formula |
|------------------------------|------------------------------------------------------------------|
| `throughput` | `payload_bytes * clk_freq / cycles`, in MB/s (10^6 bytes per second) |
| `iops` | `count * clk_freq / cycles` |
| `latency_avg` | `cycles / (count * clk_freq)`: the mean time between completions, not the latency of a command |
| `hostmem_rd_duty_pct` | Cycles a MemWr beat is offered to the window, over `cycles` |
| `max_inflight` | Largest `inflight` of the engine seen by the firmware while it polls |

Each run issues `count` commands of `nlb` sectors on sequential LBAs from `slba`, in steps of
`step` sectors: the arguments of `nvme_engine_bench [read|write] <bar0> <nsid> <slba> <nlb>
<count> <step>`.

The runs below use BAR0 at `0xe0000000`, namespace 1, 1000 commands and a step equal to the
transfer, 8 sectors for 4 KiB and 16 for 8 KiB: only the start LBA changes. The firmware gives
each queue slot a 4 KiB buffer, so the buffers of consecutive 8 KiB commands overlap; the runs
measure throughput, not data integrity.

## Gen3 x4, 256-bit datapath {#perf-gen3}

The logs of `bench/results/gen3_256b_2026-06-01/` were recorded by the harness scripts, with the
setup below.

| Item | Value |
|----------------------|----------------------------------------------------------------------|
| Board | Alibaba XCKU3P, bench SoC `bench/alibaba_xcku3p.py` |
| SSD | VID/DID `0x5427c0a9` (Identify model `CT500P310SSD8` in the Gen2 logs below) |
| Link | `link_status=0x20ad`: up, Gen3, x4 |
| MPS, MRRS | 512 bytes and 512 bytes after the firmware auto-MPS (`MPS auto-set: enc=2 (512B)`) |
| Queue depth | 32 (`max_inflight: 32`) |
| Commands per run | 1000 |

The root port supports 1024 bytes but the SSD 512 bytes, so the firmware programs 512 bytes on
both ends (`root_sup=3 ssd_sup=2` in the logs).

The two sessions below are two runs of the harness; all runs report `errors: 0`. They disagree on
several runs, and the logs do not identify the bitstream that each session loaded: they are
reported as logged, without a claim that the two sessions ran the same gateware.

Session of `throughput_8kib_default.log`:

| Run | Command | Start LBA | Throughput | Duty |
|-------------|----------------|-------------|----------------|-----------|
| `rd8k_a` | 8 KiB read | 1000000 | 2686.881 MB/s | 68.026 % |
| `rd8k_b` | 8 KiB read | 2000000 | 2685.655 MB/s | 67.995 % |
| `rd4k` | 4 KiB read | 1500000 | 1170.200 MB/s | 29.954 % |
| `wr8k_a` | 8 KiB write | 3000000 | 2624.167 MB/s | 5.371 % |
| `wr8k_b` | 8 KiB write | 4000000 | 1561.025 MB/s | 3.172 % |

Session of `throughput.log`:

| Run | Command | Start LBA | Throughput | Duty |
|-------------|----------------|-------------|----------------|-----------|
| `rd4k_a` | 4 KiB read | 1000000 | 2198.264 MB/s | 56.272 % |
| `rd4k_b` | 4 KiB read | 2000000 | 2201.165 MB/s | 56.355 % |
| `rd8k` | 8 KiB read | 0 | 1410.310 MB/s | 35.416 % |
| `wr4k_a` | 4 KiB write | 3000000 | 2490.575 MB/s | 10.152 % |
| `wr4k_b` | 4 KiB write | 4000000 | 1190.340 MB/s | 4.768 % |

Two reads of the same size in a session repeat within 0.2 %, while the other read size and the
writes differ by about a factor of two between runs and sessions. Within a session only the
command changes; `SUMMARY.md` notes that write throughput varies from run to run on the SSD
side.

The duty of a write counts the MemWr TLPs of the CQEs only: the SSD fetches write data with MemRd
requests.

`bench/results/gen3_256b_2026-06-01/SUMMARY.md` summarizes `throughput.log`, and
`integrity.log` records a write, read-back and verify of 8 sectors at LBA 5600000 with
`VERIFY OK`.

## Gen2 x4, 128-bit datapath {#perf-gen2}

The earlier runs on the same board used a Gen2 x4 link and a 128-bit datapath. They are kept for
comparison; the bench SoC of the default branch builds the Gen3 configuration only.

The firmware queue-depth sweep of `doc/archive/qd_sweep_result.txt` (run with `qd_sweep.py`, in
the same directory, and an earlier firmware) drives 4 KiB sequential commands from the soft CPU, without the I/O engine,
1000 commands per point, `err=0` on every point.

The file records neither the board nor the link; `doc/NVME_PERFORMANCE.md` states that it was
measured on the XCKU3P board at Gen2 x4. Its status is In development
([status](#intro-status)).

| Queue depth | Read | Write |
|-------------|--------------|--------------|
| 1 | 115.23 MB/s | 141.699 MB/s |
| 2 | 160.52 MB/s | 167.826 MB/s |
| 4 | 190.741 MB/s | 184.869 MB/s |
| 8 | 207.461 MB/s | 197.129 MB/s |
| 16 | 217.683 MB/s | 210.892 MB/s |
| 32 | 221.712 MB/s | 218.053 MB/s |
| 63 | 223.601 MB/s | 221.872 MB/s |

The sweep flattens at about 220 MB/s; `doc/NVME_PERFORMANCE.md` attributes the limit to the
firmware, which builds each SQE dword by dword through the debug registers of the window. The
I/O engine removes that cost. With the engine at queue
depth 32, `bench/results/archive/engine_hw_2026-06-01_readgap_pass1.log` (`LINK=0x209d`, Gen2
x4, SSD model `CT500P310SSD8`) records:

| Run | Command | Throughput | Duty |
|-------------|----------------------------|----------------|-----------|
| `r8_a` | 4 KiB read, LBA 0 | 1154.742 MB/s | 58.139 % |
| `r8_b` | 4 KiB read, LBA 0 | 844.890 MB/s | 42.477 % |
| `r8_lba9m` | 4 KiB read, LBA 9000000 | 1076.292 MB/s | 54.707 % |
| `r16_a` | 8 KiB read, LBA 0 | 1102.763 MB/s | 55.248 % |
| `w8_a` | 4 KiB write, LBA 0 | 1487.157 MB/s | 10.694 % |

These runs used an MPS of 128 bytes: a 4 KiB read took 33 MemWr TLPs (`hostmem_rd_tlps: 33000`
for 1000 commands), against 9 in the Gen3 logs at 512 bytes (`hostmem_rd_tlps: 9000`).

## Resources {#perf-resources}

The repository records resource figures in `doc/STANDALONE_CORE.md`, from Vivado runs whose logs
are not in the repository. They are reported here as recorded, for the XCKU3P
(`xcku3p-ffvb676-2-e`).

| Design | LUT | FF | Block RAM tiles | DSP | Recorded as |
|------------------------------------|---------|---------|-----------|------|--------------------------|
| Standalone core, example YAML | 11,577 | 14,408 | 133 | 0 | Out-of-context synthesis |
| Bench SoC, Etherbone and block streamer | 14,667 | 18,733 | 325 | 2 | Post-place |

The standalone figure was recorded on `examples/alibaba_xcku3p.yml` when it booted the BIOS
(`firmware: none`) without the debug registers of the window: firmware mode, block streamer,
256 KiB window. The example now uses `firmware: minimal` and keeps the debug registers; no figure
is recorded for it. The document breaks it down as 2,960 LUT, 6,303 FF and
22 block RAM tiles for the hard IP wrapper, and 1,695 LUT and 749 FF for the CPU.

It also records that the init sequencer instead of the CPU saves 2,560 LUT, 3,116 FF and 25 block
RAM tiles on the bench SoC.

The block RAM of the window is the main cost that a user can tune: `hostmem_size` sets it, and
the third read port of the debug registers replicates it (`with_hostmem_csr`,
[The host-memory window](#host-memory)). The example YAML notes that `ep_max_pending_requests` of
2 saves about 15 block RAM tiles and costs about 20 % of the read throughput, without a log.
