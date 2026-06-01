# Gen3 x4 + 256-bit datapath — WORKING (2026-06-01)

Link: Gen3 x4 (link_status=0x20ad, rate=Gen3, width=x4, ltssm=0x10), RC MPS=512B, MRRS=512B.
All numbers below are board-printed and reproduced; errors=0 on every run.

## Throughput (nvme_engine_bench, QD=32, auto-MPS 512B)
| Test                 | Throughput  | errors |
|----------------------|-------------|--------|
| 4 KiB read (rd4k_a)  | 2198.3 MB/s | 0      |
| 4 KiB read (rd4k_b)  | 2201.2 MB/s | 0      |
| 8 KiB read           | 1410.3 MB/s | 0      |
| 4 KiB write (wr4k_a) | 2490.6 MB/s | 0      |
| 4 KiB write (wr4k_b) | 1190.3 MB/s | 0      |

4 KiB reads ~2.2 GB/s = ~2x the proven 128-bit Gen2 path (~1.0 GB/s).
Write throughput varies run-to-run (SSD-side), read is stable.

## Data integrity (distinctive pattern 0xDEADBEEF)
- write_readback @LBA 5600000: readback all `deadbeef` (distinct bytes -> no byte/dword shuffle).
- `VERIFY OK: LBA 5600000 .. 5600007 (pattern 0xdeadbeef)` (8 blocks x 128 dwords).

## Bugs fixed to get here (256-bit root-port path)
1. litepcie SAxisRQAdapter: handle 4DW (force_64b) memory requests (commit 704bf6b).
2. litenvme/mem.py MMIO accessor: consume completion through `end` beat + drain stale; plus
   firmware lag-safe MMIO reads (read-until-stable), mirroring cfg_rd32_retry.
3. litenvme/hostmem.py completer responder: place sub-beat MemWr data at the address's byte
   offset and strobe only the written dwords (admin CQE to ACQ+16 was landing in slot 0 and
   the un-masked stale half clobbered slot 1). Constant-shift Case, NOT a variable shift
   (`req_dat << (off_dw*32)` mis-synthesised to no-shift while simulating fine).
