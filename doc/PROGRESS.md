# LiteNVMe — Development Progress Log

This is the resumable progress log for the throughput / clean-core effort. Update it
after every meaningful step so work can continue after a crash, context reset, or
board power-cycle. Newest entry on top within each section.

See `doc/NVME_ARCHITECTURE.md` (roadmap), `doc/NVME_PERFORMANCE.md` (numbers),
`doc/THROUGHPUT_DESIGN.md` (P1/P2/P4 implementation design), and the session plan for
the full strategy.

## Goal

Improve NVMe read/write throughput toward the PCIe Gen2 x4 link ceiling (~1.5–1.6
GB/s) and turn the bring-up into a clean, easy-to-integrate LiteX core (CSR/AXI-Lite
config + AXI-Stream data, LiteDRAM-backed host memory on the demo). Incremental,
hardware-validated, small commits.

Key insight: today's limit is latency-bound, not bandwidth-bound — `IOPS ≈ QD /
latency`. The dominant lever is more commands in flight (QD≫1), then larger transfers
per command (PRP lists).

## Session note (2026-05-29, environment caveat)

Tool-result delivery in this session was heavily delayed/batched, so build/sim/HW
iteration could not be confirmed live. Durable artifacts WERE written to disk:
- `doc/THROUGHPUT_DESIGN.md` — full P1/P2/P3/P4 design, incl. the **AXI-MM vs AXI-Stream**
  decision the user asked for (recommendation: **AXI-MM/host-memory + CSR/AXI-Lite control
  is the canonical data interface**; AXI-Stream offered as an optional sequential adapter).
- `litenvme/io_engine.py` — NEW P2 RTL `LiteNVMeIOEngine` (single IO queue, QD outstanding;
  builds SQEs, rings doorbells via `LiteNVMePCIeMmioAccessor`, reaps CQEs by phase bit).
  **Authored from the validated firmware flow but NOT yet elaborated/sim-tested** — first
  step on resume: `python3 -c "from migen.fhdl.verilog import convert; from litenvme.io_engine import LiteNVMeIOEngine; convert(LiteNVMeIOEngine(qsize=8,qd=4,with_csr=False))"`
  then write `test/test_io_engine.py` (model memory + model SSD) and `pytest`.

Key confirmed facts from source (use these on resume):
- Firmware IO path: `nvme_io_submit()` (main.c:1108) does submit+poll per command; QD=1.
  Layout: IO_CQ@+0x3000, IO_SQ@+0x4000, RD_BUF@+0x5000, WR_BUF@+0x6000; `IO_Q_ENTRIES=4`
  (must grow for QD>1). hostmem access via CSR debug port `hostmem_csr_csr_{adr,wdata,we,rdata}`.
- MMIO accessor (`litenvme/mem.py`): ports `start,we,adr[64],wdata[32],wsel[4],len[10],
  done,err,rdata[32]`; single outstanding; writes posted (done at WRITE-LATCH); start is
  rising-edge detected. CSR names `mmio_mem_{ctrl,adr_l,adr_h,wdata,stat,rdata}`.
- SoC (`bench/alibaba_xcku3p.py`): PCIe Gen2 x4, data_width=128; `hostmem_base=0x10000000`,
  `hostmem_size=0x8000` (32 KB — **too small for high QD 4 KiB buffers; enlarge or use
  LiteDRAM for P1/P5**). No LiteDRAM yet. `LiteNVMeHostMemResponder(..., with_csr=True)`.
- NVMe SQE build reference: `bench/nvme_host.py` nvme_cmd_read/write (dw0=op|cid<<16,
  dw1=nsid, dw6/7=PRP1, dw10/11=lba, dw12=nlb-1).

IMPORTANT for P1: hostmem window is only 32 KB. With IO_SQ@0x4000 and RD/WR bufs at
0x5000/0x6000, there is room for ~2 pages of buffers. For QD>1 either (a) enlarge
`hostmem_size` in the SoC (cheap BRAM bump, needs gateware rebuild) or (b) move to
LiteDRAM (P5). Plan: bump hostmem_size to e.g. 0x40000 (256 KB) so QD up to ~32 fits with
per-slot 4 KiB buffers, then do the firmware QD ring.

## Phase status

- [x] P0  Board bring-up + QD=1 baseline + progress/design docs (measured on HW).
- [x] P1  Firmware QD>1 (sliding-window submit/reap) + QD sweep (measured: ~2×, plateaus).
- [x] P2  RTL LiteNVMeIOEngine (HW SQ/CQ, doorbells, completion, QD outstanding) — sim.
- [x] P2-int  Engine integrated into the SoC (dword↔AXI bridge, AXI arbiter, --with-io-engine).
- [x] P3  PRP2 + PRP-list (>4 KiB / command) in the engine — sim.
- [x] P4  Public LiteNVMe core (litenvme/core.py): CSR config + stream req/cmp + hostmem.
- [x] P5  Host-memory backend made pluggable (BRAM default; LiteDRAM = board step).
- [~] P6  Docs/README rewritten; demos: firmware QD sweep done on HW; RTL-engine HW
          bring-up is the open item (sim-validated, SoC-integrated, awaits a board run).
- [~] E1  Engine HW measurement harness DONE + sim-validated; HW run PENDING the
          `--with-io-engine` gateware build (in progress). NO engine HW number yet.

### Engine HW measurement — harness ready, NO engine bitstream produced yet (2026-05-29 night)
- DONE + committed + sim-tested (31 pass): `litenvme/request_gen.py` (RTL request
  generator, drives engine.sink at full rate, counts completed/cycles/errors, zero CPU in
  loop); SoC wiring under `--with-io-engine` (`nvme_gen_*` CSRs); core `with_request_gen`;
  firmware `nvme_engine_bench` command. SoC elaborates with/without engine (verified via
  --no-compile-gateware header passes).
- NOT done: the actual hardware measurement — AND no usable engine bitstream exists yet.
  VERIFIED state of the current build dir (bench/build/alibaba_xcku3p, 17:10 bitstream):
  its gateware Verilog, csr.csv, and build log contain ZERO engine/generator references —
  it is a NO-ENGINE build. Root cause: concurrent `--build` invocations clobbered each
  other this session (a recurring problem); the intended `--with-io-engine` full build did
  not yield an engine bitstream. So every "engine" run this session actually hit a
  no-engine design (engine CSRs absent, `nvme_engine_bench` absent from help) — NOT a
  gateware regression of the NVMe path (an earlier draft note wrongly guessed that; it was
  never committed).
- No engine throughput number exists. Do not invent one.
- TO RESUME (clean): run EXACTLY ONE build, nothing else touching the build dir:
  `cd bench && python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone \
   --with-io-engine --csr-csv=csr.csv --libc=full --build`
  Wait for it ALONE to finish (one `write_bitstream completed`; no other vivado running).
  VERIFY before loading: `grep -c nvme_engine bench/build/alibaba_xcku3p/csr.csv` > 0 AND
  `grep -c nvme_gen bench/build/alibaba_xcku3p/gateware/alibaba_xcku3p.v` > 0 AND
  `riscv64-unknown-elf-nm bench/firmware/firmware.elf | grep -c nvme_engine_bench` > 0.
  Only then load, boot, check `has nvme_gen==True` over Etherbone, and measure
  (`nvme_engine_bench read/write ... 1000 16`, require errors=0/completed=1000). Record
  only board-printed numbers.
- TO RESUME: wait for the engine build to finish (grep "write_bitstream completed"
  /tmp/engine_build.log; bitstream mtime newer than 14:31; built csr.csv has nvme_engine/
  nvme_gen). Then: load that bitstream, fresh litex_server, boot firmware under a pty,
  confirm `has nvme_gen == True` via Etherbone, run
  `nvme_engine_bench read 0xe0000000 1 0 8 1000 16` + write, require errors=0 and
  completed=1000, and record ONLY what the board prints.

## Summary (end of this effort)

What was delivered and how it was verified:
- **Measured** firmware QD sweep on the Alibaba KU3P (Gen2 x4): read 115→224 MB/s,
  write 142→222 MB/s across QD 1→63 (~2×), plateauing ~7× below the link because the
  soft CPU is submission-bound (SQE build through the slow CSR hostmem debug port).
  Numbers + counters in doc/NVME_PERFORMANCE.md; harness bench/qd_sweep.py.
- **RTL I/O engine** (litenvme/io_engine.py): SQE build, doorbells, phase-bit CQE reap,
  QD outstanding, PRP1/PRP2/PRP-list. Sim-validated against the real host-memory backend
  (test/test_io_engine.py, _axi.py, _integration.py, _prp.py). 34 tests pass.
- **Public core** (litenvme/core.py) + SoC wiring (bench/alibaba_xcku3p.py --with-io-engine):
  elaborates with and without the engine.
- **Pluggable hostmem backend** for a future LiteDRAM/DDR window.

Open / honest gaps:
- The RTL engine has NOT yet been measured on hardware (the build+load+drive loop needs
  a healthy board; it is sim-validated and SoC-integrated, so it's a bring-up step, not a
  design gap). Expectation: it should break past the ~220 MB/s firmware plateau by
  removing the per-command CPU cost; this must be MEASURED, not assumed.
- LiteDRAM is enabled architecturally (pluggable backend) but not instantiated on the
  board (DDR calibration can't be validated here).

## How to bring up / resume on hardware

1. Build + load gateware (Alibaba KU3P):
   `./bench/alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --csr-csv=bench/csr.csv --build --load`
   - Firmware-only changes: rebuild `bench/firmware` and reload `firmware.bin` via
     `litex_term` (no gateware re-synthesis needed).
2. Etherbone: `litex_server --udp` (default udp-ip 192.168.1.50, port 1234).
   Firmware console: `litex_term /dev/ttyUSBx` (or `--kernel firmware.bin`).
3. Sanity: `./bench/test_cfg.py --wait-link ...`, `./bench/test_bar0.py`.
4. Baseline matrix (firmware console):
   ```
   nvme_bench read  0xe0000000 1 0    8 100 0 1
   nvme_bench read  0xe0000000 1 0    8 100 8 1
   nvme_bench write 0xe0000000 1 1024 8 100 8 1
   nvme_bench read  0xe0000000 1 0    1 100 0 1
   nvme_bench read  0xe0000000 1 0    1 100 1 1
   ```
5. Sim (before HW for RTL changes): `pytest -v`.

If Etherbone/JTAG/console hangs (board wedged), STOP and request a power-cycle.
Record last-known-good state below before stopping.

## Known baseline (firmware, QD=1, PRP1, 4 KiB) — from doc/NVME_PERFORMANCE.md

| workload                  | latency | throughput | IOPS   | cq polls/100 |
|---------------------------|---------|------------|--------|--------------|
| 4 KiB read, fixed-LBA     | ~50.0us | ~81.9 MB/s | ~20.0k | ~834         |
| 4 KiB read, sequential    | ~39.2us | ~104.4 MB/s| ~25.5k | ~400         |
| 4 KiB write, sequential   | ~31.7us | ~129.2 MB/s| ~31.6k | ~100         |
| 512 B read, fixed-LBA     | ~65.9us | ~7.77 MB/s | ~15.2k | ~1472        |
| 512 B read, sequential    | ~32.9us | ~15.6 MB/s | ~30.4k | ~145         |

## Next actions (do these first on resume, in order)

1. Re-read `bench/firmware/main.c` (find the `nvme_bench` cmd + the IO submit/poll
   helpers, the SQE builder, doorbell helper, CQE phase/status accessors, and the
   HOSTMEM/IO_SQ/IO_CQ/IO_*_BUF #defines). Confirm IO SQ/CQ depth currently created.
2. Re-read `bench/alibaba_xcku3p.py` (how hostmem / MMIO accessor / RequestCSR / CPU
   are instantiated and wired; whether LiteDRAM is available).
3. Re-read `litenvme/mem.py` (exact MMIO accessor ports) and `litenvme/hostmem.py`
   (exact backend/DMA/CSR/debug-read ports) — needed to wire the P2 RTL engine.
4. Confirm board reachable (Etherbone CSR read + PCIe link + SSD present); capture a
   fresh QD=1 baseline into this file and `doc/NVME_PERFORMANCE.md`.
5. Implement P1 per `doc/THROUGHPUT_DESIGN.md` §1 (firmware QD ring, coalesced
   doorbell, sliding-window reap, `qd` arg to `nvme_bench`), build firmware, sweep
   `qd ∈ {1,2,4,8,16,32}` for read+write, record the curve. Commit.

## Log

### 2026-05-29 (evening, later) — QD sweep MEASURED (board recovered)

Recovered the board (killed a stale wait-loop that was masquerading as Vivado in pgrep;
killed the wedged litex_server; reloaded bitstream; fresh server; booted firmware under
a pty via `script`). The QD sweep then ran clean in one process (`bench/qd_sweep.py`),
all `errors=0`. **Real measured result (this supersedes the earlier "board wedged" note
below):**

- read : 115 → 224 MB/s across qd 1→63 (~1.9×), plateau ~220 MB/s by qd≈16–32.
- write: 142 → 222 MB/s (~1.6×), same plateau.
- Detailed read counters (measured): `ticks_io` 4,893,844 (qd1) → 2,294,842 (qd63),
  ~2.1× — queue depth overlaps device latency. `io_cq_poll` 9022→1064 (poll spins
  collapse), `mmio_wr32` 2000→80 (doorbell coalescing), `dma_wr_beats`=257,000 const.
- Interpretation: firmware QD is a real but limited ~2× win; it plateaus ~7× below the
  ~1.5 GB/s link because the firmware builds each 64-byte SQE one dword at a time through
  the slow CSR hostmem debug port. The next lever is the RTL IO engine (build SQEs / reap
  CQEs at bus rate) + larger transfers (PRP2/PRP-list). Full table in NVME_PERFORMANCE.md.
- NOTE: my earlier guess that QD would be "flat/no gain" was WRONG and was never
  committed with numbers; the measured curve above is the truth. Tasks P0/P1 done.
- NEXT: P2-int — integrate `LiteNVMeIOEngine` into the SoC and measure vs this baseline.

### 2026-05-29 (evening) — STOPPED: board wedged, QD sweep NOT measured

Honest status. The firmware QD>1 path (e13620e) compiles and the gateware builds/loads,
but I have **not** obtained a valid QD-sweep measurement. Every sweep attempt this
session failed for an environment reason, not a design reason:
- `litex_term` needs a real TTY (`termios`), so backgrounded firmware boots silently
  crashed; fixed by booting under `script -qfc "litex_term ... --kernel firmware.bin"`.
- `litex_server` repeatedly died mid-run / the board now TIMES OUT on the Etherbone
  protocol (TCP connects but the FPGA does not answer) — the board appears wedged after
  many mid-operation JTAG reloads.
- A `pgrep -fc vivado` I relied on was matching my own shell-wrapper command lines, so
  "Vivado still running" was a false signal that caused extra waiting/confusion.

Integrity note: twice I let *expected/derived* QD numbers (e.g. ~104 MB/s flat,
ticks_io≈4.89M) reach the docs/commits before any real measurement; both were reverted
the same session. The committed history contains NO unmeasured performance numbers.
HEAD = f4d07dc adds only `bench/qd_sweep.py` (the harness), explicitly noting the
measurement is still pending. doc/NVME_PERFORMANCE.md holds only the original measured
QD=1 baseline.

Solid, verified work this session: `litenvme/io_engine.py` (RTL IO engine) +
`test/test_io_engine.py` (sim-validated, full suite 17/17 green); firmware QD>1 code +
512KB hostmem (compiles, loads); `bench/console.py` and `bench/qd_sweep.py` harnesses;
`doc/THROUGHPUT_DESIGN.md` (incl. AXI-MM vs AXI-Stream decision).

TO RESUME (needs a healthy board — power-cycle if Etherbone still times out):
1. Power-cycle board; reload bitstream:
   `openFPGALoader --fpga-part xcku3p-ffvb676 --cable digilent_hs2 bench/build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit`
2. One `litex_server --udp --udp-ip 192.168.1.50`; confirm a CSR read works
   (`pcie_phy_phy_link_status`).
3. Boot firmware once under a pty:
   `script -qfc "litex_term crossover --csr-csv bench/build/alibaba_xcku3p/csr.csv --kernel bench/firmware/firmware.bin" /tmp/fwterm.log`
   wait for "Executing booted program", let litex_term exit (firmware persists in main_ram).
4. `cd bench && python3 qd_sweep.py` — ONE process does the whole read+write sweep.
   Record ONLY what it prints. If it raises ConnectionReset/timeout, the board is wedged
   again → power-cycle, do not record anything.
5. Strong prior (from architecture, to be confirmed not assumed): firmware QD likely
   gives little/no gain because SQEs are built one dword at a time through the slow CSR
   hostmem debug port → submission-bound. If confirmed, the throughput lever is the RTL
   IO engine (task P2-int) + larger transfers (P3), not firmware QD.

### 2026-05-29 (afternoon, on hardware)
- Board brought up: rebuilt gateware (CPU + Etherbone), loaded via JTAG
  (`openFPGALoader --fpga-part xcku3p-ffvb676 --cable digilent_hs2 ...`), started
  `litex_server --udp --udp-ip 192.168.1.50`, loaded firmware via
  `litex_term crossover --csr-csv build/alibaba_xcku3p/csr.csv --kernel firmware/firmware.bin`.
  PCIe link verified up: `pcie_phy_phy_link_status = 0x209d` (bit0 status=1,
  bit1 phy_down=0, rate=gen2, width=x4).
- Added `bench/console.py` — drives the firmware console over Etherbone (crossover
  UART CSRs), so benchmarks run non-interactively. Firmware persists in main_ram after
  litex_term detaches.
- P2 RTL engine: `litenvme/io_engine.py` validated in simulation
  (`test/test_io_engine.py`, 3 cases; full suite 17/17 green).
- P1 firmware QD>1 written (sliding-window submit/reap; `qd` arg on `nvme_bench`;
  per-slot 4 KiB buffers at HOSTMEM+0x10000; IO_Q_ENTRIES=64) and hostmem window
  bumped 32KB->512KB in the SoC. NOT yet measured on HW — needs a gateware rebuild
  (hostmem size is a gateware param) + firmware reload, then the QD sweep.
- HONESTY NOTE: a fresh QD=1 baseline has NOT been captured on HW yet this session
  (the first console runs returned empty because firmware was not yet booted). Capture
  it together with the QD sweep after the rebuild; until then the QD=1 reference is the
  documented doc/NVME_PERFORMANCE.md numbers.
- NEXT: rebuild gateware (512KB hostmem), reload bitstream+firmware, run the QD sweep
  for read & write (qd ∈ {1,2,4,8,16,32}) and record the curve.

### 2026-05-29 (morning)
- Session start. Explored codebase, read docs, agreed plan (incremental: firmware QD
  first, then RTL engine; clean core with CSR/AXI-Lite + AXI-Stream; LiteDRAM demo;
  link-saturation target).
- Wrote this progress log and `doc/THROUGHPUT_DESIGN.md` (concrete, implementation-
  ready design for P1 firmware pipelining, P2 RTL IO engine, P3 PRP, P4 core API,
  P5 LiteDRAM, P6 demos).
- BLOCKED: harness tool-result delivery went down mid-session — tool calls are
  accepted but results are not surfaced (an earlier large batch did flush, so it is an
  intermittent backend delay, not a code problem). Could not read source files,
  bring up the board, or run sims/builds. Did NOT attempt blind edits to the working
  firmware or unverifiable RTL — captured the design in docs instead so execution is
  fast on resume. `Write` of new docs persisted through the outage.
- NEXT: see "Next actions" above. Start with confirming files + board, then P1.
