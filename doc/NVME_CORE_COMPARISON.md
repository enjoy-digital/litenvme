# LiteNVMe vs. Other NVMe Cores

How does LiteNVMe compare to other publicly available NVMe FPGA cores, and is its throughput
near-optimal? This note collects what is **publicly published** by other cores and places
LiteNVMe's measured numbers next to them.

> **Read this first — the comparison is rough on purpose.** The numbers below come from vendor
> datasheets, demo write-ups and one academic paper. They use **different FPGAs, different
> SSDs, different PCIe generations, and different (sometimes marketing) measurement methods**.
> They are **not** an apples-to-apples benchmark. Treat them as order-of-magnitude context, not
> a leaderboard. Where a number is not published, it says so rather than guessing.

## Two different problems: host vs. device

NVMe FPGA cores fall into two categories that are easy to confuse:

- **Host / initiator** (what LiteNVMe is): a PCIe **RootPort** that drives a *commercial* SSD —
  it issues NVMe commands and moves data to/from a user buffer. Throughput is bounded by the
  PCIe link **and the attached SSD**.
- **Device / target** (e.g. OpenExpress, Cosmos+ OpenSSD): the FPGA **is** the SSD controller
  (with its own DRAM and flash channels). Throughput is bounded by the controller + media, and
  these typically quote aggregate multi-channel bandwidth.

Only **host-side** cores are directly comparable to LiteNVMe.

## Comparison table

| Core | Side | Open source | PCIe (as tested) | Read | Write | Data interface / CPU | Source |
|------|------|-------------|------------------|-----:|------:|----------------------|--------|
| **LiteNVMe** | Host | **Yes** (BSD, LiteX/Migen) | Gen3 x4, 256-bit | **~2.69 GB/s** | **~2.74 GB/s** | block streamer (sector/count + AXI-Stream); embedded soft-CPU for bring-up only | measured (this repo) |
| IntelliProp IPC-NV164-HI | Host | No | Gen3 (KCU105) | 2.0 GB/s | 1.0 GB/s | RTL state-machine *or* processor reg; user buffer (BRAM/DDR); CPU-less | FPGA Developer demo |
| IntelliProp IPC-NV164-HI | Host | No | Gen3 (faster platform) | ~3.2 GB/s | ~2.3 GB/s | same | IntelliProp |
| Design Gateway NVMe-IP | Host | No | **Gen5** (Versal HBM) | >11 GB/s | n/p | control + FIFO data port, 256 KB internal RAM, **no CPU, no DDR** | dgway.com |
| iWave NVMe Host Controller IP | Host | No | scalable (Gen4/x8) | up to 7 GB/s | n/p | n/p | iWave |
| OpenExpress (KAIST) | **Device** | Yes (non-commercial) | — | ~7 GB/s aggregate | n/p | full SSD controller + DRAM + channels | USENIX ATC '20 |

`n/p` = not published.

## Is LiteNVMe optimal?

**For Gen3 x4 against a single commercial SSD: yes, it is close to the practical ceiling.**

- The Gen3 x4 link delivers ~3.9 GB/s raw and **~3.4 GB/s usable** after TLP overhead. LiteNVMe's
  ~2.69 GB/s read / ~2.74 GB/s write is **~79–81 % of usable link bandwidth**.
- The remaining gap is **not** the core: this SSD caps **MaxPayloadSize at 512 B**, so read data
  arrives as 512-byte TLPs, and the host-memory responder runs at ~90 % duty during reads —
  i.e. the design is **device/link-bound, not core-bound** (see `doc/NVME_PERFORMANCE.md`). Any
  host core attached to the same SSD hits the same wall.
- Against the most comparable core (IntelliProp's CPU-less host accelerator on a Gen3 board),
  LiteNVMe's read (2.69) sits between IntelliProp's KCU105 result (2.0) and its best-platform
  result (3.2), and its **write (2.74) is markedly higher** than IntelliProp's 1.0–2.3 GB/s.

**Where the real headroom is: PCIe generation, not the core.** The 7–11 GB/s figures (Design
Gateway Gen5, iWave Gen4/x8) come from **faster/ wider PCIe**, not a fundamentally different
host architecture. LiteNVMe's datapath is already parameterized on `data_width`; reaching those
numbers is a Gen4/Gen5 PHY + lane-count upgrade (and a wider datapath), which the LitePCIe PHY
layer supports in principle — it is future work here, not an architectural limit.

## Resource footprint context

LUT/FF are **never published** by the commercial cores, so a full logic comparison isn't
possible — but LiteNVMe's own core-only numbers are exact. The standalone `litenvme_core`
(block-streamer product config, `examples/alibaba_xcku3p.yml`) was synthesized on its own
(out-of-context, XCKU3P, Gen3 x4 / 256-bit); the Vivado hierarchy gives:

| Core | LUT | FF | BRAM tiles | URAM | DSP | Data buffer / CPU |
|------|----:|---:|-----------:|-----:|----:|-------------------|
| **LiteNVMe** standalone core | **11,577** | **14,408** | **133** | 0 | 0 | 256 KB window + soft-CPU |
|  ├ PCIe hard-IP wrapper (+GTY)| 2,960 | 6,303 | 22 | 0 | 0 | — |
|  ├ VexRiscv bring-up CPU      | 1,695 |   749 |  0 (+2 RAMB18) | 0 | 0 | ROM/RAM ≈ 34 BRAM |
|  └ NVMe datapath + streamer + 256 KB window + glue | ~6,922 | ~7,356 | ~106 | 0 | 0 | window ≈ 64, glue ≈ 24 |
| Design Gateway NVMe-IP (std)  | n/p | n/p | 66 | 0 | n/p | 256 KB RAM, **CPU-less** |
| Design Gateway NVMe-IP (URAM) | n/p | n/p | 2  | 8 | n/p | 256 KB RAM, **CPU-less** |
| IntelliProp / iWave           | n/p | n/p | n/p| n/p | n/p | user-defined (BRAM/DDR) |

(Post-synthesis estimate; post-place would be slightly lower.)

### Understanding the BRAM difference

An earlier build of this core used **325 BRAM tiles (90 % of the device)** — clearly out of line
with Design Gateway's 66. Investigating *why* found two fixable causes, now addressed:

1. **A debug-read port was replicating the whole window (~2×).** The host-memory BRAM already
   uses a read+write (true-dual-port) pair for the data path; the CSR memory-readback feature
   added a *third* port, which Vivado can only satisfy by **duplicating the entire array**. That
   feature is debug-only and is now **off by default** for the pin-driven standalone core (it
   stays on the test SoC, where firmware `nvme_verify` needs it). → removes ~half the window BRAM.
2. **The default window was oversized.** 512 KiB included 256 KiB of per-slot buffers that only
   the *request generator* uses — the block streamer doesn't. The standalone core now defaults
   to a **256 KiB window** (queues + PRP region + a 192 KiB staging buffer) with a small ring
   (`qsize=8`), matching Design Gateway's 256 KiB data-buffer class.

Result: **325 → 133 BRAM tiles (90 % → 37 %)**, leaving ~63 % of the device's BRAM free — at no
throughput cost (both fixes removed waste).

### BRAM vs Design Gateway — where the tiles actually go

Design Gateway's CPU-less NVMe-IP is quoted at **66 BRAM** (or **8 URAM + 2 BRAM** with the URAM
option) for a 256 KiB internal buffer; it connects to AMD's *separate* PCIe Integrated Block.
LiteNVMe's standalone core measures **133 BRAM tiles** (CPU-based) / **~108** (RTL init, CPU
removed). That gap looks large until you break it down (LiteNVMe figures measured by OOC synth;
DG figures inferred from its published 66 / URAM-option):

| Component | LiteNVMe | Design Gateway |
|-----------|---------:|---------------:|
| 256 KiB data buffer/window      | ~64 BRAM | ~64 BRAM (or 8 URAM) |
| PCIe hard-IP block (Xilinx)     | **22 BRAM (counted)** | **not counted** (separate PCIe block) |
| Controller FIFOs / crossbar glue| ~24 BRAM | ~2 BRAM |
| Bring-up CPU ROM/RAM            | ~23 BRAM (firmware) / **0 (RTL init)** | 0 (RTL) |

So the difference is **not** a heavier NVMe engine — it is three separate things:

1. **The PCIe IP (~22 BRAM) is a counting boundary.** LiteNVMe's number includes the Xilinx
   `pcie4_uscale_plus` block (replay/buffers); DG quotes only its own IP and leaves the PCIe block
   to the integrator. That alone is ~half the gap and isn't LiteNVMe being bigger.
2. **The 256 KiB buffer (~64 BRAM) is a tie** — inherent to an on-chip staging buffer. Both can
   move it off BRAM: DG via a URAM option, LiteNVMe via its pluggable `hostmem_backend`
   (URAM or DDR/LiteDRAM) — neither is fundamental.
3. **The controller glue (~24 vs ~2 BRAM) is the genuine overhead, and it trades against
   throughput.** ~18 of it is the LitePCIe outstanding-request completion buffers
   (`ep_max_pending_requests`, default 8); the rest is the host-memory-responder data FIFOs
   (depth-64 at 256-bit ≈ 1 BRAM each). Lowering `ep_max_pending_requests` 8→2 cuts the glue to
   ~6 BRAM (standalone core 133→118 tiles) — but it is **not free**: HW-measured warm read
   throughput drops from ~2687 to ~2138 MB/s (~20%), because the SSD's SQE-fetch completions need
   those outstanding-request slots. It is also a **cliff, not a gradient** — `pending=4` measured
   the *same* 2138 MB/s as `pending=2` (so 4 is strictly dominated: same throughput, more BRAM).
   So there are two sensible points: **8** (full 2687 MB/s, 133 tiles) or **2** (118 tiles, −20%);
   kept at 8 by default. A hand-optimized commercial core uses tighter buffering to get both.

Net: with **RTL init** (drops the ~23-BRAM CPU) the core is ~108 tiles; excluding the PCIe IP
(apples-to-apples with DG) that is ~86 = 64 (buffer) + ~22 (glue), vs DG's 66 = 64 + 2. The
remaining ~20-tile difference is LiteX FIFO generosity, not the NVMe logic — and the buffer is
URAM-movable in both. So LiteNVMe is in the **same class**, with a tunable, well-understood
overhead rather than a hidden one.

### Where LiteNVMe still differs — and a real architectural gap

The remaining BRAM gap to Design Gateway's 66 is almost entirely the **embedded bring-up CPU**
(≈ 34 BRAM of ROM/RAM + ~1.7 k LUT). But that CPU is not just overhead — it is *how LiteNVMe does
NVMe initialization*, and this is the one genuine architectural difference:

- **Design Gateway is truly CPU-less**: it performs the full init sequence (PCIe link, controller
  enable, Identify, creating the I/O queues) **autonomously in RTL** — no CPU, OS, or software
  anywhere. The user only issues storage commands.
- **LiteNVMe does init in software.** The bring-up sequence lives in C firmware on the embedded
  VexRiscv. The `cpu: None` variant does **not** add an RTL init sequencer — it removes the CPU
  and instead expects an **external host** to run the init over the control bus. So LiteNVMe today
  always needs software for bring-up (embedded *or* external); it has **no pure-RTL init**.

So `cpu: None` lowers BRAM to ≈ window (64) + PCIe wrapper (22), but on its own it does not make
LiteNVMe a drop-in autonomous controller — bring-up has to come from a host. The fully-standalone
answer is a **pure-RTL init sequencer** that replays the firmware bring-up as a state machine.
**This is now demonstrated** (`litenvme/init.py`, `--with-rtl-init`, on the `rtl-init` branch):
`LiteNVMeInitSequencer` drives config BAR0 assign, the root memory window, controller enable, the
three admin commands (Set Features / Create IO CQ / Create IO SQ) and the engine config entirely
in hardware, and is **HW-validated on the Alibaba KU3P with no CPU and no firmware** (init_done,
then a read completes errors=0). It's opt-in; the firmware bring-up stays the default because
doing init in C makes it far easier to read, extend (new admin commands, vendor quirks) and debug
— RTL init is for CPU-less deploy / minimum area, where it brings LiteNVMe to DG's standalone class.

Everything else is already competitive: the 256 KiB data buffer (≈ 64 BRAM) is in Design Gateway's
class, `hostmem_size` / a DDR-backed `hostmem_backend` tune it further, and the soft logic is
modest (~7 k LUT for the datapath, ~11.5 k incl. PCIe + CPU) with **0 URAM / 0 DSP**.

## Takeaways

1. On **Gen3 x4 with a real SSD, LiteNVMe is near the practical ceiling** (~80 % of usable link,
   device-bound) and competitive with / better-than the closest commercial host core on the
   metrics each publishes — especially on writes.
2. **Higher absolute numbers in the market are Gen4/Gen5/x8**, i.e. a PHY/lane scaling story, not
   a host-core-architecture gap. That is the clearest optimization direction for LiteNVMe.
3. Open-source NVMe *host* cores are **rare** — most open NVMe FPGA work is device/SSD
   controllers (OpenExpress, Cosmos+ OpenSSD), a different problem. A few open *host* efforts do
   exist (e.g. DUNE's `pl-nvme` VHDL host, and a handful of academic/Chisel/RTL projects);
   LiteNVMe is the LiteX-native one, and the one with published, reproducible perf/resource
   numbers here. (We have not benchmarked the other open host cores head-to-head.)

---

Sources (accessed June 2026):
- IntelliProp NVMe Host Accelerator IP (IPC-NV164-HI): <https://intellipropipcores.com/ipc-nv164-hi/>
- IntelliProp demo throughput (KCU105, Gen3): <https://www.fpgadeveloper.com/2017/01/demo-of-intelliprops-nvme-host-accelerator-ip-core.html/>
- Design Gateway NVMe-IP (CPU-less host, Gen5 >11 GB/s, BRAM/URAM): <https://dgway.com/NVMe-IP_X_E.html>
- iWave high-speed NVMe SSD access on FPGAs: <https://iwave-global.com/articles/high-speed-nvme-ssd-access-on-fpgas/>
- OpenExpress (KAIST, USENIX ATC '20, device-side, open): <https://www.usenix.org/conference/atc20/presentation/jung>
