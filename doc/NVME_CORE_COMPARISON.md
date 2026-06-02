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
possible — but LiteNVMe's own core-only numbers are exact. The standalone `litenvme_core` was
synthesized on its own (out-of-context, XCKU3P, Gen3 x4 / 256-bit, default 512 KB window +
embedded bring-up CPU); the Vivado hierarchy gives:

| Core | LUT | FF | BRAM tiles | URAM | DSP | Data buffer / CPU |
|------|----:|---:|-----------:|-----:|----:|-------------------|
| **LiteNVMe** standalone core | **13,303** | **15,397** | **325** | 0 | 0 | 512 KB window + soft-CPU |
|  ├ PCIe hard-IP wrapper (+GTY)| 2,960 | 6,303 | 22 | 0 | 0 | — |
|  ├ VexRiscv bring-up CPU      | 2,182 |   749 |  0 (+2 RAMB18) | 0 | 0 | — |
|  └ NVMe datapath + streamer + hostmem + glue | ~8,161 | ~8,345 | ~298 | 0 | 0 | incl. the 512 KB window |
| Design Gateway NVMe-IP (std)  | n/p | n/p | 66 | 0 | n/p | 256 KB RAM, **CPU-less** |
| Design Gateway NVMe-IP (URAM) | n/p | n/p | 2  | 8 | n/p | 256 KB RAM, **CPU-less** |
| IntelliProp / iWave           | n/p | n/p | n/p| n/p | n/p | user-defined (BRAM/DDR) |

(Post-synthesis estimate; post-place would be slightly lower. Numbers are for the
block-streamer product config — `examples/alibaba_xcku3p.yml`.)

Honest read of this: **LiteNVMe currently uses more BRAM than the leanest CPU-less commercial
cores** (325 tiles vs Design Gateway's 66). Two reasons, both configurable:
1. The **default host-memory window is 512 KB** (~half the BRAM) vs Design Gateway's 256 KB —
   shrink `hostmem_size`, or point `hostmem_backend` at DDR/LiteDRAM, to cut it.
2. LiteNVMe **embeds a soft-CPU** (ROM/RAM) for one-time bring-up, where Design Gateway is
   pure-RTL CPU-less — the `cpu: None` variant removes the CPU (and its ROM/RAM) in favour of an
   external control bus, at the cost of doing NVMe init from the host.

On **soft logic** LiteNVMe is modest (~8 k LUT for the NVMe datapath itself, ~13 k including the
PCIe wrapper and CPU) and uses **0 URAM / 0 DSP**, so it ports cleanly across Ultrascale+. The
honest summary: LiteNVMe trades some BRAM (bigger default buffer + embedded CPU) for open-source
flexibility and self-contained bring-up; the area is reasonable and tunable, not best-in-class.

## Takeaways

1. On **Gen3 x4 with a real SSD, LiteNVMe is near the practical ceiling** (~80 % of usable link,
   device-bound) and competitive with / better-than the closest commercial host core on the
   metrics each publishes — especially on writes.
2. **Higher absolute numbers in the market are Gen4/Gen5/x8**, i.e. a PHY/lane scaling story, not
   a host-core-architecture gap. That is the clearest optimization direction for LiteNVMe.
3. LiteNVMe is, as far as we can find, the **only open-source NVMe *host* core** — the other open
   cores (OpenExpress, Cosmos+ OpenSSD) are device/SSD controllers, a different problem.

---

Sources (accessed June 2026):
- IntelliProp NVMe Host Accelerator IP (IPC-NV164-HI): <https://intellipropipcores.com/ipc-nv164-hi/>
- IntelliProp demo throughput (KCU105, Gen3): <https://www.fpgadeveloper.com/2017/01/demo-of-intelliprops-nvme-host-accelerator-ip-core.html/>
- Design Gateway NVMe-IP (CPU-less host, Gen5 >11 GB/s, BRAM/URAM): <https://dgway.com/NVMe-IP_X_E.html>
- iWave high-speed NVMe SSD access on FPGAs: <https://iwave-global.com/articles/high-speed-nvme-ssd-access-on-fpgas/>
- OpenExpress (KAIST, USENIX ATC '20, device-side, open): <https://www.usenix.org/conference/atc20/presentation/jung>
