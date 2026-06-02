<!--
This file is part of LiteNVMe.
Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
Developed with LLM assistance.
SPDX-License-Identifier: BSD-2-Clause
-->

# LiteNVMe — As-Built Architecture (Gen3 x4, 256-bit)

This document describes the system **as it actually runs today**: an FPGA acting as a PCIe
**Root Complex** that drives a real NVMe SSD, with the NVMe command engine in RTL and a soft CPU
for bring-up/orchestration. For the original bottom-up plan see `NVME_ARCHITECTURE.md` (historical).

- **Board:** Alibaba KU3P (Xilinx XCKU3P).
- **SSD:** Crucial CT500P310SSD8 (NVMe, MDTS=256 KiB, MPS-supported=512 B).
- **Link:** PCIe **Gen3 x4** (8 GT/s ×4), trained and stable.
- **Datapath:** **256-bit @ 125 MHz = 4.0 GB/s** internal (PCIe hard-IP user clock 125 MHz,
  core clock 250 MHz).

---

## 1. Block diagram

```
                         FPGA (XCKU3P)  —  PCIe Root Complex
 ┌───────────────────────────────────────────────────────────────────────────────┐
 │  VexRiscv soft CPU (BIOS) ── firmware: NVMe init + engine setup + benchmarks    │
 │        │ CSR                                                                    │
 │        ▼                                                                        │
 │  ┌──────────────┐   ┌──────────────┐   ┌───────────────┐   ┌──────────────────┐ │
 │  │ cfg accessor │   │ mmio accessor│   │ mmio_db (DB)  │   │ rootcfg (cfg_mgmt)│ │
 │  │  tag 0x42    │   │  tag 0x44    │   │  tag 0x45     │   │ root DevCtl/MPS + │ │
 │  │ CFG TLPs     │   │ BAR0 regs    │   │ SQ/CQ doorbell│   │ bridge mem window │ │
 │  └──────┬───────┘   └──────┬───────┘   └──────┬────────┘   └─────────┬────────┘ │
 │         │ (requester master ports)           │ driven by            │ (local)  │
 │         └─────────────┬──────────────────────┘ io_engine            │          │
 │                       ▼                                             │          │
 │            ┌──────────────────────┐         ┌─────────────────────┐ │          │
 │            │ LitePCIe crossbar +   │◄────────│  io_engine (RTL)    │ │          │
 │            │ TLP controller (tags, │  AXI    │  qsize=64, qd=32    │ │          │
 │            │ in-order completions) │  master │  builds SQE+PRP,    │ │          │
 │            └──────────┬───────────┘         │  rings DB, reaps CQE │ │          │
 │                       │                     └──────────┬──────────┘ │          │
 │   slave (completer)   │                                │ AXI        │          │
 │   port  ▼             │                                ▼            │          │
 │  ┌──────────────────────────────┐        ┌──────────────────────────────────┐ │
 │  │ hostmem responder (512 KiB    │◄──arb──│ hostmem AXI backend (BRAM)        │ │
 │  │ BRAM): SQ/CQ rings + PRP data │        │ 256-bit, byte-write-enabled       │ │
 │  └──────────────────────────────┘        └──────────────────────────────────┘ │
 │                       │                                                        │
 │                       ▼                                                        │
 │            ┌──────────────────────┐                                            │
 │            │ USPPCIEPHY (LitePCIe) │  RootPort, pcie4_uscale_plus hard IP       │
 │            │ RQ/RC/CQ/CC adapters  │  256-bit, force_64b (4DW), cfg_mgmt        │
 │            └──────────┬───────────┘                                            │
 └───────────────────────┼────────────────────────────────────────────────────────┘
                         ▼  Gen3 x4
                    NVMe SSD (Crucial P310)
```

Source map:
- SoC / integration: `bench/alibaba_xcku3p.py` (`BaseSoC`).
- NVMe core: `litenvme/core.py` (`LiteNVMeCore`) — wires the blocks below.
- CFG accessor: `litenvme/cfg.py` (`LiteNVMePCIeCfgAccessor`), root cfg_mgmt (`LiteNVMeRootCfgMgmt`).
- MMIO accessor: `litenvme/mem.py` (`LiteNVMePCIeMmioAccessor`) — used for both BAR0 regs (`mmio`)
  and the engine doorbells (`mmio_db`).
- Host-memory window: `litenvme/hostmem.py` (`LiteNVMeHostMemResponder` + AXI-RAM backend).
- I/O engine: `litenvme/io_engine.py` (`LiteNVMeIOEngineAXI`) + generator `litenvme/request_gen.py`.
- PHY: litepcie `litepcie/phy/usppciephy.py` (branch `litenvme-cfg-mgmt`).
- Firmware: `bench/firmware/main.c`.

---

## 2. The four PCIe traffic classes

Because the FPGA is the **Root Complex**, all four PCIe request/completion classes are in play.
They map to distinct LitePCIe adapters in `usppciephy.py`, and getting each right at 256-bit was
the bulk of the Gen3 bring-up (see §6).

| Direction | TLP | Who initiates | LitePCIe path | Used for |
|---|---|---|---|---|
| FPGA → SSD request | CfgRd/Wr, MemRd/Wr | our masters | **RQ** adapter (`SAxisRQAdapter`) | config, BAR0 regs, doorbells |
| SSD → FPGA completion | CplD | SSD answers our reads | **RC** adapter (`MAxisRCAdapter`) | MMIO/cfg read data |
| SSD → FPGA request | MemRd/Wr | SSD DMAs host memory | **CQ** adapter (`MAxisCQAdapter`) | SQE fetch, CQE & data writes |
| FPGA → SSD completion | CplD | we answer SSD reads | **CC** adapter (`SAxisCCAdapter`) | serving SQE/PRP fetches |

Key 256-bit detail: on UltraScale+ at ≥256-bit, litepcie's `force_64b` makes **memory** requests
**4-DW** (64-bit address) while **config** stays **3-DW**. The RQ adapter must place the address
accordingly (fixed — see §6.1).

---

## 3. Control plane (CFG + MMIO accessors)

Three "requester" master ports hang off the LitePCIe crossbar; all read-capable masters share ONE
`LitePCIeTLPController` (tag pool, **in-order** completion delivery, demux to the owning master by
channel):

- **`cfg` (tag 0x42)** — emits Type-0 Config TLPs to the SSD: BAR discovery/sizing, Command
  register (`MEM`/`BME`), reading DevCap/DevCtl (MPS/MRRS).
- **`mmio` (tag 0x44)** — BAR0 memory-mapped register access: CAP/CSTS/CC, AQA/ASQ/ACQ, and the
  controller-enable handshake. Single outstanding transaction; reads wait for the completion.
- **`mmio_db` (tag 0x45)** — a second MMIO accessor wired straight to the `io_engine` so the engine
  can ring SQ/CQ doorbells in hardware with no CPU involvement.

`rootcfg` (`LiteNVMeRootCfgMgmt`) is different: it reaches **our own root port's** config space via
the hard-IP **cfg_mgmt** sideband (not a TLP), to raise the root's `DevCtl.MPS` and program the
**bridge memory window** so downstream MemRd/Wr to the SSD BAR0 are forwarded.

> **Read-lag note:** the shared TLP controller delivers completions in request order; an accessor
> must consume its completion through the `end` beat or a stale beat can be matched by the next read
> (off-by-one at Gen3/256b). Firmware additionally reads each MMIO register until two reads agree
> (`mmio_rd32` lag-safe), mirroring the long-standing `cfg_rd32_retry` idiom.

---

## 4. Host-memory window (the completer / DMA target)

`LiteNVMeHostMemResponder` is a **slave (completer)** port on the crossbar backing a **512 KiB
BRAM** (256-bit wide, byte-write-enabled). To the SSD it *is* host memory: it holds the admin and
I/O **SQ/CQ rings**, the **Identify** buffer, the **PRP data buffers**, and (for >2-page transfers)
the **PRP-list pages**. Address map (`bench/firmware/main.c`, base `0x1000_0000`):

```
+0x0000 ASQ   +0x1000 ACQ   +0x2000 ID   +0x3000 IO_CQ   +0x4000 IO_SQ
+0x5000 IO_RD_BUF   +0x6000 IO_WR_BUF   +0x10000 IO_BUF_BASE (per-slot 4 KiB buffers)
```

The SSD accesses it as a completer request (CQ adapter → depacketizer → responder). Writes land via
the AXI backend with **byte strobes**; a CSR debug port lets firmware read/write individual dwords.

> **Sub-beat write note:** at 256-bit a 16-byte CQE to a non-32-byte-aligned offset (e.g. ACQ+16)
> must be placed at its byte offset within the beat and strobe only its dwords — otherwise it lands
> in the wrong slot and the stale half clobbers the neighbour (fixed — see §6.3).

---

## 5. I/O command engine (RTL) and the data path

`LiteNVMeIOEngineAXI` (`qsize=64`, `qd=32`) owns the steady-state I/O path with **no per-command CPU
cost**. Firmware does one-time setup (programs SQ/CQ bases, doorbell addresses, the PRP-list region,
and a request generator with op/LBA/nlb/count), then kicks it. The engine then:

1. Builds each **SQE** in hostmem (PRP1; PRP2 = next page for 2 pages; a **PRP list** for >2 pages),
2. Rings the **SQ doorbell** (coalesced) via `mmio_db`,
3. The SSD fetches the SQE/PRPs from hostmem (its MemRd → our CC adapter answers from the responder),
4. The SSD moves the data (read: SSD writes hostmem as MemWr; write: SSD reads hostmem),
5. The SSD writes a **CQE** to hostmem; the engine **reaps** it (reads via its AXI port, phase-bit),
6. Up to **QD=32** commands stay outstanding (Little's law: IOPS ≈ qd / latency).

On-chip **duty-cycle counters** in the responder (`wr_present/accept/stall_cycles`, `tlp_count`)
measure how continuously the SSD delivers read data, which is how we know reads are
device/PCIe-overhead bound, not datapath bound (the 256-bit datapath keeps up with <2% stall).

---

## 6. Gen3 + 256-bit bring-up — the three root-port bugs

Switching from the banked Gen2/128-bit config to Gen3 x4 + 256-bit surfaced a series of independent
bugs in litepcie's less-travelled **root-port-at-256-bit** paths. All are fixed; sim + HW verified.

### 6.1 RQ requester 4-DW (litepcie `SAxisRQAdapter`)
`force_64b` makes memory requests 4-DW at 256-bit while config stays 3-DW; the adapter only handled
3-DW, so MemRd went to address 0 → Unsupported-Request. Fixed by selecting address placement on
`fmt[0]` (the 4-DW bit). litepcie commit `a0f6216` (branch `litenvme-cfg-mgmt`).

### 6.2 MMIO read-lag (`litenvme/mem.py` + firmware)
The MMIO/CFG accessors matched *any* pending completion on the shared controller → returned the
previous transaction's data (benign at Gen2, surfaces at Gen3/256b, >1 deep after a CFG→MMIO channel
switch). Fixed by consuming through the completion **`end`** beat in `mem.py`, plus firmware lag-safe
reads (read-until-stable).

### 6.3 Completer sub-beat write (`litenvme/hostmem.py`)
The SSD's 16-byte admin CQE to a mid-beat offset landed in the wrong CQ slot (data delivered
left-aligned, written full-beat) → Create IO CQ never completed. Fixed by shifting the write data to
the address's dword offset and strobing only the written dwords. **Subtlety that cost ~3 synths:**
the shift must be a constant-shift `Case`, **not** `req_dat << (off_dw*32)` — a variable shift by a
signal expression simulated correctly but **synthesised to no-shift** on hardware.

---

## 7. Clocking

| Domain | Freq | Notes |
|---|---|---|
| `sys`  | 125 MHz | CPU, CSR, hostmem, engine, MMIO accessors |
| `pcie` | 125 MHz | PCIe hard-IP user clock (axisten_freq=125 for the 256-bit interface) |
| coreclk| 250 MHz | PCIe hard-IP core clock (Gen3 x4 requires 125 MHz user × 256-bit) |

`PHYRXDatapath`/`PHYTXDatapath` cross `pcie`↔`sys` via AsyncFIFO; `sys`↔`pcie` are declared a false
path. 256-bit @ 125 MHz was chosen over 128-bit @ 250 MHz because the sys-domain fabric does not
close timing at 250 MHz (~147 MHz fmax) and 128-bit@250 needs a 250↔125 CDC that regressed writes.

---

## 8. Why MPS is 512 B and the read ceiling

PCIe MaxPayloadSize is `min(root, device)`; the **SSD caps MPS at 512 B** (DevCap), so read-data
arrives as 512-byte MemWr TLPs no matter what we set — this is the dominant per-TLP overhead. The
Gen3 x4 link usable ceiling is ~3.4 GB/s; the 256-bit datapath (4.0 GB/s) is *not* the limit. The
only lever left on this link is the **NVMe transfer size** (amortizes per-TLP/per-command overhead);
see `NVME_PERFORMANCE.md`. Going beyond ~3.4 GB/s needs a faster link (Gen4 x4 / Gen3 x8), which is
the only scenario where a 512-bit datapath would help.

---

## 9. Firmware role

The soft CPU runs `bench/firmware/main.c`: BAR0 assign + `MEM`/`BME` enable, auto-MPS, admin init
(AQA/ASQ/ACQ, CC.EN, CSTS.RDY), Identify, Set Features (#queues), Create IO CQ/SQ, then programs and
kicks the RTL engine for benchmarks. It is **bring-up/orchestration only** — steady-state I/O is
hardware-driven by the engine. A LiteScope harness (`--litescope-probe pcie`)
and HW scripts (`bench/hw_*.sh`) reproduce link training, throughput, integrity, and the captures
that located the bugs in §6.
