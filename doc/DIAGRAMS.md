<!--
This file is part of LiteNVMe.
Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
Developed with LLM assistance.
SPDX-License-Identifier: BSD-2-Clause
-->

# LiteNVMe — Architecture Diagrams

Visual overview of the current architecture (rendered by GitHub's Mermaid). For the as-built
details see [`ARCHITECTURE.md`](ARCHITECTURE.md); for the generated core see
[`STANDALONE_CORE.md`](STANDALONE_CORE.md).

---

## 1. System architecture

LiteNVMe is a PCIe **RootPort** NVMe **host**: it drives a commercial SSD and exposes a simple
block read/write interface to your logic.

```mermaid
flowchart TB
    APP["Your logic / SoC"]

    subgraph CORE["LiteNVMe core (FPGA)"]
        direction TB
        BS["Block streamer<br/>sector read/write + staging"]
        ENG["I/O command engine<br/>QD=32 · SQE build / CQE reap"]
        BRINGUP["Bring-up (one-time):<br/>firmware on soft-CPU (default)<br/>--or-- RTL init sequencer (--with-rtl-init)"]
        CFG["CFG accessor<br/>(config space)"]
        MMIO["MMIO accessor<br/>(BAR0 registers)"]
        DB["Doorbell accessor"]
        HOST["Host-memory window<br/>256-bit BRAM<br/>SQ / CQ / PRP / staging"]
        XBAR["LitePCIe crossbar"]
        PHY["USPPCIEPHY RootPort<br/>Gen3 x4 · 256-bit @ 125 MHz"]
    end

    SSD["NVMe SSD"]

    APP <-->|"block_wr_axis / block_rd_axis<br/>+ command / status pins"| BS
    BS -->|"request / completion"| ENG
    BS -->|"staging (AXI)"| HOST
    ENG -->|"SQE / CQE (AXI)"| HOST
    ENG -->|"ring doorbells"| DB
    BRINGUP -.-> CFG
    BRINGUP -.-> MMIO
    BRINGUP -.-> ENG
    CFG --> XBAR
    MMIO --> XBAR
    DB --> XBAR
    XBAR <-->|"SSD DMA / host memory"| HOST
    XBAR <--> PHY
    PHY <-->|"PCIe Gen3 x4"| SSD
```

**Two planes:**
- **Control plane** — `CFG` / `MMIO` accessors issue config + BAR0 requests *to* the SSD through
  the crossbar; the engine rings SQ/CQ doorbells via `DB`.
- **Data plane** — the SSD DMAs to/from the **host-memory window** (crossbar slave port); the I/O
  engine and the block streamer's staging also reach that window over an internal AXI arbiter.

---

## 2. Bring-up — two interchangeable paths

Both run the *same* NVMe init sequence and raise `init_done`; pick one at build time.

```mermaid
flowchart TB
    START(["Power-up · PCIe link trained"])

    subgraph FW["Firmware path — default"]
        CPU["VexRiscv runs<br/>bench/firmware/main.c<br/>(reads CAP / DSTRD / Identify; retries)"]
    end
    subgraph RTL["RTL path — --with-rtl-init (CPU-less)"]
        SEQ["LiteNVMeInitSequencer<br/>(hardware FSM)"]
    end

    SEQUENCE["1. BAR0 assign + Command MEM/BME<br/>2. root memory window<br/>3. AQA/ASQ/ACQ · CC.EN · wait CSTS.RDY<br/>4. Set Features · Create IO CQ · Create IO SQ<br/>5. program + enable the I/O engine"]
    DONE(["init_done = 1 → block interface live"])

    START --> CPU
    START --> SEQ
    CPU --> SEQUENCE
    SEQ --> SEQUENCE
    SEQUENCE --> DONE
```

| | Firmware (default) | RTL init (`--with-rtl-init`) |
|---|---|---|
| Bring-up runs on | embedded soft-CPU (VexRiscv) | a hardware FSM, **no CPU** |
| SSD coverage | broad (reads CAP/DSTRD/BAR-type/Identify) | common case (64-bit BAR, DSTRD=0) |
| Easy to extend (new admin cmds) | yes (C) | RTL change |
| Resources | +CPU (~1.7k LUT + ~25 BRAM) | ~−2,560 LUT / −25 BRAM |

---

## 3. Block transfer — write then read

How a `{write/read, sector, count}` command flows end to end.

```mermaid
sequenceDiagram
    autonumber
    participant U as Your logic
    participant B as Block streamer
    participant E as I/O engine
    participant H as Host memory
    participant S as NVMe SSD

    Note over U,S: WRITE
    U->>B: start (write, sector, count)
    U->>B: stream payload — block_wr_axis
    B->>H: stage payload
    B->>E: request {op=write, lba, nlb, buf}
    E->>H: build SQE
    E->>S: ring SQ doorbell (MMIO)
    S->>H: read SQE + staged data (PCIe DMA)
    S->>H: write CQE
    E->>H: reap CQE (phase bit)
    E-->>B: completion
    B-->>U: done

    Note over U,S: READ
    U->>B: start (read, sector, count)
    B->>E: request {op=read, lba, nlb, buf}
    E->>S: ring SQ doorbell
    S->>H: read SQE, then write data (PCIe DMA)
    S->>H: write CQE
    E-->>B: completion
    B->>H: drain staged data
    B-->>U: stream payload — block_rd_axis
```

---

## 4. I/O command engine (QD) — state machine

The engine keeps up to `qd` (32) commands in flight with **no CPU in the loop**: it builds SQEs,
rings the SQ doorbell once per burst (coalesced), reaps CQEs by phase bit, and rings the CQ
doorbell once per batch.

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> LATCH_REQ: room in flight, CID free
    LATCH_REQ --> PRP_LIST: more than 2 pages
    LATCH_REQ --> SUBMIT_WRITE: 1 or 2 pages
    PRP_LIST --> SUBMIT_WRITE
    SUBMIT_WRITE --> SUBMIT_ADVANCE: 16 SQE dwords written
    SUBMIT_ADVANCE --> IDLE: sq_tail++, inflight++

    IDLE --> SUBMIT_DOORBELL: SQEs pending
    SUBMIT_DOORBELL --> IDLE: ring SQ doorbell (once)

    IDLE --> REAP_READ: something in flight
    REAP_READ --> REAP_CHECK: read CQE dw3 (phase)
    REAP_CHECK --> REAP_EMIT: phase matches
    REAP_EMIT --> REAP_READ: emit completion, cq_head++
    REAP_CHECK --> REAP_DOORBELL: no more ready
    REAP_DOORBELL --> IDLE: ring CQ doorbell (once)
```

---

## 5. Generated standalone core — top-level interface

`litenvme_gen <config>.yml` emits `litenvme_core.v` with these named ports (`data_width=256`).

```mermaid
flowchart LR
    PCIE["PCIe pads<br/>clk_p/n · rx/tx[3:0] · rst_n (PERST#)"]
    CLK["clk / rst<br/>(sys clock from PCIe)"]
    WR["block_wr_axis<br/>write payload in (AXI-Stream)"]
    RD["block_rd_axis<br/>read payload out (AXI-Stream)"]
    CTRL["block_ctrl<br/>start · write · sector · count → done · busy · error"]
    STAT["status_init_done / status_init_error"]

    CORE["litenvme_core<br/>(self-contained Verilog<br/>+ baked bring-up)"]

    PCIE <--> CORE
    CLK --- CORE
    WR --> CORE
    CORE --> RD
    CTRL <--> CORE
    CORE --> STAT
```

> Usage: wait for `status_init_done`, drive `{write, sector, count}`, pulse `block_ctrl_start`,
> stream `count*512` bytes in/out, wait for `block_ctrl_done` (check `block_ctrl_error`).
> 1 sector = 512 B; the 256-bit bus carries 32 B/beat.
