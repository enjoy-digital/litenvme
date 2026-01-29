# LiteNVMe Architecture Plan (Bring-up -> Proper NVMe Core)
#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

# LiteNVMe Architecture Plan

## Summary (Current vs Target)

**Current bring-up**
- PCIe RootPort + MMIO accessor + hostmem window (BRAM).
- Firmware or Python scripts drive NVMe init + I/O queues.
- Identify / Read / Write working with PRP1 only.
- Single outstanding, minimal DMA assumptions.

**Target NVMe core**
- Hardware engine in RTL:
  - owns admin + IO queues,
  - accepts requests via CSR/AXI-Lite,
  - handles MMIO + doorbells,
  - DMA to system memory (AXI/DDR),
  - supports PRP2/PRP lists and multi-queue.
- SoftCPU remains optional (debug/bring-up), not required for operation.

## 1) Gap Analysis vs Current Code

**What works and should be kept or evolved**
- LitePCIe RootPort integration.
- MMIO access to BAR0 (LiteNVMePCIeMmioAccessor).
- CFG access (BAR discovery, Command.MEM/BME control).
- Hostmem BRAM window used as DMA target in bring-up.
- Verified sequences for Identify / Create IO queues / Read / Write.
- Python scripts + firmware for reproducible tests.

**Missing for a proper core**
- Queue ownership in RTL (admin + IO queues with head/tail/phase).
- Hardware command scheduler (driven by user logic, not scripts).
- DMA engine (PRP1/PRP2/PRP list, large transfers).
- Queues located in real system memory (AXI/DDR).
- Robust completion/error handling and multi-queue support.
- Interrupt/MSI support (optional).

## 2) Target Architecture (Blocks & Interfaces)

```
NVMeCore
├── PCIe RootPort (LitePCIe)
├── NVMe MMIO Bridge (BAR0 accessor)
├── Admin Engine
├── IO Engine
├── DMA Engine
├── Queue Manager (SQ/CQ)
└── CSR/AXI-Lite Control Interface
```

### 2.1 NVMe MMIO Bridge
- Issues BAR0 MMIO reads/writes to the SSD.
- Single transaction pipeline initially; later allow multiple outstanding.
- Interface:
  - req: addr, we, wdata, len
  - rsp: rdata, done, err

### 2.2 Queue Manager
- Maintains SQ/CQ head/tail and phase bits.
- Writes SQEs into system memory and polls CQEs.
- Triggers doorbell writes.

### 2.3 Admin Engine
- Initializes controller (AQA/ASQ/ACQ, CC.EN, CSTS.RDY).
- Runs Identify and Set Features.
- Creates IO queues (CQ/SQ).
- Exposes status and errors to CSR.

### 2.4 IO Engine
- Accepts read/write requests from user logic (CSR/AXI-Lite).
- Builds SQEs, rings doorbells, waits for CQEs.
- Coordinates with DMA Engine for data movement.

### 2.5 DMA Engine
- PRP1 only first, then PRP2, then PRP list.
- Moves data between NVMe SSD and system memory (AXI/DDR).

## 3) Control/Status Interface

Recommended interface (CSR or AXI-Lite):
- Registers: opcode, nsid, lba, nlb, buf_addr, flags
- Status: busy, done, error, cqe_status
- Optional IRQ on completion

## 4) Refactor Plan (Concrete Steps)

**Phase A: Clean internal API**
- Extract MMIO access as reusable module.
- Define interfaces for mmio, queue_mgr, dma_engine.

**Phase B: Queue management in hardware**
- Implement SQ/CQ head/tail tracking in RTL.
- Store queues in system memory.

**Phase C: Admin engine in hardware**
- Auto init controller.
- Identify / Set Features / Create queues.
- CSR status + error reporting.

**Phase D: IO engine in hardware**
- CSR-driven read/write.
- CQE polling, completion reporting.

**Phase E: DMA engine upgrades**
- PRP2 + PRP list.
- Larger transfers and higher throughput.

**Phase F: Remove bring-up components**
- Replace BRAM hostmem with real system memory backend.
- SoftCPU becomes optional (debug).

## 5) Milestones

1. Consolidation (stable MMIO/CFG accessors).
2. Admin engine in RTL (Identify via hardware).
3. IO engine with PRP1.
4. PRP2/PRP list support.
5. Multi-queue + performance.

## 6) Firmware-first strategy (and when to move to RTL)

Firmware-driven admin/IO is a valid **first production step** if the expected
IO rate is modest and simplicity/iteration speed matters most. It lets us:

- ship a working system faster,
- reuse the validated bring-up sequences,
- iterate on the NVMe protocol without re-spinning RTL.

When the bottlenecks show up (throughput, concurrency, latency), we can move
queue management and scheduling into RTL. Until then, firmware is acceptable.

**Signs you should migrate to RTL queue ownership:**
- you need multiple outstanding IOs and higher IOPS,
- firmware spends most of its time polling CQEs,
- you want deterministic latency under load.

## 7) Firmware-driven production plan (proposed)

### Goal
Keep the SoftCPU in charge of NVMe queue management and DMA sequencing, while
exposing a clean CSR/AXI-Lite interface to user logic (no manual UART).

### Block diagram

```
User Logic (CSR/AXI-Lite)
    |
    v
Request FIFO + Status CSRs
    |
    v
SoftCPU Firmware (queue mgmt, MMIO, CQ polling)
    |
    v
PCIe RootPort + NVMe SSD
```

### CSR interface (minimal)

**Command registers (written by user logic):**
- `req_op`     (READ/WRITE/IDENTIFY)
- `req_nsid`
- `req_lba`
- `req_nlb`
- `req_buf`    (host memory address for PRP1)
- `req_start`  (kick; enqueue request)

**Status registers (read by user logic):**
- `req_busy`
- `req_done`
- `req_error`
- `req_cqe_status`
- `req_bytes_done` (optional)

**Optional:**
- `req_queue_level` (FIFO depth)
- `req_irq_enable` / `req_irq_status`

### Firmware state machine

1. **Init**
   - Configure BAR0 / Command.MEM+BME.
   - Admin init (AQA/ASQ/ACQ, CC.EN, RDY).
   - Create IO queues (Set Features + Create CQ/SQ).

2. **Idle**
   - Wait for `req_start` or FIFO entry.
   - Optionally sleep or low-power loop.

3. **Submit**
   - Build SQE in host memory (PRP1 only initially).
   - Ring SQ doorbell.

4. **Wait**
   - Poll CQE; on completion update status.
   - If read: firmware can leave data in host memory for user logic.

5. **Complete**
   - Write `req_done`, `req_error`, `req_cqe_status`.
   - Return to idle.

### Phase 1 limitations (acceptable)
- PRP1 only (≤ 4K per IO, or ≤ 8 blocks at 512B LBA).
- Single outstanding request at a time.
- Single IO queue (QID=1).

### Phase 2 upgrades
- Small request FIFO depth (4–8).
- Optional coalesced CQ polling.
- PRP2 + PRP list support.

## 8) Next Steps (Actionable)

1. Define CSR request format (op/nsid/lba/len/buf).
2. Implement request FIFO + status CSR.
3. Extend firmware to service the CSR requests.
4. Measure throughput; decide when to migrate queue mgmt to RTL.

## 9) RTL migration path (when needed)

When firmware is no longer sufficient, migrate in steps:

1. Hardware Admin Engine (init + identify).
2. Hardware Queue Manager (SQ/CQ state).
3. IO Engine with PRP1.
4. PRP2/PRP list and performance tuning.

## 10) Progressive validation plan

**Step 0 (done):** UART-driven firmware tests (Identify/Read/Write).

**Step 1 (CSR interface):**
- Add a minimal CSR request block (`nvme_req_*`).
- Verify CSR values are visible in firmware.

**Step 2 (firmware-driven production loop):**
- Firmware polls `req_start`, runs Identify/Read/Write.
- Updates `req_status` and `req_cqe_status`.
- Validate with a simple Python script that writes CSRs.

**Step 3 (queue depth + PRP2):**
- Add small FIFO (optional).
- PRP2/PRP list for >4KB transfers.
