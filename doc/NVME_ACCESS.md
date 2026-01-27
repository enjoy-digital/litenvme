# NVMe Access Flows (Identify, Read, Write)

This document explains how the LiteNVMe bring-up scripts drive NVMe Identify/Read/Write
using PCIe MMIO for controller configuration and PCIe DMA for host memory access.

## Actors

- **Host/RootPort (FPGA + scripts)**
  - Programs NVMe BAR0 registers via PCIe **MMIO MemWr/MemRd**.
  - Exposes a **host memory window** (`hostmem`) the SSD can access via PCIe DMA.
- **NVMe SSD**
  - Fetches SQ entries from host memory (DMA **MemRd**).
  - Writes completion entries and data buffers to host memory (DMA **MemWr**).

## Memory layout (hostmem window)

Use 4K-aligned regions inside the hostmem window:

- **ASQ** (Admin Submission Queue): `asq_addr` (64B entries).
- **ACQ** (Admin Completion Queue): `acq_addr` (16B entries).
- **Identify data buffer (PRP1)**: `id_buf` (4KB).
- **I/O CQ1**: `io_cq_addr` (16B entries, typically 4KB page).
- **I/O SQ1**: `io_sq_addr` (64B entries, typically 4KB page).
- **Read buffer (PRP1)**: `rd_buf` (4KB aligned).
- **Write buffer (PRP1)**: `wr_buf` (4KB aligned).

The scripts populate these buffers through the **hostmem CSR debug port**, but to the SSD
they look like normal host RAM.

## NVMe Identify flow (Admin command)

### 1) Configure the controller (MMIO)

Over PCIe MMIO (RootPort -> SSD BAR0):

1. **Disable controller** (if needed)
   - `MMIO: MemWr CC.EN=0`
   - poll `MMIO: MemRd CSTS.RDY` until 0
2. **Program Admin queue addresses and sizes**
   - `MMIO: MemWr AQA` (queue sizes, N-1 encoding)
   - `MMIO: MemWr ASQ` (physical address of ASQ)
   - `MMIO: MemWr ACQ` (physical address of ACQ)
3. **Enable controller**
   - `MMIO: MemWr CC` (EN=1, IOSQES/IOCQES, MPS, ...)
   - poll `MMIO: MemRd CSTS.RDY` until 1

At this point the SSD knows where your admin queues live in host memory.

### 2) Place Identify SQE in ASQ (host memory write)

Write 16 dwords (64B) into `ASQ[0]`:

- `CDW0`: `opcode=0x06` + `CID`
- `PRP1`: `id_buf` address (data destination in host memory)
- `CDW10`: `CNS=1` (Identify Controller)

### 3) Ring the Admin SQ doorbell (MMIO)

Tell the SSD there is 1 entry in the admin SQ:

- `MMIO: MemWr SQ0_TAIL = 1`

### 4) SSD DMA fetches the SQE (DMA MemRd)

The SSD initiates a PCIe MemRd to fetch the SQE from `ASQ[0]`. The host responder
returns Completion-with-Data (CplD) beats containing the SQE.

### 5) SSD executes Identify, then DMA writes results (DMA MemWr)

If the command is valid, the SSD performs two DMA writes:

1. **Identify data buffer (PRP1)**
   - `DMA: SSD -> Host MemWr` of 4096 bytes into `id_buf`.
2. **Completion entry into ACQ**
   - `DMA: SSD -> Host MemWr` of 16 bytes into `ACQ[0]`.

### 6) Host polls ACQ and reads Identify buffer

The script polls `ACQ[0]`, decodes the CQE, then reads and decodes `id_buf`.

## NVMe Read flow (I/O command)

The script performs a few Admin steps before issuing the first I/O:

- **Set Features (Number of Queues)** to request at least 1 I/O SQ/CQ.
- **Identify Namespace List (CNS=2)** and select the first active NSID.

### 1) Admin: Create I/O Completion Queue (CQ1)

**Admin command** `opcode=0x05` (Create I/O CQ), submitted via SQ0/CQ0.

- Host writes Create-IOCQ SQE into ASQ (QID=1, QSIZE=N-1, PRP1=io_cq_addr).
- Host rings **SQ0 tail**.
- SSD DMA fetches SQE, then writes completion into ACQ.

### 2) Admin: Create I/O Submission Queue (SQ1)

**Admin command** `opcode=0x01` (Create I/O SQ), submitted via SQ0/CQ0.

- Host writes Create-IOSQ SQE into ASQ (QID=1, QSIZE=N-1, CQID=1, PC=1).
- Host rings **SQ0 tail**.
- SSD DMA fetches SQE, then writes completion into ACQ.

### 3) Submit Read command to SQ1

Write Read SQE into `SQ1[tail]`:

- `CDW0`: `opcode=0x02` + `CID`
- `NSID`: usually 1
- `PRP1`: `rd_buf` (destination buffer)
- `CDW10-11`: `SLBA`
- `CDW12[15:0]`: `NLB = blocks - 1`

For first tests, keep `NLB=0` (one block) so **PRP1 only** is sufficient.

Ring **SQ1 tail** (MMIO):

- `MMIO: MemWr SQ1_TAIL = new_tail`

### 4) SSD executes read (DMA MemRd + MemWr)

1. **DMA MemRd**: SSD fetches SQE from SQ1.
2. **DMA MemWr**: SSD writes data to `rd_buf`.
3. **DMA MemWr**: SSD writes CQE to `CQ1[head]`.

### 5) Host polls CQ1 and acknowledges completion (MMIO)

- Poll `CQ1[head]` for matching phase bit.
- Decode `SCT/SC` (success if both 0).
- Advance head and ring **CQ1 head doorbell**:
  - `MMIO: MemWr CQ1_HEAD = new_head`

### 6) Host reads the data buffer

Read `rd_buf` from hostmem and display or validate contents.

## NVMe Write flow (I/O command)

### 1) Prepare write buffer (host memory)

Fill `wr_buf` with a known pattern. Keep it 4K-aligned.

### 2) Submit Write command to SQ1

Write Write SQE into `SQ1[tail]`:

- `CDW0`: `opcode=0x01` + `CID`
- `NSID`: usually 1
- `PRP1`: `wr_buf` (source buffer)
- `CDW10-11`: `SLBA`
- `CDW12[15:0]`: `NLB = blocks - 1`

Ring **SQ1 tail** (MMIO).

### 3) SSD executes write (DMA MemRd + CQE)

1. **DMA MemRd**: SSD reads data from `wr_buf`.
2. **DMA MemWr**: SSD writes CQE into `CQ1[head]`.

### 4) (Optional) Read-back verify

Issue a Read into `rd_buf` for the same LBA range and compare data. The script
compares a small prefix (first 64 dwords) for a quick sanity check.

## Practical notes

- **PRP1 only**: For first tests, keep transfers <= 4KiB so you do not need PRP2.
- **Queue sizes**: Use small queues (2-4 entries) during bring-up.
- **Doorbells**: Always ring SQ tail after writing SQEs and ring CQ head after consuming CQEs.
- **Alignment**: Keep all queue/buffer addresses 4K-aligned.
