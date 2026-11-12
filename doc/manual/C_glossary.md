# Glossary {#glossary}

The terms of NVMe, of PCIe and of LiteNVMe used in this manual, in alphabetical order. Each entry
points to the chapter that explains it; PCIe terms are explained in the LitePCIe manual.

| Term | Meaning |
|------------------|----------------------------------------------------------------------------------|
| ACQ | Admin Completion Queue Base Address: controller register with the address of the admin CQ. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Admin queue | Queue 0 of the controller, for admin commands such as Identify and Create I/O SQ. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| AQA | Admin Queue Attributes: controller register with the sizes of the admin SQ and CQ. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| ASQ | Admin Submission Queue Base Address: controller register with the address of the admin SQ. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| BAR, BAR0 | Base Address Register of a PCIe function; BAR0 of the controller maps its NVMe registers and doorbells. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| BDF | Bus, device and function numbers that address a PCIe function in configuration requests. See [Configuration and register access](#pcie-access) |
| Block streamer | Front-end of the I/O engine that moves sectors between data streams and the staging region. See [Front-ends](#frontends) |
| BME | Bus Master Enable: bit of the PCIe Command register that lets the SSD issue requests. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Bring-up | Sequence from the release of PERST# to `init_done`: BAR0, queues, controller enable. See [Bring-up](#bringup) |
| CAP | Controller Capabilities: read-only controller register with MQES, DSTRD and TO. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| CC | Controller Configuration: controller register with the enable bit (CC.EN), the memory page size and the entry sizes. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| cfg_mgmt | Configuration management port of the hard IP, which reaches the configuration space of the root port itself. See [Configuration and register access](#pcie-access) |
| CID | Command Identifier: 16-bit tag of an SQE, returned in its CQE; the I/O engine uses the SQ slot index. See [The I/O engine](#io-engine) |
| CNS | Controller or Namespace Structure: field of Identify that selects the data structure returned. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Configuration accessor | Block of the core that sends CfgRd0 and CfgWr0 requests to the SSD. See [Configuration and register access](#pcie-access) |
| CplD | PCIe completion with data, the answer to a MemRd or a CfgRd0. See [The host-memory window](#host-memory) |
| CQ | Completion queue: ring in host memory where the controller writes CQEs. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| CQE | Completion queue entry, 16 bytes: SQHD, CID, phase bit and status. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Crossover UART | UART of the soft CPU carried by Etherbone registers instead of pins; `litex_term crossover` reaches it. See [Bench, firmware and host tools](#examples) |
| CSTS | Controller Status: controller register with the ready bit (CSTS.RDY). See [NVMe over PCIe in a nutshell](#nvme-primer) |
| DNR | Do Not Retry bit of a CQE status: the same command would fail again. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Doorbell | Controller register the host writes with a new SQ tail or CQ head. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Doorbell accessor | MMIO accessor without registers through which the I/O engine rings its doorbells. See [Configuration and register access](#pcie-access) |
| DSTRD | Doorbell Stride: field of CAP that spaces the doorbells by `4 << DSTRD` bytes. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Etherbone | Protocol that carries bus accesses over UDP, so that the bench PC reads and writes the registers of the board. See [Getting started](#getting-started) |
| Host-memory window | Memory of the FPGA that the SSD reads and writes through PCIe: queues, PRP lists and data. See [The host-memory window](#host-memory) |
| I/O engine | Block that builds SQEs, rings the doorbells and reaps CQEs in hardware. See [The I/O engine](#io-engine) |
| I/O queue | Queue pair for Read and Write commands; LiteNVMe creates one, queue 1. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Init sequencer | State machine that performs the bring-up without a CPU (`init_mode` `rtl`). See [Bring-up](#bringup) |
| LBA | Logical block address: number of a sector of a namespace. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| LTSSM | Link Training and Status State Machine of the PCIe PHY; its state `0x10` is L0, the link in use. See [Debugging](#debugging) |
| MDTS | Maximum Data Transfer Size of the controller, from Identify. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| MemRd, MemWr | PCIe memory read and memory write requests. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| MMIO accessor | Block of the core that reads and writes the controller registers in BAR0 with MemRd and MemWr. See [Configuration and register access](#pcie-access) |
| MPS | PCIe Max Payload Size: largest data payload of a TLP, programmed on both ends of the link. See [Integration](#integration) |
| MQES | Maximum Queue Entries Supported: field of CAP, the largest queue size minus one. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| MRRS | PCIe Max Read Request Size: largest MemRd a function may issue. See [Integration](#integration) |
| Namespace | Addressable range of sectors of the controller; LiteNVMe uses namespace 1. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| NLB | Number of Logical Blocks of a Read or Write, 0-based in the SQE. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| NSID | Namespace identifier of a command. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| PERST# | PCIe reset signal of the slot; the standalone core releases it 100 ms after its own reset. See [Bring-up](#bringup) |
| Phase bit | P bit of a CQE; the controller inverts it on each pass over the ring, so the host sees a new entry. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| PRP | Physical Region Page entry: address of a memory page of a command's data (PRP1, PRP2). See [NVMe over PCIe in a nutshell](#nvme-primer) |
| PRP list | Page of PRP entries pointed to by PRP2 when the data spans more than two pages. See [The I/O engine](#io-engine) |
| PRP-list region | Part of the host-memory window with one 4 KiB PRP-list page per I/O queue slot. See [The I/O engine](#io-engine) |
| Queue depth | Commands in flight at a time (`qd`); at most the queue size minus one. See [The I/O engine](#io-engine) |
| Request generator | Front-end of the I/O engine that issues sequential commands for benchmarks. See [Front-ends](#frontends) |
| Root configuration accessor | Block that reads and writes the configuration space of the root port through the cfg_mgmt port of the hard IP. See [Configuration and register access](#pcie-access) |
| Root port | The PCIe port of the FPGA that owns the link to the SSD (`LitePCIeRootPort`). See [Architecture](#architecture) |
| SC, SCT | Status Code and Status Code Type of a CQE; both zero on success. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Sector | Logical block of 512 bytes, the only block size of LiteNVMe. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| SGL | Scatter Gather List: alternative to PRPs to describe data; not used by LiteNVMe. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| SQ | Submission queue: ring in host memory where the host writes SQEs. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| SQE | Submission queue entry, 64 bytes: opcode, CID, NSID, PRP1, PRP2 and command fields. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| SQHD | SQ Head Pointer of a CQE: SQ entries the controller has consumed. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| Staging region | Part of the host-memory window through which the block streamer moves data. See [Front-ends](#frontends) |
| TLP | Transaction Layer Packet, the unit of PCIe requests and completions. See [NVMe over PCIe in a nutshell](#nvme-primer) |
| TO | Timeout field of CAP: worst-case time for CSTS.RDY to change, in units of 500 ms. See [NVMe over PCIe in a nutshell](#nvme-primer) |
