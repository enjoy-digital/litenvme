# NVMe over PCIe in a nutshell {#nvme-primer}

This chapter explains NVMe to an FPGA engineer who has never read the specification. It follows
what a host does with a new SSD: find it on the link, enable its controller, create its queues,
then send commands and collect their completions. Each section ends with the part of LiteNVMe
that implements it.

The reference is the NVM Express Base Specification, cited by name. PCI Express itself is the
subject of the LitePCIe manual: this chapter links to its
[PCI Express primer](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/02_pcie_primer.md)
instead of explaining the link, the packets and the configuration space again.

## Why NVMe {#primer-why}

Older storage interfaces such as AHCI for SATA give each port one queue of 32 command slots, and
the host reads controller registers to learn which commands have finished. That one queue serves
the whole system.

NVMe moves the queues into the memory of the host. The host writes commands into a ring in its
own memory, then tells the SSD with one register write, the *doorbell*. The SSD fetches the
commands, moves the data and writes the results into a second ring, all by reading and writing
the host memory directly.

A host can create many queue pairs and keep many commands in flight in each. The SSD works on
them in parallel, which is how a drive reaches its rated throughput.

## The actors {#primer-roles}

Three parties take part on the host side of the link: the *driver*, the *root port* and the *host
memory*; the *SSD* is on the other side. In a computer, the driver runs on the CPU, the root port
is part of the chipset and the memory is the system DRAM. The figure below shows who talks to
whom.

![Who does what. The host reaches the controller registers with memory writes and reads through
its root port. The SSD reaches the host memory with its own reads and writes (DMA) to fetch the
commands and the data and to return the completions.](primer_roles.svg)

Two directions of traffic matter. The host reads and writes the controller registers in the SSD's
BAR0 with MemRd and MemWr requests. The SSD, as a bus master, reads and writes the host memory with
its own MemRd and MemWr requests, and the host answers its reads with completions.

The
[PCI Express primer](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/02_pcie_primer.md)
of the LitePCIe manual explains the tree of links in its section *The topology*, and the request
and completion packets in its section *Transaction layer packets*.

In LiteNVMe, the FPGA is the whole host side. The LitePCIe root port drives the link; the
accessors send the register requests; the host-memory window answers the SSD's requests from FPGA
block RAM ([Architecture](#architecture)).

## The SSD as a PCI Express function {#primer-function}

Before any NVMe register can be touched, the SSD must be set up like any PCI Express device. The
[PCI Express primer](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/02_pcie_primer.md)
of the LitePCIe manual describes the configuration space in its section *Configuration space and
enumeration*. Four steps matter for NVMe.

| Step | What the host does | Why |
|------------------------|------------------------------------------------------|------------------------------------|
| BAR0 sizing | Writes all ones to BAR0, reads back the size, assigns a bus address | The controller registers live in BAR0, usually a 64-bit BAR (BAR0 and BAR1) |
| Command register | Sets Memory Space Enable (MEM) and Bus Master Enable (BME) | MEM lets the SSD answer register accesses; BME lets it read and write host memory |
| Payload sizes | Programs the same MPS on both ends, and the MRRS | The largest data payload of a TLP; both ends must agree |
| Bridge window | Sets the memory base and limit of the root port | The root port forwards a register access downstream only inside this window |

The SSD may need up to 100 ms after link training before it answers configuration requests; the PCI
Express Base Specification sets this delay, and lets the SSD answer with a Configuration Request
Retry Status meanwhile.

A register access to BAR0 before MEM is set, or outside the root port's window, gets an Unsupported
Request: the accessors of LiteNVMe report it as an error.

MPS stays at its reset value of 128 bytes until software raises it on both ends. Each TLP then
carries 128 bytes of data for about 20 bytes of header and framing, so the link loses
efficiency. The section *Root port designs* of the
[Integration chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/12_integration.md)
of the LitePCIe manual describes how to program the root port side.

In LiteNVMe, the configuration accessor sends the configuration requests, and the root
configuration accessor programs the root port's own registers through the cfg_mgmt port of the
hard IP ([Configuration and register access](#pcie-access)).

## The controller registers {#primer-registers}

The controller exposes a small set of registers at the start of BAR0, then one doorbell per queue
from offset `0x1000`. The figure below shows the map.

![The registers of an NVMe controller in BAR0. The fixed registers start at offset 0x00; the
doorbells start at 0x1000, one per queue and direction, spaced by the doorbell
stride.](primer_bar0.svg)

The registers and the fields a host uses at bring-up are in the table below.

| Register | Offset | Fields used |
|----------|---------|----------------------------------------------------------------------|
| CAP | `0x00` | MQES `[15:0]`: largest queue size, minus one. TO `[31:24]`: ready timeout in 500 ms units. DSTRD `[35:32]`: doorbell stride |
| VS | `0x08` | Version of the specification the controller implements |
| CC | `0x14` | EN `[0]`: enable. MPS `[10:7]`: memory page size, 2^(12 + MPS) bytes. IOSQES `[19:16]` and IOCQES `[23:20]`: entry sizes, as powers of two |
| CSTS | `0x1c` | RDY `[0]`: the controller is ready. CFS `[1]`: fatal error |
| AQA | `0x24` | Admin SQ size `[11:0]` and admin CQ size `[27:16]`, minus one |
| ASQ | `0x28` | Bus address of the admin submission queue, 64 bits |
| ACQ | `0x30` | Bus address of the admin completion queue, 64 bits |

CAP is read-only: it tells the host what the controller supports. CC is written by the host to
configure and enable the controller; CSTS reports the result.

In LiteNVMe, the firmware and the init sequencer write CC with IOSQES 6 (64-byte SQEs), IOCQES 4
(16-byte CQEs) and a memory page size field of 0, so the memory page size is 4 KiB.

## Doorbells {#primer-doorbells}

Each queue has one doorbell register. Queue `y` has its submission queue tail doorbell at offset
`0x1000 + (2y) * stride` and its completion queue head doorbell at
`0x1000 + (2y + 1) * stride`. The stride is `4 << CAP.DSTRD` bytes.

Many SSDs report DSTRD 0, a stride of 4 bytes. A host that assumes it works on those SSDs only;
reading DSTRD from CAP works on all of them.

The host never reads a doorbell. It writes the new tail of a submission queue after adding
commands, and the new head of a completion queue after consuming completions. A doorbell write is
a posted MemWr: it carries no answer.

In LiteNVMe, the firmware computes the doorbell addresses from CAP.DSTRD. The init sequencer
assumes DSTRD 0 ([Bring-up](#bringup)). The I/O engine writes its doorbells through a dedicated
MMIO accessor ([The I/O engine](#io-engine)).

## Enabling the controller {#primer-enable}

A controller is disabled while CC.EN is 0. The host sets it up in that state, enables it, then
waits for it to report ready. The sequence is short.

1. If CC.EN is 1, write it to 0 and wait until CSTS.RDY is 0.
2. Write AQA with the sizes of the admin queues, ASQ and ACQ with their addresses.
3. Write CC with EN at 1 and the entry sizes and the memory page size.
4. Read CSTS until RDY is 1. CAP.TO bounds the wait.

The controller then serves the admin queue, number 0. The I/O queues do not exist yet: the host
creates them with admin commands ([Admin commands](#primer-admin)).

In LiteNVMe, both bring-up paths use two-entry admin queues. They poll CSTS.RDY with their own
timeouts, a number of reads in the firmware and in the init sequencer; neither reads CAP.TO
([Bring-up](#bringup)).

## Queues, heads, tails and the phase bit {#primer-queues}

A queue is a ring of fixed-size entries in host memory. The *producer* adds entries at the
*tail*; the *consumer* takes them at the *head*. Both indexes wrap to 0 after the last entry. The
figure below shows a submission queue and a completion queue.

![A submission queue with three commands between its head and its tail, and a completion queue
whose new entries carry a phase bit of 1. The indexes wrap at the end of the ring; the phase bit
flips on each pass.](primer_queue.svg)

For a submission queue (SQ), the host is the producer and the SSD the consumer. For a
completion queue (CQ), the SSD produces and the host consumes. The other side learns the new
tail or head from a doorbell write, or from a field of a completion.

A queue is empty when the head equals the tail, and full when the tail is one entry behind the
head. A queue of N entries therefore holds at most N - 1 commands. Queue sizes are written minus
one in the registers and commands, so a value of 63 means 64 entries.

The host does not read a CQ tail: none exists. It reads the next CQ entry instead and checks its
*phase bit*. The SSD writes the phase bit as 1 on its first pass over the ring, 0 on the second
pass, and alternates from then on. An entry whose phase bit equals the value expected for this
pass is new; an entry with the old value is from the previous pass.



The host starts with an expected phase of 1 on zeroed memory and flips it each time its head wraps.
A host that reads an entry dword by dword must read the phase dword first, then the others: dwords
read before it may hold the entry of the previous pass.

In LiteNVMe, the I/O engine and the firmware read the phase dword of an entry first and the other
fields after it. The I/O engine enforces queue depth at most queue size minus one
([The I/O engine](#io-engine)).

## The submission queue entry {#primer-sqe}

A command is a 64-byte submission queue entry (SQE), sixteen 32-bit dwords. The figure below
shows the fields of a Read or a Write, the only I/O commands LiteNVMe sends.

![The submission queue entry of a Read or a Write. Dword 0 holds the opcode and the command
identifier; PRP1 and PRP2 point at the data; SLBA and NLB select the sectors. The fields in grey
are zero in LiteNVMe.](primer_sqe.svg)

| Field | Dwords | Meaning |
|-------------|----------------|----------------------------------------------------------------|
| Opcode | 0 `[7:0]` | The command: `0x02` Read, `0x01` Write |
| CID | 0 `[31:16]` | Command identifier, chosen by the host, unique among the commands in flight on this queue |
| NSID | 1 | Namespace the command applies to |
| PRP1, PRP2 | 6-7, 8-9 | Data pointers ([Data pointers](#primer-prp)) |
| SLBA | 10-11 | First logical block, 64 bits |
| NLB | 12 `[15:0]` | Number of logical blocks, minus one |

The CID ties a completion to its command. A host must not reuse a CID while the command that
carries it is in flight; the SSD may reject such a command with the status Command ID Conflict.

In LiteNVMe, the I/O engine uses the SQ slot index as the CID and keeps one busy bit per slot, so
a CID is free again only once its completion has been reaped ([The I/O engine](#io-engine)).

## The completion queue entry {#primer-cqe}

The SSD reports each finished command with a 16-byte completion queue entry (CQE). The figure
below shows its four dwords and the status field.

![The completion queue entry. Dword 2 returns the SQ head; dword 3 carries the command identifier,
the phase bit and the status. The status is success when SCT and SC are both
zero.](primer_cqe.svg)

| Field | Where | Meaning |
|---------|-------------------|------------------------------------------------------------------|
| SQHD | dword 2 `[15:0]` | Submission queue head: the SQ entries up to here are consumed |
| SQ ID | dword 2 `[31:16]` | The submission queue of the command |
| CID | dword 3 `[15:0]` | The command this entry completes |
| P | dword 3 `[16]` | Phase bit |
| SC, SCT | dword 3 `[24:17]`, `[27:25]` | Status code and status code type; both zero on success |
| DNR | dword 3 `[31]` | Do not retry: the same command would fail again |

Completions can arrive in any order, which is why they carry the CID. SQHD is how the host learns
that SQ entries are free again: the SSD has fetched them, even if their commands are not finished.

In LiteNVMe, the I/O engine returns the CID, the status half of dword 3 and SQHD on its
completion stream; the firmware prints the whole entry.

## Admin commands {#primer-admin}

The admin queue carries the commands that manage the controller. A host uses the four below to go
from an enabled controller to working I/O queues; Identify is optional.

| Command | Opcode | What it does |
|----------------------|---------|----------------------------------------------------------------|
| Identify | `0x06` | Returns a 4 KiB data structure; CNS 1 selects the controller, with its model, firmware revision and MDTS |
| Set Features | `0x09` | Feature `0x07`, Number of Queues: requests the number of I/O SQs and CQs, minus one |
| Create I/O CQ | `0x05` | Queue identifier, size minus one, base address in PRP1, physically contiguous flag, interrupt enable and vector |
| Create I/O SQ | `0x01` | Queue identifier, size minus one, base address in PRP1, the CQ it completes into, priority |

The CQ comes first, because the SQ names its CQ. Each admin command is an SQE in the admin SQ,
a doorbell write and a CQE in the admin CQ, exactly like an I/O command.

In LiteNVMe, the firmware and the init sequencer send Set Features for one queue pair, then
create I/O CQ 1 and I/O SQ 1 with interrupts off. The firmware sends Identify only on request,
with its `nvme_identify` command ([Bring-up](#bringup)).

## I/O commands and namespaces {#primer-io}

The storage of an SSD is organised in *namespaces*: numbered ranges of logical blocks, each with
its own block size. Most consumer SSDs have one, namespace 1. A logical block is addressed by its
*LBA*, its number from 0.

The block size of a namespace is part of its format, 512 bytes or 4 KiB on most SSDs. A Read or a
Write moves NLB + 1 blocks starting at SLBA. Flush, which commits a volatile write cache, and
Dataset Management, which tells the SSD that blocks are unused, are the next most common I/O
commands.

In LiteNVMe, a sector is 512 bytes, the namespace is given with each request, and only Read and
Write exist. The core does not read the namespace format, so the SSD must be formatted with
512-byte blocks.

## Data pointers: PRP and SGL {#primer-prp}

A command does not carry its data. It points at buffers in host memory, which the SSD then reads
for a Write or writes for a Read. The usual pointers over PCI Express are *PRP entries*: 64-bit
addresses of memory pages, of the size set by CC.MPS.

PRP1 points at the first page, and may start at an offset inside it. What PRP2 holds depends on
the length of the transfer, as the figure below shows.

![PRP entries. PRP1 points at the first data page. For a transfer of two pages PRP2 points at the
second page; for more, PRP2 points at a PRP list whose entries point at the following pages. The
pages are drawn contiguous, as LiteNVMe uses them.](primer_prp.svg)

| Transfer | PRP2 |
|------------------------|--------------------------------------------------------------------|
| One page | Unused, zero |
| Two pages | Address of the second page |
| Three pages or more | Address of a PRP list: one 8-byte entry per following page |

A list page holds 512 entries. When a transfer needs more, the last entry of a list page points at
the next list page. The pages themselves need not be contiguous: PRPs are a scatter list at page
granularity.

The SSD also caps the transfer size of one command. MDTS, a field of Identify Controller, gives
the cap as a power of two of the smallest memory page size; 0 means no cap. *SGL* descriptors are
a second, more general pointer format, optional for an SSD.

In LiteNVMe, a data buffer is contiguous and page-aligned. The I/O engine builds PRP2 and a single
PRP list page per SQ slot itself, without chaining, and uses no SGL ([The I/O
engine](#io-engine)).

## Interrupts or polling {#primer-irq}

A computer learns that a completion has arrived from an interrupt: each CQ is tied to an
interrupt vector, MSI-X, MSI or INTx, when it is created. The section *Interrupts* of the
[PCI Express primer](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/02_pcie_primer.md)
of the LitePCIe manual explains these messages.

The host can instead create the CQ with interrupts disabled and poll its next entry. Polling
costs nothing on an FPGA, where a state machine can read a block RAM every cycle.

In LiteNVMe, the I/O CQ is created with interrupts disabled and nothing in the core handles an
interrupt: the firmware, the init sequencer and the I/O engine poll the phase bit.

## One command on the link {#primer-command}

The figure below follows one Read through the PCI Express link, from the doorbell to the
completion. The host-side steps without an arrow happen in host memory and cost no TLP.

![One Read command on the link. The host writes the SQE into its own memory and rings the SQ
doorbell; the SSD fetches the SQE, writes the data into the PRP pages, then writes the CQE. The
host sees the new phase and rings the CQ doorbell.](primer_command.svg)

Only the two doorbell writes go from the host to the SSD. Everything else is the SSD reading or
writing host memory: the SQE fetch, the PRP list fetch when there is one, the data and the CQE.

The data travels in TLPs of at most MPS bytes. A 4 KiB Read arrives as 32 MemWr of 128 bytes at the
reset MPS, or as 8 MemWr of 512 bytes with MPS raised to 512 bytes.

A Write is the mirror: the SSD sends MemRd requests of at most MRRS bytes, and the host answers with
CplD of at most MPS bytes each: a completer splits a larger read into several completions.

In LiteNVMe, the host-memory window answers all of the SSD's requests: SQE and PRP list fetches,
data and CQE writes ([The host-memory window](#host-memory)). It answers each read with one
completion and does not split it at MPS ([Known issues](#intro-issues)).

## Bring-up on the link {#primer-init}

The whole bring-up, seen from the link, is the configuration of the PCI Express function, the
enable sequence of the controller and the admin commands. The figure below shows it.

![Bring-up on the link. Configuration writes set BAR0 and the Command register; register accesses
set the admin queue and enable the controller; admin commands, each an SQE, a doorbell and a
CQE, identify the controller (optional) and create the I/O queue pair.](primer_init.svg)

After the last admin command, the I/O SQ and CQ exist and the host can send Reads and Writes.

In LiteNVMe, either the firmware or the RTL init sequencer runs this sequence, then programs the
I/O engine with the queue addresses and the doorbells and enables it ([Bring-up](#bringup)).

## Queue depth and throughput {#primer-qd}

An SSD serves a Read from its flash, which takes long compared with the transfer on the link. One
command at a time leaves the link idle most of the time. Throughput comes from keeping many
commands in flight, the *queue depth*, so that the SSD works on several at once.

Little's law gives the relation: the number of commands in flight equals the command rate times
the latency of one command. The throughput is therefore at most the queue depth times the
transfer size, divided by the latency. The link rate and the SSD's own internal parallelism cap
it from above.

Both levers matter: a larger transfer per command, and more commands in flight. A queue of N
entries allows a queue depth of N - 1.

In LiteNVMe, the I/O engine keeps up to `qd` commands in flight, 32 by default.
[Performance](#performance) gives the measured throughputs.

## Where to go next {#primer-next}

[Architecture](#architecture) shows how the core maps these mechanisms onto modules;
[Bring-up](#bringup) follows the initialization sequence step by step.
