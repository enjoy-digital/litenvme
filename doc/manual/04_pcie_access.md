# Configuration and register access {#pcie-access}

This chapter describes the accessors: the small blocks that send one request at a time from the
FPGA to the SSD, or to the root port itself. The firmware and the init sequencer use them to set
the SSD up; the I/O engine uses one of them to ring doorbells.

Read it to drive the accessors
from software or from your own logic, or to understand a register read that returns the wrong
value.

## At a glance {#acc-glance}

| Accessor | Reaches | Requests | Registers |
|-------------------------------|------------------------------|-------------------|-----------|
| Configuration accessor | Configuration space of the SSD | CfgRd0, CfgWr0 | Yes |
| MMIO accessor | Controller registers in BAR0 | MemRd, MemWr | Yes |
| Doorbell accessor | Doorbells in BAR0 | MemWr | No |
| Root configuration accessor | Configuration space of the root port | cfg_mgmt port | Yes |

The first three send TLPs through master ports of the LitePCIe crossbar and share one design,
described first. The root configuration accessor sends no TLP: it drives a side port of the hard
IP, and has its own section.

## The shared accessor {#acc-shared}

The configuration and MMIO accessors are two subclasses of one base, `_LiteNVMePCIeAccessor`,
which holds the request, completion and status logic. A subclass only adds the fields of its
request (the address, or the bus, device and register numbers) and its registers. The figure
below shows the state machine of the base.

![State machine of the shared accessor. Follow the top row for a read: the request is sent, then
the accessor waits for the last beat of its completion. The bottom row holds the other outcomes:
a rejected write, a posted write and a timeout. Every outcome sets `done` and returns to
IDLE.](acc_fsm.svg)

One access works as follows:

1. The caller sets the request fields, then raises `start`. The accessor reacts to the rising
   edge of `start` only, and clears `done` and `err` at that edge.
2. In SEND, the accessor offers a one-beat request to its crossbar port and waits until the port
   accepts it. Writes put the 32-bit write data on every dword of the beat.
3. A posted write ends there (WRITE-LATCH). A read, or a configuration write, waits in WAIT for
   its completion.
4. In WAIT, the accessor keeps the first dword of the first completion beat as `rdata` and sets
   `err` if the completion reports an error. It leaves WAIT on the beat that ends the
   completion, not on its first beat.
5. `done` rises and stays set, with `err` and `rdata`, until the next `start`.

A completion that never arrives ends the access after 2^20 cycles in WAIT (TIMEOUT, about 8.4 ms
at 125 MHz), with `done` and `err` set. SEND has no timeout: the accessor waits there as long as
the port does not accept the request. The caller polls `done`, then reads `err` and `rdata`.

A timeout does not clear the read in the TLP controller of the crossbar: the later reads of all
three master ports wait behind it and time out too, until the core is reset
([Known issues](#intro-issues)).

### Stale completions {#acc-stale}

The three master ports of the core share the TLP controller of the crossbar, which returns
completions in request order and releases a request only when its last completion beat is
consumed.

A completion beat left unread would stay at the head of the queue and be taken by the next read
of the same port, which would then return the data of the previous request.

The accessor avoids this in two ways. It accepts and drops any completion beat while it is in
IDLE or SEND, so a completion that arrives late cannot be matched to the next request.

And it retires a read only on the beat marked as the end of the completion: at 256 bits, a
one-dword completion can take more than one beat.

## Configuration accessor {#acc-cfg}

The configuration accessor (`LiteNVMePCIeCfgAccessor`) sends Type 0 configuration requests,
CfgRd0 and CfgWr0, of one dword. It reads the identity of the SSD, sizes and assigns BAR0, sets
the Memory Space and Bus Master Enable bits of the Command register, and walks the capability
list. Configuration writes are not posted: a write waits for its completion like a read.

| Field | Width | Meaning |
|------------|-------|----------------------------------------------------------|
| `bus` | 8 | Bus number of the target function |
| `dev` | 5 | Device number |
| `fn` | 3 | Function number |
| `reg` | 6 | Register number, the byte offset divided by 4 |
| `ext` | 3 | Extended register number, the upper bits of the dword index |

The register and extended register numbers form a 9-bit dword index, so the accessor reaches the
first 2 KiB of the 4 KiB configuration space of a PCIe function. The capabilities LiteNVMe uses,
the PCI Express capability among them, are in the first 256 bytes.

The firmware and the init sequencer address the SSD as bus 0, device 1, function 0, and both
ignore `err` on configuration writes: the write takes effect even when its completion reports an
error, and the next read checks the result. The firmware also discards the first read of a
register before it trusts one (`cfg_rd32_retry` in `bench/firmware/main.c`).

## MMIO accessor {#acc-mmio}

The MMIO accessor (`LiteNVMePCIeMmioAccessor`) sends MemRd and MemWr requests with a 64-bit bus
address, to the controller registers in BAR0. The firmware and the init sequencer use it for CAP,
CC, CSTS, AQA, ASQ, ACQ and the admin doorbells.

| Field | Width | Meaning |
|--------|-------|---------------------------------------------------------------|
| `adr` | 64 | Byte address of the register on the PCIe bus |
| `we` | 1 | 0 for a read, 1 for a write |
| `wsel` | 4 | Byte enables of a write; only `0xf` (whole dword) is accepted |
| `len` | 10 | Length in dwords; use 1 |
| `wdata` | 32 | Write data |

Memory writes are posted: the accessor reports `done` as soon as the crossbar accepts the
request, without any answer from the SSD. A write with `wsel` other than `0xf` is not sent; the
accessor ends it at once with `err` set (BAD-WSEL).

A read returns the first dword of its completion only, so a read with `len` above 1 yields one
dword.

The controller registers of 64 bits (CAP, ASQ, ACQ) take two accesses, low dword first. Because
a posted write gives no answer, the firmware follows a series of writes with a read of CSTS, which
returns only after the writes have reached the SSD. It also reads each register until two reads
agree (`mmio_rd32` in the firmware).

## Doorbell accessor {#acc-doorbell}

The doorbell accessor is a second MMIO accessor, built without registers, that belongs to the
I/O engine. It sends the MemWr that ring the SQ and CQ doorbells of the I/O queue. A dedicated
accessor lets the engine ring doorbells while the firmware owns the MMIO accessor.

The engine drives it through `connect_mmio()`: it pulses `start` and holds the address and the
data until `done`, since the accessor reads them while it waits in SEND. The engine does not look
at `err`. The accessor has no timeout in SEND, so a port that never accepts the request stalls the
engine. See [The I/O engine](#io-engine).

## Root configuration {#acc-rootcfg}

The root port has a configuration space of its own, which no enumeration software programs in an
embedded design.

Two of its registers matter to LiteNVMe. The Device Control register of the PCI Express capability
holds the MPS of the root port, and the Memory Base and Limit register (byte offset `0x20`) is the
bridge memory window that lets requests to BAR0 reach the SSD.

The root configuration accessor (`LiteNVMeRootCfgMgmt`) reaches this space through the cfg_mgmt
port of the hard IP, without any TLP. It needs a PHY built with `with_cfg_mgmt=True`; the
7-Series and UltraScale+ PHYs of LitePCIe provide it, and the generator uses the UltraScale+ one.

The figure below shows how it crosses from `sys` to the `pcie` domain of the port.

![Root configuration accessor. Look at the dotted clock boundary: the start and done pulses and
the read data cross it through synchronizers; the address, data and byte enables are held static
during an access and driven straight to the hard IP.](acc_rootcfg.svg)

| Field | Width | Meaning |
|---------|-------|---------------------------------------------------|
| `addr` | 10 | Dword address in the root port configuration space |
| `func` | 8 | Function number, 0 |
| `wdata` | 32 | Write data |
| `be` | 4 | Byte enables, `0xf` at reset |
| `write` | 1 | 0 for a read, 1 for a write |

A pulse on `start` sets `busy` and clears `done`. The transaction state machine, in the `pcie`
domain, asserts the read or write strobe of the port until the hard IP reports completion, keeps
the read data and sends a pulse back. `done` then rises in `sys`, with `rdata` valid.

The address is a dword address: byte offset `0x20` is address `0x08`. The PCIe side of MPS, and
why it stays at 128 bytes until both ends are programmed, is described in the section *Root port
designs* of the
[Integration chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/12_integration.md)
of the LitePCIe manual.

## Registers {#acc-registers}

Each accessor built with registers has one bank. The table below gives the registers by bank
suffix; the full layout, with the names of a generated SoC, is in
[Register reference](#registers).

| Accessor | Registers |
|------------------------------|----------------------------------------------------------------------|
| Configuration accessor | `cfg_ctrl` (start, we), `cfg_bdf`, `cfg_wdata`, `cfg_stat` (done, err), `cfg_rdata` |
| MMIO accessor | `mem_ctrl` (start, we, wsel, len), `mem_adr_l`, `mem_adr_h`, `mem_wdata`, `mem_stat`, `mem_rdata` |
| Root configuration accessor | `ctrl` (start, write), `addr`, `func`, `wdata`, `be`, `stat` (done, busy), `rdata` |

The start fields are pulses: each write of 1 starts one access. Software
writes the address, data and write registers first, then the control register, then polls the
status register until `done` is 1.

## In the code {#acc-code}

The shared base and the MMIO accessor are in `litenvme/mem.py`; the configuration accessor and the
root configuration accessor are in `litenvme/cfg.py`. The firmware helpers that drive them
(`cfg_rd32`, `mmio_rd32`, `rootcfg_op`) are in `bench/firmware/main.c`.

## Where to go next {#acc-next}

[The host-memory window](#host-memory) describes the other side of the root port, the requests
the SSD sends; [Bring-up](#bringup) puts the accessors in sequence.
