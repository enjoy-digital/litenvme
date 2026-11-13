# Bring-up: firmware and RTL init sequencer {#bringup}

This chapter describes how the SSD goes from reset to a working I/O queue: the steps of the
NVMe initialization and the two implementations of the project. The firmware runs them on a soft
CPU; the init sequencer runs them in a state machine, without a CPU.

Read it to choose one, or to find the step where a bring-up stops.

## At a glance {#boot-glance}

| Item | Firmware | Init sequencer |
|---------------------|----------------------------------|-------------------------------------|
| Runs on | Soft CPU (VexRiscv in the bench SoC) | Gateware, `LiteNVMeInitSequencer` |
| Entry point | `nvme_setup` console command, or at boot with `MINIMAL=1` | Pulse on `start` |
| BAR0 | Sized, then assigned | Assigned at a fixed address |
| Doorbell stride | Read from CAP.DSTRD | Assumed 0 (4 bytes) |
| MPS | Raised on both ends | Left at reset value (128 bytes) |
| Identify | Separate command (`nvme_identify`) | Not sent |
| Failure report | Console messages, `init_error` | `init_error`; a failure code on the bench SoC |
| Source | `bench/firmware/main.c` | `litenvme/init.py` |

The status of each path is in the [status table](#intro-status).

## The common sequence {#boot-sequence}

Both paths run the steps of the figure below. The firmware programs the root port just after BAR0,
before the Command register, and only when it finds the PCI Express capability of both ends.

Each step uses one of the accessors of [Configuration and register access](#pcie-access), and the
admin commands go through the host-memory window like the commands of the I/O engine.

![Bring-up steps shared by the firmware and the init sequencer. Read each box as one step and the
path it uses: configuration space first, then the root port, the controller registers, the admin
queue and the I/O engine.](boot_sequence.svg)

1. **Configuration.** Read the vendor and device identifiers and check that they are valid (the
   init sequencer retries 16 times), write the
   address of BAR0 (`0xe0000000` on the reference design), then set Memory Space and Bus Master
   Enable in the Command register. The SSD now answers memory requests and may send its own.
2. **Root port.** Program the memory window of the root port, the Memory Base and Limit register,
   so that requests to BAR0 go down to the SSD. The value `0xe000e000` covers `0xe0000000` to
   `0xe00fffff`. The firmware also raises the MPS of both ends here.
3. **Controller.** Read CAP until it is not zero, which shows that BAR0 answers. Write AQA (two
   entries per admin queue), ASQ and ACQ, then CC with EN set, 64-byte SQEs, 16-byte CQEs and a
   memory page size of 4 KiB. Poll CSTS.RDY until it is 1.
4. **Admin commands.** Send Set Features (Number of Queues, one SQ and one CQ), Create I/O CQ
   (queue 1, physically contiguous, interrupts disabled), then Create I/O SQ (queue 1, bound to
   CQ 1). Each command waits for its CQE and checks its status.
5. **I/O engine.** Write the addresses of the I/O SQ, the I/O CQ, the two doorbells and the
   PRP-list region into the engine, then set `enable`.
6. **Done.** Raise `init_done`, or `init_error` if a step failed.

An admin command follows the same pattern in both paths: write the SQE into the admin SQ, write
the next CQ slot with the opposite of the expected phase so that an old entry cannot match, ring
the admin SQ doorbell, poll the CQE until its phase bit matches, check its status, then ring the
admin CQ doorbell.

## PERST# {#boot-perst}

The SSD must stay in reset until its power and reference clock are stable. The bench SoC and the
generator drive PERST# from a one-shot timer: it holds PERST# low for 100 ms after the FPGA reset,
then releases it, together with the reset input of the PCIe PHY. Link training starts there.

In the generator, the end of this timer also starts the init sequencer; its configuration probe
retries 16 times, without delay, then fails (see [Timeouts and failure codes](#boot-fail)). The
timer of the generator runs in a clock domain that the hard IP holds in reset, which may keep
PERST# low forever ([Known issues](#intro-issues)).

## Firmware {#boot-firmware}

The firmware runs bring-up in the console command `nvme_setup`, which takes an optional BAR0
address. It calls the steps above, programs the engine and writes 1 to `init_done`, or to
`init_error` on a failure. It handles more cases than the init sequencer:

- It sizes BAR0 by writing all ones and reading the mask back, and checks that the size is a
  power of two and that the base is aligned to it.
- It raises the MPS of the root port, through the root configuration accessor, and of the SSD, to
  the largest size both support. It changes the MPS field of Device Control only and leaves MRRS
  at its reset value of 512 bytes.
- It disables the controller first if CC.EN is already set, and waits for CSTS.RDY to clear.
- It reads CAP.DSTRD and computes the doorbell addresses with that stride.
- It reads each controller register until two reads agree, and discards the first read of a
  configuration register.

The firmware treats a CSTS.RDY that never rises as a warning and goes on to the admin commands.
Identify Controller is a separate console command, `nvme_identify`, not part of `nvme_setup`.
[Examples](#examples) lists the other console commands.

The firmware layout puts the I/O queues at fixed offsets. On the bench SoC it creates them with
64 entries, in a window at `0x10000000`, and the engine of the SoC must be built with the same
queue size. A generated core gives the base of the window, the queue size and the address of the
PRP-list region to the firmware, as the constants `LITENVME_HOSTMEM_BASE`,
`LITENVME_IO_Q_ENTRIES` and `LITENVME_PRP_LIST_BASE` of `generated/soc.h`.

The firmware is written with the register names of the bench SoC. When they are absent, it maps
them to the names of the `nvme` bank of a generated core (`nvme_cfg_*`, `nvme_mmio_*`,
`nvme_io_engine_*`, `nvme_hostmem_*`, `nvme_request_gen_*`), so the same source initializes both.
[Integration](#integration) gives the details.

The firmware has two build options in `bench/firmware/Makefile`. `BOOT=rom` links it to run from
the integrated ROM instead of being loaded by the BIOS. `MINIMAL=1` builds a firmware without
console or messages that runs `nvme_setup` at boot, then idles; it is meant for a standalone core
with a small ROM.

## Init sequencer {#boot-rtl}

The init sequencer (`LiteNVMeInitSequencer`) runs the same steps from constant tables, with no
CPU and no software. It drives the configuration accessor, the MMIO accessor and the root
configuration accessor directly, writes admin commands into the window through its own dword
bridge, and drives the configuration signals of the I/O engine. The figure below shows its states.

![States of the init sequencer. Follow the chain from the configuration probe to READY: each box is
one step, and the dotted lines show the steps that can fail, with the failure code they leave in
`fail_code`.](boot_rtl_fsm.svg)

| Parameter | Default | Meaning |
|------------------------|-----------------|---------------------------------------------------|
| `bar0_base` | `0xe0000000` | Address written to BAR0 |
| `hostmem_base` | `0x10000000` | Base of the window; the queues are at fixed offsets |
| `admin_q_entries` | 2 | Entries of the admin SQ and CQ |
| `io_q_entries` | 64 | Entries of the I/O queues; the generator passes `qsize` |
| `io_qid` | 1 | I/O queue identifier |
| `mem_window_dw` | `0x08` | Dword address, on the cfg_mgmt port, of the Memory Base and Limit register of the root port (byte `0x20`) |
| `mem_window_val` | `0xe000e000` | Value of the memory window |
| `poll_timeout` | 2^22 | Polls before a timeout |
| `prp_list_base` | `None` | Bus address of the PRP-list region; `None` places it at `+0x10000` plus one page per I/O queue entry |

The sequencer relies on the reference set-up and checks none of it:

- The SSD is at bus 0, device 1, function 0, and BAR0 is a 64-bit memory BAR; the sequencer writes
  BAR0 and BAR1 without sizing them.
- The doorbell stride is 4 bytes (CAP.DSTRD of 0), so the doorbells are at fixed addresses.
- The controller is disabled when the sequence starts, as after PERST#: the sequencer sets CC.EN
  without clearing it first.
- The I/O CQ is zero, as the BRAM is after the FPGA is configured.

It sends no Identify. It leaves the MPS of both ends at 128 bytes and the MRRS of the SSD at 512
bytes, which the window does not support for reads above 128 bytes (see
[The responder](#mem-responder)).

It writes the memory window value at the cfg_mgmt dword address `mem_window_dw`. The default,
`0x08`, is the Memory Base and Limit register at byte `0x20` of the root port, the register that
the firmware writes too.

### Timeouts and failure codes {#boot-fail}

The configuration probe is retried up to 16 times, back to back. CAP, CSTS.RDY and each admin
CQE are polled up to `poll_timeout` times; a poll of CAP or CSTS is a full MMIO read, so the time
depends on the latency of the SSD.

Configuration write errors are ignored, as in the firmware, and the CAP poll checks that BAR0
took its address.

| `fail_code` | Step | Meaning |
|---------------|-------------------|------------------------------------------------------------|
| 1 | Configuration probe | No valid identifier: link down or SSD not at 0:1:0 |
| 2 | CAP poll | CAP stays 0: BAR0, memory window or Memory Space problem |
| 3 | CSTS.RDY poll | The controller never became ready |
| 4 | Admin CQE poll | No CQE: the SSD cannot write the window or the doorbell is lost |
| 5 | Admin CQE status | An admin command failed; `dbg_cqe3` holds dword 3 of its CQE |

A failure stops the sequencer with `init_error` set; only a reset starts it again.

The generator builds it without registers: the standalone core reports `init_error` only. With
`with_csr=True`, as on the bench SoC, the sequencer has registers: `ctrl` (`start`), `status`
(`done`, `error`, `busy`, `fail_code`, `cmd`, the index of the admin command in progress), and
`dbg_cqe3`, `dbg_mmio` and `dbg_cfg` with the last values read. [Debugging](#debugging) uses them.

## Choosing a path {#boot-compare}

| Criterion | Firmware | Init sequencer |
|-------------------|-----------------------------------|----------------------------------------|
| Needs a CPU | Yes, with ROM or RAM for the code | No |
| Adapts to the SSD | BAR size, DSTRD, MPS, CC.EN state | Fixed values for the reference set-up |
| Diagnostics | Console messages, Identify, register dumps | Failure code and three debug registers |
| Changes | Edit C code | Edit the tables in `init.py` |
| Throughput | MPS raised to the common maximum | MPS of 128 bytes |

The firmware is the path to bring up a new SSD or board: it reports each step and adapts to more
controllers. The init sequencer suits a fixed design once the SSD is known, and saves the CPU and
its memories.

MPS matters for reads: at 128 bytes, the SSD sends read data in four times more packets than at
512 bytes; see [Performance](#performance).

## In the code {#boot-code}

The firmware steps are `nvme_bar0_assign`, `nvme_configure_mps_once`, `nvme_admin_init`,
`nvme_io_setup` and `nvme_autoinit` in `bench/firmware/main.c`. The init sequencer is
`litenvme/init.py`. The PERST# timer is in `bench/alibaba_xcku3p.py` and `litenvme/gen.py`.

## Where to go next {#boot-next}

[Getting started](#getting-started) runs the firmware path on the bench;
[Integration](#integration) chooses the path in a SoC or a standalone core.
