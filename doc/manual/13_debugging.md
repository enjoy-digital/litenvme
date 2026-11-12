# Debugging {#debugging}

This chapter explains how to find why an SSD does not come up or runs slower than expected,
without a PCIe protocol analyzer. It describes the method, the tools built into the bench SoC and
the firmware, then goes through the failures in the order of the bring-up, and ends with a
troubleshooting table. Read it before touching hardware.

## Method {#debug-method}

The approach is to observe before capturing: registers and firmware messages answer most
questions, and a logic analyzer capture is worth taking only once the question is narrow. It is
the method of the LiteX wiki page
[Debug a SoC on hardware](https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware).

1. Check the connection to the board: Etherbone answers and the identifier matches the build
   (`litex_cli --csr-csv csr.csv --ident`).
2. Check the PCIe link: `link_status` of the PHY
   (`litex_cli --csr-csv csr.csv --regs --filter link_status`, [Link status](#debug-link)).
3. Follow the bring-up step by step from the firmware console, and read the registers of the
   failing block.
4. Capture a LiteScope trace of the PCIe ports only when the question is about the TLPs
   ([LiteScope](#debug-litescope)).
5. Reproduce the finding in simulation, fix it there, then check again on the board
   ([Simulation and verification](#simulation)).

Two rules keep the observations valid. Only one program reads the crossover UART of the
firmware at a time, otherwise each reader gets part of the characters (`bench/engine_console.py`
is the single reader of the harness scripts). And a result counts only with its complete capture:
the whole firmware output of the command, not a partial line.

## Tools {#debug-tools}

The table below lists what the bench SoC and the firmware provide for debugging. The bench SoC,
the firmware commands and the harness scripts are described in
[Bench, firmware and host tools](#examples).

| Tool | Shows | Needs |
|------------------------|----------------------------|----------------------------------|
| `link_status` register | Link up, rate, width, LTSSM state | Any build |
| Firmware console | Each bring-up step, CQE decode, errors | `--with-cpu` |
| `status` command | `link_status` and the window beat counters | `--with-cpu` |
| Window debug registers | Any dword of the host-memory window | `with_hostmem_csr` (bench: on) |
| Init sequencer registers | Fail code, last CQE, last reads | `--with-rtl-init` |
| LiteScope | PCIe ports, window port, counters | `--litescope-probe pcie` |
| Etherbone and `RemoteClient` | Every register by name from the bench PC | `--with-etherbone` |

The register names are those of the `csr.csv` of the build. The bench SoC names its blocks
differently from the `LiteNVMe` class, for example `cfg_cfg_*` on the bench SoC and
`nvme_cfg_cfg_*` with `LiteNVMe` ([Register reference](#registers)). The link register is
`pcie_phy_link_status` in both, and the bench scripts and the firmware use that name.

## Link status {#debug-link}

`link_status` of the PHY packs the state of the link in one register. Read it first: no NVMe
traffic is possible before the link is up at the expected rate and width.

| Field | Bits | Meaning |
|--------------|---------|--------------------------------------------------------|
| `status` | 0 | 1: link up |
| `phy_down` | 1 | 1: PHY link down |
| `phy_status` | 3:2 | 3: link up, data link layer initialized |
| `rate` | 5:4 | 0: Gen1, 1: Gen2, 2: Gen3 |
| `width` | 8:6 | 0: x1, 1: x2, 2: x4, 3: x8, 4: x16 |
| `ltssm` | 14:9 | LTSSM state, `0x10` is L0 |

The Gen3 logs of `bench/results/` read `0x20ad`: up, data link initialized, Gen3, x4, LTSSM
`0x10`. The harness function `hw_link_decode` of `bench/hw_common.sh` prints this decoding with
the MPS and MRRS registers of the PHY.

A link that stays down has three usual causes:

- **PERST# held.** The SSD stays in reset until PERST# is released. The generated core releases
  it 100 ms after its reset ([Known issues](#intro-issues) lists a case where it may not); in a
  custom SoC, check that the design drives the PERST# pin of the slot and releases it
  ([Integration](#integ-parts)).
- **No reference clock.** The hard IP needs a 100 MHz reference clock on the `pcie_clk` pins of
  the card.
- **Wrong lanes or quad.** `phy_config` selects the transceiver quad and the block location
  (`select_quad`, `pcie_blk_locn` in `examples/alibaba_xcku3p.yml`); a wrong choice builds, but
  the link never trains.

The PCIe side of link training is explained in the
[Debugging chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/16_debugging.md)
of the LitePCIe manual.

## Configuration and BAR0 {#debug-config}

Once the link is up, the configuration accessor reads the configuration space of the SSD. The
firmware commands that talk to the controller start by assigning BAR0. The first time, or when
the address changes, they print the vendor and device IDs, the original BAR0, its size and the
assigned address.

| Symptom | Likely cause | Check |
|------------------------------|------------------------------------------|----------------------------------|
| `err` set on every configuration read | Link not up, or wrong BDF | `link_status`; the SSD is at bus 0, device 1, function 0 on the bench |
| Read data of the previous request | Stale completion | Fixed in the accessor; the firmware also reads until two reads agree |
| Registers above 2 KiB not reachable | `ext` is 3 bits, `reg` 6 bits | Reach is the first 2 KiB of the configuration space |

The accessor keeps one request in flight and retires it on the last beat of its completion, so a
late completion of an earlier request cannot be taken for the current one
([Configuration and register access](#pcie-access)).

## Controller registers {#debug-bar0}

The MMIO accessor reaches the controller registers in BAR0 only when two conditions hold: the
Command register of the SSD has MEM set (and BME, for the SSD to reach the window later), and the
root port forwards the address range of BAR0.

- **Memory window of the root port.** A root port forwards memory requests downstream only inside
  its bridge memory window. The firmware writes `0xe000e000` to the dword at byte `0x20` of the
  root port configuration (memory base and limit), covering `0xe0000000` to `0xe00fffff`, through
  the root configuration accessor. Without it, MMIO reads of BAR0 return no data.
- **CAP reads 0.** The firmware retries CAP, then stops with
  `ERR: CAP is 0 (BAR0 not responding).`; the init sequencer stops with fail code 2. Check MEM
  and BME (`cmd_enable` sets them), the window, and the assigned BAR0 address.
- **Posted writes.** A write to BAR0 gets no completion: `done` only means the request left. The
  firmware reads a register back after each group of writes (`flush read`) to know that the
  writes arrived. It prints a warning such as
  `WARN: CC write err=1 (write may still be accepted).` when the accessor reports an error on a
  write.

`bench/results/gen3_256b_2026-06-01/nvme_init.log` records a self-test of these steps
(`nvme_mmiotest`, a command of an earlier firmware build), quoted in
[BAR0 and MMIO self-test](#ex-mmiotest): the register values it prints are a reference for a
working setup.

## Controller enable {#debug-enable}

After the admin queue registers, the host sets CC.EN and waits for CSTS.RDY
([NVMe over PCIe in a nutshell](#nvme-primer)).

- **`WARN: CSTS.RDY did not assert.`** The firmware polls a fixed number of loops
  (`NVME_RDY_SET_LOOPS`), not the time given by CAP.TO. A slow SSD may need more; check that
  AQA, ASQ and ACQ read back what was written before CC.EN.
- **Init sequencer fail code 3.** Same condition in the init sequencer, after its poll timeout of
  2^22 cycles.
- **`WARN: CSTS.RDY did not clear.`** When the firmware disables a controller that was enabled by
  an earlier run, it waits for CSTS.RDY to fall before reprogramming the queues. `nvme_reset`
  clears the state of the firmware and disables the controller.

## Admin commands {#debug-admin}

An admin command is an SQE in the window, a doorbell write, a fetch of the SQE by the SSD, and a
CQE written back by the SSD. The firmware gives the CQE 100 ms (`NVME_CQE_TIMEOUT_TICKS`) before
`ERR: Admin CQ timeout.`; the init sequencer reports fail code 4, or 5 for a CQE with an error
status.

When the CQE does not arrive, the beat counters of the window tell where the exchange stops. The
`status` command prints `hostmem_wr_count` and `hostmem_rd_count`: beats written and read by the
SSD.

| Counters after the doorbell | Meaning | Check |
|------------------------------|------------------------------------------|----------------------------------|
| Read count unchanged | The SSD never fetched the SQE | Doorbell address (CAP.DSTRD), BME, ASQ value |
| Read count up, write count unchanged | Fetched, no CQE or data written back | Completion of the fetch, see below |
| Write count up, CQE wrong or in the wrong slot | Data placement | Sub-beat writes, window decoder |

- **Address decoder.** The slave port of the core accepts only requests inside
  `[hostmem_base, hostmem_base + hostmem_size)`. A PRP or a queue address outside the window is
  silently ignored, and a MemRd there gets no completion.
- **Completion of the fetch.** The window drives the Byte Count and Lower Address of each CplD,
  and every LitePCIe PHY sends them as given. A Byte Count of 0 means 4096 bytes to the SSD, which
  then waits for the rest of the SQE forever: a clean link, a served MemRd and no CQE. A read
  completion larger than the MPS of the link is another cause ([Known issues](#intro-issues)).
- **Sub-beat writes.** A 16-byte CQE lands in the middle of a 256-bit beat. The window shifts it to
  its dword offset and writes only its dwords, so the neighbouring CQ entry is kept
  ([The host-memory window](#host-memory)).

The window debug registers read any dword of the window: write the dword index to `csr_adr`, then
read `csr_rdata` twice and keep the second value, since the first read returns the dword of the
previous address (`bench/hostmem_tool.py` and the firmware do the same).

## I/O commands {#debug-io}

The firmware prints each CQE it decodes: SQ head, SQ ID, CID and the status fields P, SCT, SC, M
and DNR. With the I/O engine, the request generator keeps the first error: `nvme_engine_bench`
prints an `err0` line with the status, SC, SCT, CID and index of that command when `errors` is
not 0.

SCT 0 is the generic status type of the NVM Express Base Specification. The engine never reuses
a CID still in flight (per-slot busy bits); its code `0x03`, Command ID Conflict, near the wrap of
the ring points to a CID collision.

Other codes point to the command itself: check the sector count against MDTS, the LBA range of the
namespace and the PRP addresses.

The engine reads dword 3 of a CQE, which holds the phase bit, before the other dwords; this avoids a
torn read at the wrap of the ring ([The I/O engine](#io-engine)).

The engine registers `submitted` and `completed` count from reset, and `inflight` of `status` gives
the commands waiting for a CQE: an engine stuck with `inflight` above 0 waits for CQEs that never
come.

## Throughput {#debug-throughput}

Measured throughputs, and how the firmware computes them, are in
[Performance and resources](#performance). A run well below them usually has one of these causes:

- **MPS left at 128 bytes.** MPS stays at its reset value until both the root port and the SSD
  are programmed, root first. The firmware does it (`MPS auto-set:` line); the init sequencer does
  not. A 4 KiB read then takes 33 MemWr TLPs instead of 9 at 512 bytes.
- **Few outstanding reads.** `ep_max_pending_requests` limits the read requests of the root port
  in flight; the example YAML notes that 2 instead of 8 costs read throughput.
- **Small transfers.** Each command has a fixed cost; larger transfers amortize it.
- **Low queue depth.** `max_inflight` of `nvme_engine_bench` shows the queue depth reached.

The window counters separate the SSD from the core: `hostmem_rd_stall_cycles` near 0 means the
window accepts read data as fast as it arrives, so the gaps come from the SSD or the link.

## LiteScope {#debug-litescope}

The bench SoC built with `--litescope-probe pcie` adds a LiteScope analyzer of 2048 samples in the
`sys` domain. The build writes the signal list to `analyzer.csv` in the current directory.

The analyzer records the PCIe reset and the fields of `link_status`, the four streams of the PHY,
the configuration and memory master ports with their request and completion fields, the slave
port of the window and its completions, and the two beat counters of the window.

```sh
# From bench/, with litex_server running over Etherbone (bench/hw_common.sh, hw_serve).
# List the signals of the capture, then trigger on one of them.
litescope_cli --csr-csv build/alibaba_xcku3p/csr.csv --csv analyzer.csv --list
litescope_cli --csr-csv build/alibaba_xcku3p/csr.csv --csv analyzer.csv \
    -r <signal> --dump nvme.vcd
```

A useful first trigger is the `valid` of the window slave port: the capture then shows the first
request of the SSD, a MemRd for an SQE or a MemWr for a CQE, and the completion the window
returns. The LiteScope method (groups, triggers, depth) is in the LiteScope manual.

## Troubleshooting {#debug-table}

| Symptom | First check | Then |
|--------------------------------|--------------------------------|------------------------------------------|
| `link_status` bit 0 at 0 | PERST#, reference clock | `phy_config` quad and location |
| Link at Gen1 or narrower than x4 | Adapter, slot wiring | LTSSM state, link training in LitePCIe |
| Configuration reads with `err` | Link up, BDF 0:1:0 | Configuration accessor timeout |
| CAP reads 0 | MEM and BME, root memory window | BAR0 address, MMIO accessor |
| CSTS.RDY never set | AQA, ASQ, ACQ read back | Longer wait, CAP.TO |
| Admin CQ timeout | Window beat counters | Doorbell address, decoder, CplD Byte Count |
| CQE in the wrong slot | Sub-beat writes of the window | Data width of the window |
| Fail code 1 to 5 (init sequencer) | Step of the code, `dbg_*` registers | Same step from the firmware console |
| I/O error status | `err0` line, SC and SCT | NLB, LBA range, PRP addresses |
| Throughput low | MPS line, `max_inflight` | Transfer size, `ep_max_pending_requests` |
| Garbled console output | Single reader on the crossover UART | Restart `engine_console.py` |
