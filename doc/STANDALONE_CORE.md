# LiteNVMe Standalone Core

A **standalone, configurable NVMe host core** — a PCIe RootPort that brings up an NVMe SSD
and exposes a simple LiteSATA-style block read/write interface, generated as a self-contained
Verilog file (plus C headers) from a YAML config, exactly like `litedram_gen` / `litepcie_gen`.

The core embeds everything needed to talk to the SSD:

```
  PCIe pads ── USPPCIEPHY (RootPort) ── LitePCIeRootPort ── LiteNVMe datapath
                                                              ├ cfg/mmio accessors
                                                              ├ host-memory window (BRAM)
                                                              ├ I/O command engine (QD32)
                                                              └ block streamer  ─── wr_sink / rd_source
  embedded VexRiscv + firmware (ROM) ── one-time bring-up ── status.init_done
```

A small soft-CPU runs the NVMe bring-up firmware at boot (BAR0 assign, controller enable,
create the IO SQ/CQ, program + enable the I/O engine) and raises `init_done`. After that the
block streaming interface is hardware-driven: you push a `{write, sector, count}` command and
stream the data — no PRP / host-memory knowledge required, no per-command CPU cost.

> Scope: the current generator targets **Xilinx Ultrascale+** (USPPCIEPHY, `pcie4_uscale_plus`
> hard IP, Gen3 x4, 256-bit datapath). Other LitePCIe PHYs can be added to `litenvme/gen.py`.

---

## 1. Generating the core

```bash
# from the litenvme repo root
litenvme_gen examples/alibaba_xcku3p.yml --output-dir build --name litenvme_core
# (or: python3 -m litenvme.gen examples/alibaba_xcku3p.yml ...)
```

Outputs under `--output-dir`:

| File | Contents |
|------|----------|
| `gateware/litenvme_core.v`            | the standalone core (one Verilog module, named by `--name`) |
| `gateware/litenvme_core.tcl`          | Vivado script that generates the PCIe hard IP + assembles the project |
| `gateware/litenvme_core.xdc`          | timing/placement constraints (PCIe IP, clocks) |
| `gateware/litenvme_core_rom.init`     | embedded-CPU ROM image (the baked NVMe firmware, when `firmware: auto`) |
| `software/include/generated/csr.h`    | CSR map (for the embedded CPU / optional host access) |
| `software/include/generated/{soc,mem}.h` | SoC constants / memory-region map |

When `firmware: "auto"`, the generator runs the **two-pass firmware build** automatically
(emit headers → compile the NVMe firmware → bake it into the CPU ROM), so the produced core is
self-initializing. With `firmware: "none"` the core boots the LiteX BIOS instead (useful to
validate generation).

### YAML configuration

```yaml
{
    # PCIe PHY (RootPort).
    "phy"          : "USPPCIEPHY",
    "phy_device"   : "xcku3p-ffvb676-2-e",
    "phy_lanes"    : 4,
    "phy_speed"    : "gen3",
    "phy_ip_name"  : "pcie4_uscale_plus",
    "data_width"   : 256,               # core/PHY datapath width (256b @125MHz = 4.0 GB/s).
    "phy_config"   : { "select_quad": "GTY_Quad_225", "pcie_blk_locn": "X0Y0",
                       "plltype": "QPLL0", "axisten_freq": 125, "coreclk_freq": 250, ... },

    "clk_freq"     : 125e6,             # sys clock (derived from the PCIe user clock).
    "clk_external" : False,            # False: core drives clk/rst out; True: user drives them in.

    "cpu"          : "vexriscv",        # embedded bring-up CPU (None => CPU-less, see below).
    "cpu_variant"  : "minimal",
    "uart"         : "serial",          # serial | crossover | stub.
    "firmware"     : "auto",            # auto: bake NVMe init firmware into ROM; none: LiteX BIOS.

    "qid"          : 1, "qsize": 64, "qd": 32,   # I/O queue id / ring size / commands outstanding.
    "hostmem_base" : 0x10000000,        # host-memory window (queues + PRP buffers + staging).
    "hostmem_size" : 0x80000,           # 512 KiB on-FPGA BRAM.

    "with_request_gen"    : False,      # CSR benchmark generator (mutually exclusive front-end).
    "with_block_streamer" : True,       # the block read/write data interface (the product front-end).
    "staging_base" : 0x40000, "staging_size": 0x40000,   # streamer staging region in the window.
}
```

Key knobs:
- **`data_width`** — must match the PHY's native width (256 for Gen3 x4 here). 256b @ 125 MHz = 4.0 GB/s.
- **`firmware`** — `auto` bakes the NVMe bring-up firmware into ROM (self-initializing core);
  `none` boots the BIOS.
- **`with_block_streamer` (sole front-end)** — exposes the block interface at the top
  (pin-driven). With `with_request_gen` instead you get a CSR-driven benchmark variant.
- **`staging_*`** — the host-memory sub-region the streamer stages data through; must not
  overlap the SQ/CQ/PRP regions, and bounds the per-transfer size in v1 (see §5).

---

## 2. Top-level interface

The generated `litenvme_core` module exposes named ports (Gen3 x4, `data_width=256`):

### PCIe / clock / status
| Port | Dir | Width | Notes |
|------|-----|-------|-------|
| `pcie_clk_p/n`                | in  | 1   | 100 MHz PCIe reference clock (to the GTY) |
| `pcie_rx_p/n`, `pcie_tx_p/n`  | in/out | 4 | the PCIe lanes |
| `rst_n`                       | out | 1   | PERST# driven to the downstream SSD (100 ms one-shot) |
| `clk`, `rst`                  | out | 1   | sys clock/reset (internal-clk mode); inputs if `clk_external` |
| `serial_tx`, `serial_rx`      | out/in | 1 | bring-up CPU UART (`uart: serial`) |
| `status_init_done`            | out | 1   | **1 once NVMe bring-up completed** — gate your logic on this |
| `status_init_error`           | out | 1   | 1 if bring-up failed |

### Block streaming interface
| Port | Dir | Width | Notes |
|------|-----|-------|-------|
| `block_ctrl_start`            | in  | 1   | pulse to begin a transfer (internally gated on `init_done`) |
| `block_ctrl_write`            | in  | 1   | 1 = write (host→SSD), 0 = read (SSD→host) |
| `block_ctrl_sector`           | in  | 64  | start LBA |
| `block_ctrl_count`            | in  | 32  | number of 512-byte sectors |
| `block_ctrl_nsid`             | in  | 32  | namespace id (1) |
| `block_ctrl_done`             | out | 1   | 1 at idle / on completion (registered) |
| `block_ctrl_busy`             | out | 1   | transfer in progress |
| `block_ctrl_error`            | out | 1   | a completion returned an NVMe error status |
| `block_wr_axis_t{valid,ready,last,data[255:0]}` | mixed | — | **write payload in** (AXI-Stream) |
| `block_rd_axis_t{valid,ready,last,data[255:0]}` | mixed | — | **read payload out** (AXI-Stream) |

All block ports run in the `clk` (sys) domain.

> **CPU-less variant** (`cpu: None`): the core omits the soft-CPU and instead exposes a control
> AXI-Lite/Wishbone bus so an external master performs the NVMe bring-up over CSRs (the LiteDRAM
> "no CPU" pattern). The embedded-CPU variant is the default and the simplest to integrate.

---

## 3. Usage protocol

```
1. Power-up: the embedded CPU brings up the SSD; wait for status_init_done == 1.
2. Write N sectors at LBA L:
     block_ctrl_write  = 1
     block_ctrl_sector = L
     block_ctrl_count  = N
     pulse block_ctrl_start
     stream N*512 bytes on block_wr_axis  (tvalid/tready, tlast on the final beat)
     wait block_ctrl_done; check block_ctrl_error == 0
3. Read N sectors at LBA L:
     block_ctrl_write  = 0 ; sector = L ; count = N ; pulse start
     consume N*512 bytes on block_rd_axis (tlast marks the final beat)
     wait block_ctrl_done; check block_ctrl_error == 0
```

Notes:
- `block_ctrl_start` is internally ANDed with `init_done`, so a premature start is ignored.
- 1 sector = 512 bytes; the 256-bit data bus carries 32 bytes/beat → `count*16` beats per transfer.
- `done` is registered (1 at idle, cleared within a cycle of `start`), so it is safe to poll.
- One transfer at a time; assert `block_ctrl_done` before issuing the next.

---

## 4. Integration in a traditional FPGA flow

The standalone core is a plain Verilog module — instantiate it like any IP:

1. **Generate** `litenvme_core.v` + headers (§1) and copy them into your project.

2. **Provide the PCIe hard IP.** The core instantiates the Xilinx `pcie4_uscale_plus` block by
   name. The emitted `litenvme_core.tcl` contains the commands that generate this IP (for the
   configured device/quad/location) — either source it in Vivado, or replicate it in your own
   flow by generating a `pcie4_uscale_plus` IP with the same parameters and adding the
   generated `.v` alongside it. (LitePCIe is the only generation-time dependency; the
   integrator does not need Migen/LiteX to *use* the core.)

3. **Instantiate** the module and wire the top ports:
   ```verilog
   litenvme_core u_nvme (
       .pcie_clk_p(pcie_refclk_p), .pcie_clk_n(pcie_refclk_n),
       .pcie_rx_p(pcie_rx_p), .pcie_rx_n(pcie_rx_n),
       .pcie_tx_p(pcie_tx_p), .pcie_tx_n(pcie_tx_n),
       .rst_n(ssd_perst_n),                 // drive PERST# to the SSD/slot
       .clk(nvme_clk), .rst(nvme_rst),      // internal-clk mode: outputs to your logic
       .status_init_done(nvme_ready), .status_init_error(nvme_err),
       // block interface, all synchronous to nvme_clk:
       .block_ctrl_write(...), .block_ctrl_sector(...), .block_ctrl_count(...),
       .block_ctrl_start(...), .block_ctrl_done(...), .block_ctrl_error(...),
       .block_wr_axis_tvalid(...), .block_wr_axis_tready(...),
       .block_wr_axis_tlast(...),  .block_wr_axis_tdata(...),
       .block_rd_axis_tvalid(...), .block_rd_axis_tready(...),
       .block_rd_axis_tlast(...),  .block_rd_axis_tdata(...),
       .serial_tx(uart_tx), .serial_rx(uart_rx)
   );
   ```

4. **Clocking.** In the default *internal-clk* mode the core derives `clk`/`rst` from the PCIe
   user clock and drives them **out** — run your block-interface logic on that `clk`. Set
   `clk_external: True` to instead provide your own `clk`/`rst` (synchronized to PCIe internally).

5. **Constraints.** Assign the GTY/refclk and lane pins for your board, and the `pcie_refclk`
   period (100 MHz). The PCIe IP supplies its own internal timing constraints.

6. **Firmware.** With `firmware: auto` the bring-up firmware is baked into the core's ROM; keep
   the emitted `litenvme_core_rom.init` next to the `.v` (the ROM loads it via `$readmemh`).
   Nothing to load at runtime — `init_done` rises automatically after link-up + NVMe init. (The
   `serial` UART is optional, for the debug console.)

7. **Build** with your normal synth/impl flow. The core meets timing at Gen3 x4 / 125 MHz on the
   reference KU3P.

---

## 5. Performance & resources

### Throughput

Measured on an Alibaba KU3P (XCKU3P) with a Crucial CT500P310SSD8, Gen3 x4, 256-bit @ 125 MHz:

| Workload | Result |
|----------|--------|
| 8 KiB reads (QD32)  | **~2.69 GB/s**, errors = 0 |
| 8 KiB writes (QD32) | **~2.74 GB/s**, errors = 0 |
| 4 KiB reads         | ~1.9–2.0 GB/s |
| Data integrity      | write → read-back bit-exact (BIST + isolated scrub) |

Design notes that bound throughput (see `doc/NVME_PERFORMANCE.md` for the full study):
- The datapath ceiling is `data_width × clk` = 256 b × 125 MHz = **4.0 GB/s**; the usable Gen3 x4
  payload ceiling is ~3.4 GB/s.
- **MaxPayloadSize is SSD-capped at 512 B**, so read data arrives as 512-byte TLPs; this — not
  the core — is the read-throughput limiter. 8 KiB is the transfer-size sweet spot (~+22 % over 4 KiB).
- **Transfer size (v1):** one NVMe command per `start`, so `count × 512 ≤ staging_size`. The
  engine builds the PRP list internally for transfers spanning >2 pages, so larger transfers
  just need a larger `staging_size` (or DDR-backed host memory). Multi-chunk streaming
  (transfers larger than the staging region) is a planned follow-up.
- **Queue depth** is set by `qd` (default 32); the engine keeps that many commands outstanding
  with no CPU in the loop (IOPS ≈ qd / latency).

To enlarge the host-memory window / staging beyond the on-FPGA BRAM (for very large transfers),
provide a DDR-backed `hostmem_backend` (e.g. a LiteDRAM AXI port) — the responder accepts any
backend exposing a matching `.axi` slave.

### Resource utilization

Out-of-context synthesis of the **standalone `litenvme_core`** (this config:
`examples/alibaba_xcku3p.yml`, Gen3 x4 / 256-bit, 256 KiB window + bring-up CPU) on the XCKU3P
(`xcku3p-ffvb676-2-e`):

| Resource | Used | Device | % |
|----------|-----:|-------:|--:|
| CLB LUTs       | 11,577 | 162,720 | 7.1 % |
| CLB Registers  | 14,408 | 325,440 | 4.4 % |
| Block RAM tiles| 133    | 360     | 37 % |
| URAM           | 0      | 48      | 0 % |
| DSP            | 0      | 1,368   | 0 % |

Per-block breakdown (Vivado hierarchy):

| Block | LUT | FF | RAMB36 |
|-------|----:|---:|-------:|
| PCIe hard-IP wrapper (`pcie4_uscale_plus` + GTY) | 2,960 | 6,303 | 22 |
| VexRiscv bring-up CPU (minimal)                  | 1,695 |   749 | 0 (+2 RAMB18) — ROM/RAM ≈ 34 |
| NVMe datapath + block streamer + 256 KiB window + glue | ~6,922 | ~7,356 | ~106 (window ≈ 64) |

Notes (post-synthesis estimate; post-place is slightly lower):
- **BRAM is the dominant, tunable cost** and is split between the host-memory window (~64 RAMB36
  for 256 KiB) and the CPU ROM/RAM (~34, this config boots the LiteX BIOS: 24 KiB ROM + 64 KiB
  main_ram). Shrink `hostmem_size`, or point `hostmem_backend` at DDR/LiteDRAM, to cut the window.
- **CPU footprint** — the bring-up CPU is one-time-only, so it is already the smallest VexRiscv
  (`cpu_variant: minimal`, ~1.7 k LUT) and never needs to be fast. To shrink its ROM/RAM:
  - `firmware: minimal` bakes a **code-only firmware** (auto-init + idle, no console, no prints)
    into a small ROM with **no main_ram** — ~8.6 KiB vs the 24 KiB BIOS, saving ~20 BRAM tiles
    (→ core ≈ 113). *Status:* the minimal/auto firmware builds, but self-init on the **generated**
    core still needs the firmware↔core CSR names aligned (the wrapper nests cfg/mmio as
    `nvme_cfg`/`nvme_mmio` while the firmware uses `cfg_cfg_*`/`mmio_mem_*`) — a tracked follow-up.
  - `cpu: None` removes the CPU and its ROM/RAM entirely; bring-up then comes from an external
    host over the control bus.
  - **RTL init** (`litenvme/init.py`, `--with-rtl-init`, `rtl-init` branch) does the whole
    bring-up *in hardware* — no CPU at all. Measured on the bench SoC, swapping VexRiscv+firmware
    for the sequencer saves **−2,560 LUT / −3,116 FF / −25 BRAM** (the CPU's ROM/RAM); the
    sequencer FSM itself is small and uses 0 BRAM. So the RTL-init core is strictly smaller and
    drops the CPU's ~25 BRAM (a CPU-based 133-tile standalone core → ~108 tiles; the exact figure
    needs wiring RTL init into the generator + an OOC synth).

**SSD flexibility — firmware vs RTL init.** The firmware bring-up is the flexible one: it *reads*
the device's actual values (doorbell stride from `CAP.DSTRD`, the BAR0 64/32-bit type, namespace
via Identify) and retries, so it copes with arbitrary NVMe SSDs and quirks. The RTL sequencer
trades that for area: it currently **assumes** the common case — SSD at BDF 0:1:0, a 64-bit BAR0,
`DSTRD=0` (4-byte doorbell stride), `NSID=1`, fixed queue sizes — which holds for the large
majority of NVMe SSDs but is not universal. (It already does the `CAP` read, so deriving `DSTRD`
and the BAR type from it to widen SSD coverage is a small extension.) This is exactly why the two
paths coexist: **use firmware for development and broad SSD support; use RTL init for a known,
controlled, CPU-less deployment** where minimum area matters.
- **Two earlier BRAM wastes were removed** to get here (from 325 → 133 tiles): the CSR memory-
  debug frontend was replicating the whole window (a third BRAM port) and is now off by default
  for the pin-driven core; and the window was right-sized to 256 KiB (the 512 KiB default carried
  256 KiB of request-generator buffers the block streamer doesn't use).
- **0 URAM, 0 DSP** — LUT/FF + BRAM only, so it ports cleanly across Ultrascale+ parts. The PCIe
  hard IP (1 GTY quad) and one QPLL are the only hard resources required.

For reference, the **bench test SoC** (`--with-etherbone --with-block-streamer`, which adds the
Ethernet/Etherbone bridge + request-generator/BIST harness + the 512 KiB window with CSR debug)
is larger end-to-end (post-place: 14,667 LUT / 18,733 FF / 325 BRAM / 0 URAM / 2 DSP, timing met
@125 MHz; HW-revalidated: Gen3 x4, ~2.69 GB/s reads errors=0, block-streamer correctness PASS,
nvme_verify OK).

For how these numbers and the throughput compare to other publicly available NVMe cores
(IntelliProp, Design Gateway, iWave, OpenExpress) and whether LiteNVMe is near-optimal, see
`doc/NVME_CORE_COMPARISON.md`.

---

## 6. Test design (Etherbone)

The board bring-up SoC `bench/alibaba_xcku3p.py --with-io-engine --with-block-streamer` wraps the
same datapath with Etherbone + a CSR-driven test harness so it can be driven from a host PC:

```bash
# build + load the test SoC (bitstream), then:
bench/hw_block.sh                 # load -> boot -> nvme_setup -> host test
# host test (over Etherbone), standalone:
python3 bench/test_block.py --csr-csv build/alibaba_xcku3p/csr.csv            # throughput + correctness
python3 bench/test_block.py --isolated                                       # write/scrub/read-back proof
```

It exercises both engine front-ends behind a 1-bit CSR mux:
- **throughput** via the RTL request generator (HW-measured cycles → MB/s),
- **correctness** via the block streamer + a counter BIST on `wr_sink`/`rd_source` — write a
  pattern, optionally scrub staging with a different LBA, read it back, PASS when 0 mismatches.

This is the harness used to validate the numbers in §5.
