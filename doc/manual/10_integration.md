# Integration {#integration}

This chapter explains how to add LiteNVMe to your own design. It starts with a LiteX SoC, the native
way: the root port, the core, its registers, the choice of a front-end, the host-memory window,
bring-up and clocks.

It then covers the standalone Verilog core for designs built without LiteX, and what a port to
another PCIe PHY needs. The status of each path is in the [status table](#intro-status).

## At a glance {#integ-glance}

| Item | Value |
|---------------------------|------------------------------------------------------------------|
| Language | Python (Migen, LiteX); Verilog through `litenvme_gen` |
| Core class | `LiteNVMe` (`litenvme/core.py`) |
| Needs | A LitePCIe root port: PHY with `mode="RootPort"` and `with_cfg_mgmt=True` |
| Data interfaces | Block streamer, request generator, or the raw I/O engine streams |
| Bring-up | Firmware on a soft CPU, or the init sequencer in logic |
| Generator PHY | `USPPCIEPHY` (Kintex UltraScale+), Gen3 x4, 256-bit |
| Reference designs | `bench/alibaba_xcku3p.py`, `examples/alibaba_xcku3p.yml` |

## Integration in a LiteX SoC {#integ-litex}

A LiteNVMe design is a chain of parts, shown in the figure below. The PCIe PHY runs the hard IP as a
root port and drives the link to the SSD. The LitePCIe root port gives the core its crossbar, on
which the core takes three master ports and one slave port.

A CPU or Etherbone programs the core through registers, and user logic moves the data through a
front-end.

![LiteNVMe in a LiteX SoC. The PHY and the LitePCIe root port carry the TLPs between the SSD and
the core (cyan arrow); the PERST# timer releases the SSD after configuration. Registers come from
a CPU or from Etherbone; the data of user logic goes through a front-end of the
core.](integ_soc.svg)

The LitePCIe manual describes the root port mode itself, in the section *Root port designs* of
its [Integration chapter](https://github.com/enjoy-digital/litepcie/blob/master/doc/manual/12_integration.md).
The example below is the PCIe and NVMe part of a SoC for the Alibaba XCKU3P, with the settings of
the bench SoC and of the example configuration. The clock generator, `_CRG`, stands for the one of
your board.

```python
from litex.gen.genlib.misc import WaitTimer
from litex.soc.integration.soc_core import SoCCore

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.core.rootport  import LitePCIeRootPort

from litenvme.cfg  import LiteNVMeRootCfgMgmt
from litenvme.core import LiteNVMe, LiteNVMeCoreControl

class MySoC(SoCCore):
    def __init__(self, platform, sys_clk_freq=125e6, **kwargs):
        self.crg = _CRG(platform, sys_clk_freq)
        SoCCore.__init__(self, platform, sys_clk_freq, **kwargs)

        # PHY: the hard IP as a root port, with its local configuration port.
        pcie_pads = platform.request("pcie_x4")
        perst_n   = pcie_pads.rst_n
        pcie_pads.rst_n = Signal() # Driven by the PERST# timer below.
        self.pcie_phy = USPPCIEPHY(platform, pcie_pads,
            speed         = "gen3",
            data_width    = 256,
            ip_name       = "pcie4_uscale_plus",
            mode          = "RootPort",
            with_cfg_mgmt = True)
        self.pcie_endpoint = LitePCIeRootPort(self.pcie_phy,
            endianness         = self.pcie_phy.endianness,
            address_width      = 64,
            with_configuration = True)
        self.pcie_rootcfg = LiteNVMeRootCfgMgmt(self.pcie_phy)
        platform.add_false_path_constraints(self.crg.cd_sys.clk,
            self.pcie_phy.cd_pcie.clk)

        # PERST#: hold the SSD in reset for 100 ms, then release it.
        self.perst_timer = WaitTimer(int(100e-3*sys_clk_freq))
        self.comb += [
            self.perst_timer.wait.eq(1),
            perst_n.eq(self.perst_timer.done),
            pcie_pads.rst_n.eq(self.perst_timer.done),
        ]

        # NVMe host core, with the block streamer as its front-end.
        self.nvme = LiteNVMe(self.pcie_endpoint,
            hostmem_base        = 0x1000_0000,
            hostmem_size        = 0x4_0000,
            qsize               = 8,
            qd                  = 4,
            with_block_streamer = True,
            staging_base        = 0x1_0000,
            staging_size        = 0x3_0000)
        self.nvme_ctrl = LiteNVMeCoreControl()
```

The bench SoC also sets the quad, the PLL and the clock frequencies of the hard IP with
`update_config()`; the example configuration lists the same values under `phy_config`. This
snippet elaborates as shown with the LiteX, LitePCIe and LiteX-Boards versions this manual
describes.

### Parts of the snippet {#integ-parts}

The PHY needs two settings beyond an endpoint design. `mode="RootPort"` configures the hard IP as
a root port. `with_cfg_mgmt=True` exposes its local configuration port, through which the root
configuration accessor (`LiteNVMeRootCfgMgmt`) programs the MPS and the memory window of the root
port. `LitePCIeRootPort` with `with_configuration=True` lets the configuration accessor send
CfgRd0 and CfgWr0 requests to the SSD.

The PERST# pin is driven by the design, not by the slot: the pad of the platform is detached from
the PHY and both are driven by a 100 ms timer. The SSD leaves reset when the timer ends, and the
link then trains. A design that also resets the SSD later must drive the same two signals.

`LiteNVMe` takes the root port and builds the accessors, the host-memory window and the I/O
engine on its crossbar; [Architecture](#architecture) describes them. Its main parameters are
below; [Build options](#build-options) lists all of them.

| Parameter | Default | Role |
|------------------------------|-----------------|-------------------------------------|
| `hostmem_base` | `0x10000000` | Bus address of the host-memory window |
| `hostmem_size` | `0x80000` | Window size: 512 KiB of block RAM by default |
| `data_width` | PHY width | 64, 128 or 256 bits |
| `qid`, `qsize`, `qd` | 1, 64, 32 | I/O queue number, entries, commands in flight |
| `with_block_streamer` | False | Add the block streamer front-end |
| `with_request_gen` | False | Add the request generator front-end |
| `staging_base` | `0x40000` | Staging region, offset in the window |
| `staging_size` | `0x40000` | Staging region size: the largest transfer |
| `hostmem_backend` | None | Another AXI memory in place of the block RAM |

`qd` must be at most `qsize - 1`: one ring entry always stays empty. `LiteNVMeCoreControl` adds
the two registers `init_done` and `init_error`, which the firmware sets at the end of bring-up.

### Registers {#integ-registers}

LiteX collects the registers of `self.nvme` into one bank named after the attribute, `nvme`. The
names follow the hierarchy of the core: `nvme_cfg_cfg_*` for the configuration accessor,
`nvme_mmio_mem_*` for the MMIO accessor, `nvme_io_engine_engine_*`, `nvme_hostmem_csr_*` and
`nvme_block_streamer_*`. [Register reference](#registers) lists them.

`add_csrs(soc)` registers each block of the core again as a bank of its own (`nvme_cfg`,
`nvme_mmio`, `nvme_engine`, `nvme_hostmem`, `nvme_block`, `nvme_gen`). The same registers then exist
under two names, so the snippet does not call it, and neither does the generator.

`with_csr=False` removes the registers of the accessors, of the engine and of the request generator,
for a design that drives them from logic, as the init sequencer does.

### Choosing a front-end {#integ-frontend}

The I/O engine takes read and write requests on a stream and returns completions on another.
One front-end owns these two streams; [Front-ends](#frontends) describes them. The choice is made
with two parameters.

| `with_block_streamer` | `with_request_gen` | Engine streams |
|------------------------------|--------------------------|--------------------------------|
| True | False | Owned by the block streamer: `wr_sink`, `rd_source` |
| False | True | Owned by the request generator, for benchmarks |
| False | False | Exported as `nvme.request` and `nvme.completion` |
| True | True | Exported too: the SoC selects the front-end itself |

The block streamer is the simple choice: user logic gives a sector number and a count, then streams
the data, and never sees a PRP or an address in the window. Custom logic that drives `nvme.request`
and `nvme.completion` gets the full queue depth, but places its own buffers in the window.

With `--with-block-streamer`, the bench SoC builds both front-ends and selects one with a register.

### Sizing the window {#integ-window}

The host-memory window is the memory the SSD reads and writes: the admin and I/O rings, the PRP
lists and the data buffers. It is block RAM by default, so its size is the main block RAM cost of
the core.

The layout inside it is set by the code that brings the SSD up, not by `LiteNVMe`;
[The host-memory window](#host-memory) gives the layouts of the firmware and of the init
sequencer.

Three rules follow from these layouts:

- The window holds the rings and buffers of bring-up below `0x10000`, at fixed offsets.
- The staging region of the block streamer must not overlap them, nor the PRP-list region of the
  I/O engine. The PRP-list region takes one 4 KiB page per I/O queue entry.
- A transfer of the block streamer must fit in the staging region: `count * 512` bytes at most.
  The streamer does not check this.

The generator checks the second rule. The example configuration places the PRP-list region at
`0x7000` with `prp_list_offset`, between the queues and the staging region from `0x10000`, for
both bring-up modes. A configuration whose PRP-list region cannot be placed inside the window,
above the queues and clear of the staging region is refused.

A SoC that builds the core itself, as the bench SoC does, has no such check: there, keep transfers
of the block streamer to 8 KiB, or place the PRP-list region clear of the staging region.

`hostmem_backend` replaces the block RAM by another AXI memory, such as a LiteDRAM port, for a
larger window. It is not used on hardware in the repository.

### Bring-up and payload size {#integ-bringup}

Before the first command, the SSD must be enabled, the controller reset and enabled, and the I/O
queues created; [Bring-up](#bringup) describes the sequence. Two implementations exist:

- **Firmware** (`bench/firmware`). It raises the MPS of both ends, reads the doorbell stride
  from CAP and programs the I/O engine; Identify is a command of its own. It is written for the
  register names of the bench SoC, which builds the blocks one by one (`cfg_cfg_*`,
  `mmio_mem_*`, `nvme_engine_engine_*`), and maps them to the `nvme_*` names of a SoC that names
  its `LiteNVMe` instance `nvme`, as the generator does. It needs the debug registers of the
  window (`with_hostmem_csr`).
- **Init sequencer** (`LiteNVMeInitSequencer`, `litenvme/init.py`). It needs no CPU and drives
  the accessors and the engine directly, as `litenvme/gen.py` and the bench SoC show. It assumes
  the SSD at bus 0, device 1, function 0, a 64-bit BAR0 and a doorbell stride of 4 bytes.

The PCIe MPS of the link stays at 128 bytes until both ends are programmed. The firmware raises it
to the largest size both ends support, 512 bytes on the bench. The init sequencer does not, so a
design that uses it runs with 128-byte payloads. Reads then arrive in four times more TLPs.

### Clocks and constraints {#integ-clocks}

The core runs in the system clock domain, `sys`. The PHY adds the `pcie` domain, the user clock
of the hard IP, and crosses between the two with asynchronous FIFOs. The root configuration
accessor also crosses with a handshake, and relies on a false path between the two domains, which
the snippet declares.

The reference designs run the datapath at 256 bits and 125 MHz, 4 GB/s, which is above the payload
rate of a Gen3 x4 link.

The bench SoC makes its 125 MHz system clock with an MMCM from the 100 MHz oscillator of the board.
The hard IP runs its user clock at 125 MHz (`axisten_freq`) and its core clock at 250 MHz
(`coreclk_freq`).

## Standalone core {#integ-standalone}

The generator, `litenvme_gen`, turns a YAML configuration into one Verilog module. It holds the
PHY and the LitePCIe root port, the PERST# timer, the root configuration accessor, LiteNVMe with
the block streamer, and the bring-up: a VexRiscv CPU or the init sequencer. It targets
`USPPCIEPHY` only; another PHY name is refused. [Getting started](#gs-generate) runs it.

```sh
litenvme_gen <config> --output-dir build --name litenvme_core
```

The run writes the files of the table below. It never calls Vivado, even though the TCL script
it writes would build the module as a top-level design.

| File | Content |
|-----------------------------------------|----------------------------------------------------|
| `gateware/litenvme_core.v` | The module, named after `--name` |
| `gateware/litenvme_core.tcl` | Vivado script; creates the hard IP, `pcie_usp` |
| `gateware/litenvme_core.xdc` | Clock and reset constraints |
| `gateware/*.init` | Contents of the ROM and RAMs of the SoC |
| `csr.csv`, `csr.json` | Register map |
| `software/include/generated/` | C headers, for the firmware of a CPU |

### Ports {#integ-ports}

The figure below shows the port groups of the generated module.

![Port groups of the standalone core. The PCIe pins and PERST# go to the slot; the clock, the
status pins and the block interface go to user logic. `serial_tx/rx` exist only with a CPU and
`uart: serial`.](integ_gen.svg)

| Port | Direction | Width | Role |
|----------------------------|------------|--------|----------------------------------------|
| `pcie_clk_p/n` | in | 1 | 100 MHz PCIe reference clock |
| `pcie_rx_p/n`, `pcie_tx_p/n` | in, out | lanes | PCIe lanes |
| `rst_n` | out | 1 | PERST# of the SSD |
| `clk`, `rst` | out or in | 1 | System clock and reset |
| `status_init_done` | out | 1 | 1 once bring-up has completed |
| `status_init_error` | out | 1 | 1 if bring-up has failed |
| `block_ctrl_start` | in | 1 | Start a transfer, ignored before `init_done` |
| `block_ctrl_write` | in | 1 | 1: write to the SSD, 0: read |
| `block_ctrl_sector` | in | 64 | First sector |
| `block_ctrl_count` | in | 32 | Number of sectors |
| `block_ctrl_nsid` | in | 32 | Namespace, 1 on usual SSDs |
| `block_ctrl_done` | out | 1 | 1 when idle, 0 during a transfer |
| `block_ctrl_busy` | out | 1 | Transfer in progress |
| `block_ctrl_error` | out | 1 | The last CQE had an error status |
| `block_wr_axis_t*` | in | data | Write data: `valid`, `ready`, `last`, `data` |
| `block_rd_axis_t*` | out | data | Read data: `valid`, `ready`, `last`, `data` |

All the block ports are synchronous to `clk`.

With `clk_external: False`, the default, the core drives `clk` and `rst` out from the PCIe user
clock, and user logic runs on them. With `clk_external: True`, user logic drives them in. The
generator then declares a false path between this clock and the PCIe user clock in the
constraints of the core, as the bench SoC does.

### Using the block interface {#integ-protocol}

The figure below shows one write and one read, as user logic sees them. A transfer moves
`count * 512` bytes, so `count * 16` beats on the 256-bit bus.

![One write then one read on the block interface. Each transfer starts with a pulse on
block_ctrl_start; the data goes through the staging region of the window, and done rises after
the completion (write) or after the last read beat (read).](integ_protocol.svg)

1. Wait for `status_init_done` to be 1.
2. Set `block_ctrl_write`, `block_ctrl_sector`, `block_ctrl_count` and `block_ctrl_nsid` (1 on
   usual SSDs; 0 is not a valid namespace), then pulse `block_ctrl_start` for one cycle.
3. For a write, stream the data on `block_wr_axis`. The core counts the beats from `count` and
   does not use `tlast`.
4. For a read, take the data from `block_rd_axis`; `tlast` marks the last beat.
5. Wait for `block_ctrl_done`, then check `block_ctrl_error`.

`done` is registered: it falls in the cycle after `start` and rises when the transfer ends, so it
is safe to poll. One transfer runs at a time, as one NVMe command. The core does not check `count`
against the staging region and passes only its 16 low bits to the command; a count of 0 is not
supported.

### Bring-up modes {#integ-modes}

`init_mode` selects how the standalone core brings the SSD up, and `firmware` what a CPU runs.

| `init_mode` | `firmware` | Bring-up |
|-------------|-------------|------------------------------------------------------------|
| `rtl` | `none` | Init sequencer, started when PERST# is released; no CPU |
| `firmware` | `none` | CPU boots the LiteX BIOS; firmware loaded by hand |
| `firmware` | `auto` | Firmware of `bench/firmware` in the ROM of the CPU |
| `firmware` | `minimal` | Same firmware without a console, in a smaller ROM: bring-up at boot |

With `auto` and `minimal`, the generator builds twice: once for the headers, once with the
firmware compiled by `make -C bench/firmware BOOT=rom`, which needs a RISC-V toolchain. The
firmware takes the register names of the generated core and the layout of its window from these
headers ([Firmware](#boot-firmware)).

The example configuration uses `firmware: minimal`: the core runs the bring-up at boot and
raises `status_init_done` by itself. With `auto`, the bring-up waits for the `nvme_setup` command
on the UART. With `none`, nothing sets `init_done`, and `block_ctrl_start` stays ignored until a
firmware loaded through the UART does. The `init_mode: rtl` core needs no firmware: the init
sequencer starts when PERST# is released. None of these modes has a recorded run on a board
([status](#intro-status)).

In firmware mode the generator keeps the debug registers of the window, which the firmware uses
to write the admin commands and read their completions, and accepts I/O queue 1 only.

[Build options](#build-options) lists the keys of the YAML file. The required keys are `phy` and
`phy_device`; the generator refuses unknown keys, a CPU-less firmware mode, both front-ends
at once, a `qsize` outside 2 to 64 and a PRP-list region that does not fit.

### In a non-LiteX flow {#integ-flow}

1. Generate the core and copy `litenvme_core.v` and its `.init` files into your project.
2. Create the hard IP with the commands of `litenvme_core.tcl`: the `create_ip` of
   `pcie4_uscale_plus` named `pcie_usp`, with its configuration. Do not run the whole script,
   which builds the core as a top-level design.
3. Instantiate `litenvme_core`, and assign the reference clock, the lanes and PERST# to the pins
   of your board.
4. Constrain the reference clock, 100 MHz; the generated `.xdc` holds the constraints of the
   core itself.

## Porting to another PHY {#integ-porting}

The core itself uses only the crossbar of LitePCIe and the local configuration port of the PHY.
Any LitePCIe PHY with a root port mode and `with_cfg_mgmt` can therefore carry it in a LiteX SoC;
the 7-Series PHY, `S7PCIEPHY`, has both.

The host-memory window needs a data width of 64, 128 or 256 bits. It fills the byte count and the
lower address of its completions, which every LitePCIe PHY sends as given.

The generator is limited to `USPPCIEPHY`: `_resolve_platform()` in `litenvme/gen.py` maps the
`phy` key to a PHY class and a platform. A new PHY needs an entry there, with its platform and its
configuration keys. The generator rejects a data width other than 64, 128 or 256 bits when it
checks the configuration.

## Where to go next {#integ-next}

[Bench, firmware and host tools](#examples) describes the reference design that uses all these
parts. [Debugging](#debugging) helps when the SSD does not come up.
