# LiteNVMe soft-CPU firmware (VexRiscv)

This folder is a placeholder for firmware sources to be used with the optional
VexRiscv soft-CPU in `bench/usp_target.py`.

The target accepts an integrated ROM init file:

```
./usp_target.py --with-cpu --cpu-firmware path/to/rom.init --build
```

Or use the auto two-pass flow (generate headers, build firmware, integrate ROM):

```
./usp_target.py --with-cpu --cpu-firmware=auto --build --no-compile-gateware
```

For faster iteration, you can boot the LiteX BIOS and load the firmware into RAM:

```
./usp_target.py --with-cpu --cpu-boot=bios --build --no-compile-gateware
litex_term /dev/ttyUSBX --kernel bench/firmware/firmware.bin
```

The firmware linker script is selected by `BOOT`:
- `BOOT=rom` links `.text` into ROM (`linker_rom.ld`).
- `BOOT=bios` links `.text` into main RAM (`linker.ld`).

The ROM init file can be generated from a compiled firmware image using LiteX
tools or other RISC-V toolchains. The firmware is expected to perform the same
setup steps as the host scripts (BAR0 discovery, MEM/BME enable, admin+IO queue
init) and then optionally execute read/write commands.

Console commands (from the firmware prompt):

- `status` — link status + hostmem counters
- `cfg_rd <reg>` / `cfg_wr <reg> <val>` — config space access
- Note: some CFG writes can return `err=1` but still take effect (posted/UR/CA). Always read back to confirm.
- `cmd_enable` / `cmd_disable` — set/clear Command.MEM + Command.BME
- `mmio_rd <addr>` / `mmio_wr <addr> <val>` — absolute MMIO access
- `mmio_dump <addr> <len> [s]` — dump MMIO space
- `nvme_identify [bar0] [cid]` — assign BAR0, enable MEM/BME/INTx-off, run Identify
- `nvme_read [bar0] [nsid] [slba] [nlb]` — read NLB blocks into hostmem
- `nvme_read_dump [bar0] [nsid] [slba] [nlb] [dwords]` — read + dump hostmem (cap at 256 dwords)
- `nvme_write [bar0] [nsid] [slba] [nlb]` — write NLB blocks from hostmem

Notes:
- The PCIe BDF is fixed in firmware (0:1:0). Update `cfg_bus/cfg_dev/cfg_fun` in `bench/firmware/main.c` if needed.

Suggested next step: add a small firmware that:
- polls PCIe link status,
- configures BAR0 and Command.MEM/BME,
- initializes admin queues and IO queues,
- services a simple mailbox for read/write requests.

One-shot Identify flow:
```
nvme_identify 0xe0000000 1
```

One-shot Read flow:
```
nvme_read 0xe0000000 1 0 1
```

One-shot Read+Dump flow:
```
nvme_read_dump 0xe0000000 1 0 1 64
```
