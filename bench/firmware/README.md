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
- `bdf <b> <d> <f>` — set target BDF for config access
- `cfg_rd <reg>` / `cfg_wr <reg> <val>` — config space access
- `cmd_enable` / `cmd_disable` — set/clear Command.MEM + Command.BME
- `bar0 <addr>` — set BAR0 base
- `bar0_rd <off>` / `bar0_wr <off> <val>` — BAR0 MMIO access
- `bar0_dump <len> [s]` — dump BAR0 space
- `mmio_rd <addr>` / `mmio_wr <addr> <val>` — absolute MMIO access
- `mmio_dump <addr> <len> [s]` — dump MMIO space
- `bar0_info` — read CAP/VS/CSTS at BAR0

Suggested next step: add a small firmware that:
- polls PCIe link status,
- configures BAR0 and Command.MEM/BME,
- initializes admin queues and IO queues,
- services a simple mailbox for read/write requests.
