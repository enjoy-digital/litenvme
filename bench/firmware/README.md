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

The ROM init file can be generated from a compiled firmware image using LiteX
tools or other RISC-V toolchains. The firmware is expected to perform the same
setup steps as the host scripts (BAR0 discovery, MEM/BME enable, admin+IO queue
init) and then optionally execute read/write commands.

Suggested next step: add a small firmware that:
- polls PCIe link status,
- configures BAR0 and Command.MEM/BME,
- initializes admin queues and IO queues,
- services a simple mailbox for read/write requests.
