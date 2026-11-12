# LiteNVMe soft-CPU firmware (VexRiscv)

Bring-up and test firmware for the optional VexRiscv soft-CPU of the bench SoC
(`bench/alibaba_xcku3p.py`). The standalone generator (`litenvme_gen`) builds the same sources
when `firmware` is `"auto"` or `"minimal"` (`MINIMAL=1`: auto-init then idle, no console).

The bench SoC builds the firmware and integrates it into ROM by default (two-pass flow:
generate headers, build firmware, integrate ROM):

```
./alibaba_xcku3p.py --with-cpu --cpu-firmware=auto --build
```

It also accepts a pre-built ROM init file:

```
./alibaba_xcku3p.py --with-cpu --cpu-firmware path/to/rom.init --build
```

For faster iteration, boot the LiteX BIOS and load the firmware into RAM:

```
./alibaba_xcku3p.py --with-cpu --cpu-boot=bios --build
litex_term /dev/ttyUSBX --kernel bench/firmware/firmware.bin
```

The firmware linker script is selected by `BOOT`:
- `BOOT=rom` links `.text` into ROM (`linker_rom.ld`).
- `BOOT=bios` links `.text` into main RAM (`linker.ld`).

The firmware performs the same setup steps as the host scripts (BAR0 discovery, MEM/BME enable,
MPS setup, admin and I/O queue creation) and then executes read/write commands.

Console commands (from the firmware prompt, see `help`):

- `status`: PCIe link status and hostmem counters.
- `reboot`: reboot the CPU.
- `cmd_enable` / `cmd_disable`: set/clear Command.MEM + Command.BME.
- `nvme_reset`: clear cached NVMe init state and disable the controller.
- `nvme_fill <pattern>`: set the write buffer fill pattern (hex).
- `nvme_identify [bar0] [cid]`: assign BAR0, enable MEM/BME/INTx off, run Identify.
- `nvme_read [bar0] [nsid] [slba] [nlb]`: read NLB blocks into hostmem.
- `nvme_write [bar0] [nsid] [slba] [nlb]`: write NLB blocks from hostmem.
- `nvme_verify [bar0] [nsid] [slba] [nlb] [dwords]`: write the fill pattern, read back and verify
  (`nlb` <= 8).
- `nvme_engine_bench <read|write> [bar0] [nsid] [slba] [nlb] [count] [step]`: benchmark the
  hardware I/O engine with the request generator (needs `--with-io-engine`, `nlb` 1..16).
- `nvme_setup [bar0]`: one-time NVMe bring-up, engine enable and `init_done` (for
  Etherbone-driven host tests, needs `--with-io-engine`).

Notes:
- The PCIe BDF is fixed in firmware (0:1:0). Update `cfg_bus/cfg_dev/cfg_fun` in `bench/firmware/main.c` if needed.
- Some CFG writes can return `err=1` but still take effect (posted/UR/CA). Always read back to confirm.

One-shot Identify flow:
```
nvme_identify 0xe0000000 1
```

One-shot Read flow:
```
nvme_read 0xe0000000 1 0 1
```

Verify flow:
```
nvme_fill 0x12345678
nvme_verify 0xe0000000 1 1024 4 64
```

Engine throughput (8 KiB reads, see `bench/NOTES.md`):
```
nvme_engine_bench read 0xe0000000 1 1000000 16 1000 16
```
