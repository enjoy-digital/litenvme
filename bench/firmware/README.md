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
- `nvme_reset` — clear cached NVMe init state (forces re-init on next command)
- `mmio_warn_writes <0|1>` — enable/disable warnings on MMIO writes (default 0)
- `nvme_debug <0|1>` — enable/disable NVMe debug prints
- `nvme_fill <pattern>` — set write buffer fill pattern (hex)
- `mmio_rd <addr>` / `mmio_wr <addr> <val>` — absolute MMIO access
- `mmio_dump <addr> <len> [s]` — dump MMIO space
- `nvme_identify [bar0] [cid]` — assign BAR0, enable MEM/BME/INTx-off, run Identify
- `nvme_read [bar0] [nsid] [slba] [nlb]` — read NLB blocks into hostmem
- `nvme_read_dump [bar0] [nsid] [slba] [nlb] [dwords]` — read + dump hostmem (cap at 256 dwords)
- `nvme_write_readback [bar0] [nsid] [slba] [nlb] [dwords]` — write then read+dump
- `nvme_verify [bar0] [nsid] [slba] [nlb] [dwords]` — verify pattern on LBA range
- `nvme_write [bar0] [nsid] [slba] [nlb]` — write NLB blocks from hostmem
- `nvme_bench <read|write> [bar0] [nsid] [slba] [nlb] [count] [step]` — repeated I/O benchmark with latency, MB/s and IOPS

Notes:
- The PCIe BDF is fixed in firmware (0:1:0). Update `cfg_bus/cfg_dev/cfg_fun` in `bench/firmware/main.c` if needed.

CSR request flow (firmware-driven, no UART):
- The gateware exposes `nvme_req_*` CSRs (op/nsid/lba/nlb/buf/bar0 + start/status).
- Firmware polls `req_start`, executes the request, then updates `req_status` and `req_cqe_status`.

Performance testing, current scope:
- The current request path is still single-shot and firmware-polled.
- I/O requests are limited to `PRP1` only and `nlb <= 8`.
- This means the first measurements should focus on request latency and effective throughput of the existing path, not peak NVMe bandwidth.

Recommended first measurements:
- Compare `read` vs `write` for `nlb=1,2,4,8`.
- Measure both cold runs (first request after reset/init) and steady-state runs.
- Record host memory DMA counter deltas with `status` before/after a batch.
- Repeat the same request many times to separate firmware/setup overhead from data movement time.

Suggested benchmark loop:
1. Reset the firmware state with `nvme_reset`.
2. Run one request to capture cold-start behavior.
3. Run 100-1000 identical requests through the CSR request path.
4. Capture elapsed host-side time and the `hostmem_wr_count` / `hostmem_rd_count` deltas.
5. Sweep `nlb` from 1 to 8 and compare scaling.

What to add next for better measurements:
- Request instrumentation CSRs such as `req_cycles` and `req_bytes_done` are now exposed by the firmware-owned request block.
- Use `bench/bench_req.py` to automate repeated runs and report MB/s and average latency.
- After that, the next architectural step is a small request FIFO so the CPU is not idle between back-to-back submissions.

Benchmark helper:
```sh
./bench_req.py --op read --nsid 1 --lba 0 --nlb 8 --buf 0x10005000 --bar0 0xe0000000 --count 100 --warmup 1
./bench_req.py --op write --nsid 1 --lba 1024 --nlb 8 --buf 0x10006000 --bar0 0xe0000000 --count 100 --warmup 1
```

The benchmark reports:
- host-side average latency and payload rate
- firmware-side average cycles and derived latency/rate
- hostmem DMA beat deltas when the counters are present

Firmware-side benchmark:
```sh
nvme_bench read  0xe0000000 1 0    8 100 0
nvme_bench write 0xe0000000 1 1024 8 100 8
```

Arguments:
- `read|write` selects the I/O opcode
- `bar0`, `nsid`, `slba`, `nlb` match the existing read/write commands
- `count` is the number of requests to issue
- `step` is the LBA increment between requests (`0` = fixed-LBA, `nlb` = sequential)

The firmware benchmark reports:
- total timer ticks for the batch
- average latency in microseconds
- throughput in MB/s
- IOPS
- payload bytes and hostmem DMA beat deltas

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

Write + Readback flow:
```
nvme_fill 0xa5a5a5a5
nvme_write_readback 0xe0000000 1 1024 1 64
```

Verify flow:
```
nvme_fill 0x12345678
nvme_verify 0xe0000000 1 1024 4 64
```
