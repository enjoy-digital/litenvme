# LiteNVMe bring-up quickstart

See `doc/NVME_ACCESS.md` for a step-by-step description of Identify/Read/Write flows.
See `bench/firmware/README.md` for the soft-CPU firmware build flow.

BIOS load quickstart:
```
./alibaba_xcku3p.py --with-etherbone --with-cpu --cpu-boot=bios --build --no-compile-gateware
litex_term /dev/ttyUSBX --kernel bench/firmware/firmware.bin
```

## 0) Build + load bitstream
./alibaba_xcku3p.py --with-cpu --cpu-boot=bios --csr-csv=csr.csv --with-etherbone --libc=full --build --load

## 1) Start LiteX server
litex_server --udp

## 2) PCIe config / BAR0
# Inspect config space
./test_cfg.py --wait-link --dump-all --caps

# BAR0 sizing + assignment
./test_cfg.py --wait-link --bar0-assign --bar0-base 0xe0000000

# Enable MEM+BME (and disable legacy INTx)
./test_cfg.py --wait-link --enable-mem --enable-bme --disable-intx

# Re-dump to confirm BAR and CMD
./test_cfg.py --wait-link --dump-all

## 3) BAR0 / MMIO sanity
# Read NVMe CAP low dword (offset 0x0000)
./test_bar0.py --addr 0xe0000000 --read

# Dump first 0x100 bytes of BAR0
./test_bar0.py --bar0 0xe0000000 --dump 0x100

## 4) NVMe identify + read/write
# Identify
./test_nvme.py --wait-link --identify \
  --hostmem-base 0x10000000 \
  --asq-addr      0x10000000 \
  --acq-addr      0x10001000 \
  --id-buf        0x10002000 \
  --cid 1

# Read (example)
./test_nvme.py --wait-link --read \
  --q-entries 16 \
  --io-q-entries 4 \
  --nsid 1 \
  --slba 0 \
  --nlb 1

# Write (example)
./test_nvme.py --wait-link --write \
  --q-entries 16 \
  --io-q-entries 4 \
  --nsid 1 \
  --slba 0 \
  --nlb 1

# Write + verify (read-back)
./test_nvme.py --wait-link --write --write-verify \
  --q-entries 16 \
  --io-q-entries 4 \
  --nsid 1 \
  --slba 0 \
  --nlb 1

## 5) Quick I/O helper
`nvme_io.py` is a smaller helper for read/write that can optionally do setup.

# Full setup + read
./nvme_io.py --wait-link --setup --read \
  --q-entries 16 \
  --io-q-entries 4 \
  --nsid 1 \
  --slba 0 \
  --nlb 1

# Read/write without setup (assumes queues already created)
./nvme_io.py --read --write \
  --bar0 0xe0000000 \
  --io-q-entries 4 \
  --nsid 1 \
  --slba 0 \
  --nlb 1

## 6) Throughput measurements
Throughput is measured with the hardware I/O engine driven by the hardware request generator
(no CPU in the steady-state loop). Build with the engine and the console firmware:

```sh
./alibaba_xcku3p.py --with-etherbone --with-cpu --with-io-engine --build --load
```

Firmware-side benchmark (from the `litenvme>` prompt):
```sh
nvme_engine_bench <read|write> [bar0] [nsid] [slba] [nlb] [count] [step]
nvme_engine_bench read  0xe0000000 1 1000000 16 1000 16
nvme_engine_bench write 0xe0000000 1 3000000 16 1000 16
```

- `nlb` is 1..16 (16 = 8 KiB transfers, the default); `count` is the number of requests;
  `step` is the LBA increment between requests.
- It reports the I/O engine and request generator counters, the cycle count and the
  throughput in MB/s, plus the hostmem DMA counters when present.

`hw_gen3.sh` runs the recorded sequence (8 KiB and 4 KiB reads, 8 KiB writes) and keeps the
board-printed output. Recorded runs are in `bench/results/` (see `doc/VALIDATION.md`).
`doc/archive/qd_sweep.py` and its results (`qd_sweep_result.txt`, `qd_sweep_detail.txt`) come
from an earlier firmware with a CPU-driven `nvme_bench` command that the current firmware no
longer has; they are archived there.
