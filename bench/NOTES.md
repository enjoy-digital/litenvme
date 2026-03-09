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

## 6) First performance measurements
Use the current firmware-driven CSR request path as the baseline.

What this measures well:
- end-to-end request latency
- effective throughput for small `PRP1` transfers
- firmware/setup overhead vs host memory DMA activity

Current limits:
- single outstanding request
- polling completion path
- `PRP1` only
- `nlb <= 8`

Recommended sweep:
```sh
./test_req.py --op read  --nsid 1 --lba 0    --nlb 1 --buf 0x10005000 --bar0 0xe0000000
./test_req.py --op read  --nsid 1 --lba 0    --nlb 2 --buf 0x10005000 --bar0 0xe0000000
./test_req.py --op read  --nsid 1 --lba 0    --nlb 4 --buf 0x10005000 --bar0 0xe0000000
./test_req.py --op read  --nsid 1 --lba 0    --nlb 8 --buf 0x10005000 --bar0 0xe0000000
./test_req.py --op write --nsid 1 --lba 1024 --nlb 1 --buf 0x10006000 --bar0 0xe0000000
./test_req.py --op write --nsid 1 --lba 1024 --nlb 2 --buf 0x10006000 --bar0 0xe0000000
./test_req.py --op write --nsid 1 --lba 1024 --nlb 4 --buf 0x10006000 --bar0 0xe0000000
./test_req.py --op write --nsid 1 --lba 1024 --nlb 8 --buf 0x10006000 --bar0 0xe0000000
```

Recommended procedure:
1. Use `status` at the firmware console before and after a batch to capture `hostmem_wr_count` and `hostmem_rd_count`.
2. Separate cold measurements from steady-state measurements.
3. Run each point many times and time the whole batch from the host side.
4. Compute average latency per request and effective MB/s.

Next improvements for performance work:
- use the new `req_cycles` and `req_bytes_done` CSRs
- automate repeated runs with `bench_req.py`
- add a small request FIFO before moving queue management into RTL

Example batch run:
```sh
./bench_req.py --op read  --nsid 1 --lba 0    --nlb 1 --buf 0x10005000 --bar0 0xe0000000 --count 100
./bench_req.py --op read  --nsid 1 --lba 0    --nlb 8 --buf 0x10005000 --bar0 0xe0000000 --count 100
./bench_req.py --op write --nsid 1 --lba 1024 --nlb 1 --buf 0x10006000 --bar0 0xe0000000 --count 100
./bench_req.py --op write --nsid 1 --lba 1024 --nlb 8 --buf 0x10006000 --bar0 0xe0000000 --count 100
```

Firmware-side benchmark:
```sh
nvme_bench read  0xe0000000 1 0    8 100 0 1
nvme_bench write 0xe0000000 1 1024 8 100 8 1
```

This reports:
- average latency in microseconds
- throughput in MB/s
- IOPS
- setup vs I/O timer ticks
- MMIO read/write counts and MMIO timing ticks
- admin/I/O CQ polling loop counts
- payload bytes
- hostmem DMA beat deltas
