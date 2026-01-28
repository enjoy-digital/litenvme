# LiteNVMe bring-up quickstart

See `doc/NVME_ACCESS.md` for a step-by-step description of Identify/Read/Write flows.
See `bench/firmware/README.md` for the soft-CPU firmware build flow.

## 0) Build + load bitstream
./usp_target.py --csr-csv=csr.csv --build --load

## 1) Start LiteX server
litex_server --uart --uart-port=/dev/ttyUSBX --uart-baudrate=2e6

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
