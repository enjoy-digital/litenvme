./usp_target.py --csr-csv=csr.csv --build --load
litex_server --uart --uart-port=/dev/ttyUSB0
litescope_cli

# Inspect config space:
./test_cfg.py --wait-link --dump-all --caps

# BAR0 sizing + assignment:
./test_cfg.py --wait-link --bar0-assign --bar0-base 0xe0000000

# Enable MEM+BME (and optionally disable legacy INTx):
./test_cfg.py --wait-link --enable-mem --enable-bme --disable-intx

# Re-dump to confirm BAR and CMD:
./test_cfg.py --wait-link --dump-all

# Read NVMe CAP low dword (offset 0x0000)
./test_bar0.py --addr 0xe0000000 --read

# Dump first 0x100 bytes of BAR0
./test_bar0.py --bar0 0xe0000000 --dump 0x100

# Write to NVMe CC register (offset 0x14) for example:
./test_bar0.py --addr 0xe0000014 --write 0x00000000


./test_cfg.py --dump-all
./test_cfg.py --wait-link --enable-mem --enable-bme --disable-intx
./test_cfg.py --wait-link --bar0-assign --bar0-base 0xe0000000
./test_cfg.py --dump-all
litescope_cli -r usppciephy_req_sink_valid
./test_bar0.py --addr 0xe0000000 --read


# Basic decode of NVMe controller regs:
./test_nvme.py --wait-link --info

# Dump first 0x100 bytes:
./test_nvme.py --dump 0x100

# Disable controller (clear CC.EN, wait RDY=0):
./test_nvme.py --info --disable

# Show what is needed next (queues/admin cmds):
./test_nvme.py --next-steps


# ---------------------------------------------------------------------
# 0) Usual bring-up sequence (one-time)
# ---------------------------------------------------------------------
./usp_target.py --csr-csv=csr.csv --build --load
litex_server --uart --uart-port=/dev/ttyUSB0

# BAR0 sizing/assignment + enable MEM/BME (using your existing scripts)
./test_cfg.py --wait-link --bar0-assign --bar0-base 0xe0000000
./test_cfg.py --wait-link --enable-mem --enable-bme --disable-intx


# ---------------------------------------------------------------------
# 1) Basic NVMe info (CAP/VS/CC/CSTS/AQA/ASQ/ACQ)
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --info


# ---------------------------------------------------------------------
# 2) Default “useful” run (if your script defaults to info)
# ---------------------------------------------------------------------
./test_nvme.py --wait-link


# ---------------------------------------------------------------------
# 3) MMIO checks that still don’t need DMA
# (INTMS/INTMC, AQA/ASQ/ACQ write/readback while CC.EN=0)
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --mmio-check


# ---------------------------------------------------------------------
# 4) Dump first N bytes of BAR0 register space
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --dump 0x100
./test_nvme.py --wait-link --dump 0x400


# ---------------------------------------------------------------------
# 5) Doorbell read sanity (read-only, does not ring)
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --doorbells
./test_nvme.py --wait-link --doorbells --max-q 16


# ---------------------------------------------------------------------
# 6) Force-disable controller (CC.EN=0) and wait CSTS.RDY=0
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --disable


# ---------------------------------------------------------------------
# 7) Override BAR0 manually (skip BAR discovery from config space)
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --bar0 0xe0000000 --info --mmio-check


# ---------------------------------------------------------------------
# 8) Print “next steps” (queues/DMA required)
# ---------------------------------------------------------------------
./test_nvme.py --next-steps


# ---------------------------------------------------------------------
# 9) Tune timeouts
# ---------------------------------------------------------------------
./test_nvme.py --wait-link --timeout-ms 500 --cfg-timeout-ms 200 --mmio-check




# Build/load bitstream with responder + analyzer
./usp_target.py --build --load

# Server
litex_server --uart --uart-port=/dev/ttyUSB0

# Bring-up
./test_cfg.py  --wait-link --enable-mem --enable-bme --disable-intx
./test_cfg.py  --wait-link --bar0-assign --bar0-base 0xe0000000

# Arm LiteScope (your usual flow), then:
./test_nvme.py --wait-link --info
./test_nvme.py --wait-link --identify \
  --hostmem-base 0x10000000 \
  --asq-addr      0x10000000 \
  --acq-addr      0x10001000 \
  --id-buf        0x10002000 \
  --cid 1
