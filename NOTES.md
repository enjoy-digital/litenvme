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
