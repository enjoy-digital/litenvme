./litex_acorn_baseboard_mini.py --cpu-type=None --uart-name=uartbone --integrated-main-ram-size=0x100 --csr-csv=csr.csv --with-pcie --build --load
litex_server --uart-port=/dev/ttyUSB2
litescope_cli
./test_ltssm_tracer.py

./kosagi_netv2.py --cpu-type=None --uart-name=uartbone --integrated-main-ram-size=0x100 --csr-csv=csr.csv --with-pcie --build --load



Inspect config space:
./test_cfg.py --wait-link --dump-all --caps

BAR0 sizing + assignment:
./test_cfg.py --wait-link --bar0-assign --bar0-base 0xe0000000

Enable MEM+BME (and optionally disable legacy INTx):
./test_cfg.py --wait-link --enable-mem --enable-bme --disable-intx

Re-dump to confirm BAR and CMD:
./test_cfg.py --wait-link --dump-all
