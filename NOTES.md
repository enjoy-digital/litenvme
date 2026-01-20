./litex_acorn_baseboard_mini.py --cpu-type=None --uart-name=uartbone --integrated-main-ram-size=0x100 --csr-csv=csr.csv --with-pcie --build --load
litex_server --uart-port=/dev/ttyUSB2
litescope_cli
./test_ltssm_tracer.py

./kosagi_netv2.py --cpu-type=None --uart-name=uartbone --integrated-main-ram-size=0x100 --csr-csv=csr.csv --with-pcie --build --load