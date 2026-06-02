#!/usr/bin/env bash
# Pure-RTL NVMe bring-up test (CPU-less): load the --with-rtl-init bitstream, then drive the RTL
# init sequencer + a read entirely over Etherbone (no firmware). The bitstream must have been
# built with --with-rtl-init. -> /tmp/rtlinit.log
LOG=/tmp/rtlinit.log; : > "$LOG"
source "$(dirname "$0")/hw_common.sh"
SOC_ARGS="--with-etherbone --with-io-engine --with-rtl-init"

hw_load            || { echo RTLINIT_DONE >>"$LOG"; exit 2; }
hw_serve
hw_link_decode

say "etherbone rtl-init test:"
python3 test_rtlinit.py --csr-csv "$CSR" >>"$LOG" 2>&1

say "DONE"; hw_stop; echo "RTLINIT_DONE" >> "$LOG"
