#!/usr/bin/env bash
# Single-reader HW session for the NVMe engine bench.
#
# Prevents the concurrent-reader UART corruption: litex_term's crossover2pty thread drains
# uart_xover_rxtx continuously while it lives, so it must FULLY EXIT before engine_console.py
# (the sole reader) runs any command. This script enforces that ordering.
#
# Usage: run_engine_session.sh            # boot fw, then drop to a marker; caller runs cmds
# Prereqs: bitstream already loaded, ONE litex_server --udp running, csr.csv copied to build/.
set -u
cd "$(dirname "$0")"
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
LOG=/tmp/engine_boot.log

# 1) Boot firmware via litex_term, but only long enough to upload + execute, THEN it exits.
rm -f "$LOG"
# Run in foreground with a bounded timeout: litex_term uploads the kernel, prints
# "Executing booted program", and we kill it shortly after so its reader thread is gone.
( timeout 70 litex_term crossover --csr-csv "$CSR" --kernel "$FW" > "$LOG" 2>&1 ) &
TPID=$!

# Wait for the boot banner, then give the upload a moment and stop litex_term.
for i in $(seq 1 60); do
    grep -aq "Executing booted program" "$LOG" 2>/dev/null && break
    sleep 1
done
if ! grep -aq "Executing booted program" "$LOG" 2>/dev/null; then
    echo "BOOT_FAIL (no Executing banner)"; tail -c 200 "$LOG" | tr -cd '[:print:]\n'; exit 1
fi
# Let the firmware finish printing its banner, then terminate litex_term so it stops reading.
sleep 3
kill "$TPID" 2>/dev/null
wait "$TPID" 2>/dev/null

# 2) Confirm NO litex_term is still attached (the whole point).
if ps -eo args 2>/dev/null | grep -q "[l]itex_term crossover"; then
    echo "BOOT_FAIL (litex_term still alive — would corrupt captures)"; exit 1
fi
echo "BOOT_OK (single reader free)"
