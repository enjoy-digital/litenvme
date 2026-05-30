#!/usr/bin/env bash
# Single-reader HW boot for the NVMe engine bench.
#
# Fixes two harness issues that produced "corrupt"/empty captures:
#  1) litex_term's crossover2pty thread drains uart_xover_rxtx for its whole lifetime, so it
#     must FULLY EXIT before engine_console.py (the sole reader) runs any command.
#  2) litex_term needs a PTY (termios.tcgetattr) -> run it under `script -qfc`, else it dies
#     with "Inappropriate ioctl for device" and the firmware never boots (board stays at the
#     litex> BIOS prompt -> all later captures are BIOS "Command not found").
#
# Usage: run_engine_session.sh     # boots fw, kills litex_term, asserts single reader free.
# Prereqs: bitstream loaded, ONE litex_server --udp running, csr.csv copied into build/.
set -u
cd "$(dirname "$0")"
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
LOG=/tmp/engine_boot.log

rm -f "$LOG"
# litex_term under a pty, bounded, backgrounded; uploads the kernel then we stop it.
( script -qfc "timeout 90 litex_term crossover --csr-csv $CSR --kernel $FW" "$LOG" >/dev/null 2>&1 ) &

for i in $(seq 1 70); do
    grep -aq "Executing booted program" "$LOG" 2>/dev/null && break
    sleep 1
done
if ! grep -aq "Executing booted program" "$LOG" 2>/dev/null; then
    echo "BOOT_FAIL (no Executing banner)"; tail -c 300 "$LOG" 2>/dev/null | tr -cd '[:print:]\n'; exit 1
fi
sleep 3  # let the firmware finish printing its banner.

# Stop every litex_term/script so no reader thread remains on the crossover UART.
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do
    kill "$p" 2>/dev/null
done
sleep 1
if ps -eo args 2>/dev/null | grep -q "[l]itex_term crossover"; then
    echo "BOOT_FAIL (litex_term still alive — would corrupt captures)"; exit 1
fi
echo "BOOT_OK (single reader free)"
