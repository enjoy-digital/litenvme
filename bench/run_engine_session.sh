#!/usr/bin/env bash
# Boot the firmware over the crossover UART and leave engine_console.py as the sole reader, for
# interactive use. Prereqs: bitstream loaded and ONE litex_server --udp already running.
LOG=/tmp/engine_boot.log; : > "$LOG"
source "$(dirname "$0")/hw_common.sh"

hw_boot && echo "BOOT_OK (single reader free)" || { echo "BOOT_FAIL"; tail -c 400 "$LOG" | tr -cd '[:print:]\n'; exit 1; }
