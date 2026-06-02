#!/usr/bin/env bash
# Data-integrity check (firmware-driven, fresh queue): write a distinctive pattern (distinct bytes
# catch any byte/dword shuffle) to an LBA range and read it back. Run on a fresh boot, NOT mixed
# with the RTL engine (engine and firmware keep separate queue state). -> /tmp/integ.log
LOG=/tmp/integ.log; : > "$LOG"
source "$(dirname "$0")/hw_common.sh"

hw_load        || { echo INTEG_DONE >>"$LOG"; exit 2; }
hw_serve
hw_boot        || { hw_stop; echo INTEG_DONE >>"$LOG"; exit 4; }

hw_cmd fill   8  "nvme_fill 0xDEADBEEF"
hw_cmd verify 25 "nvme_verify 0xe0000000 1 5600000 8 128"

say "DONE"; hw_stop; echo "INTEG_DONE" >> "$LOG"
