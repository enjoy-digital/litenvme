#!/usr/bin/env bash
# Gen3 x4 + 256-bit: link training + 8 KiB read/write throughput (auto-MPS 512B), QD32.
# Records board-printed numbers only. (Data integrity: see hw_integrity.sh.) -> /tmp/gen3.log
LOG=/tmp/gen3.log; : > "$LOG"
source "$(dirname "$0")/hw_common.sh"

hw_load            || { echo GEN3_DONE >>"$LOG"; exit 2; }
hw_serve
hw_link_decode     # link trains at power-on, before firmware boots.
hw_boot            || { hw_stop; echo GEN3_DONE >>"$LOG"; exit 4; }

# Throughput: 8 KiB (nlb=16) is the sweet spot at the SSD's 512B MPS ceiling; 4 KiB for reference.
hw_cmd rd8k_a 20 "nvme_engine_bench read  0xe0000000 1 1000000 16 1000 16"
hw_cmd rd8k_b 20 "nvme_engine_bench read  0xe0000000 1 2000000 16 1000 16"
hw_cmd rd4k   20 "nvme_engine_bench read  0xe0000000 1 1500000 8 1000 8"
hw_cmd wr8k_a 20 "nvme_engine_bench write 0xe0000000 1 3000000 16 1000 16"
hw_cmd wr8k_b 20 "nvme_engine_bench write 0xe0000000 1 4000000 16 1000 16"

say "DONE"; hw_stop; echo "GEN3_DONE" >> "$LOG"
