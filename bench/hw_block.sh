#!/usr/bin/env bash
# Etherbone block-streamer test: load the (block-streamer) bitstream, boot firmware, run
# nvme_setup over the console (controller + engine bring-up, init_done=1), then drive
# throughput (request generator) + correctness (block streamer BIST) over Etherbone. The
# bitstream must have been built with --with-block-streamer. -> /tmp/block.log
LOG=/tmp/block.log; : > "$LOG"
source "$(dirname "$0")/hw_common.sh"
SOC_ARGS="$SOC_ARGS --with-block-streamer"

hw_load            || { echo BLOCK_DONE >>"$LOG"; exit 2; }
hw_serve
hw_link_decode
hw_boot            || { hw_stop; echo BLOCK_DONE >>"$LOG"; exit 4; }

# One-time NVMe bring-up over the console, then everything else over Etherbone.
hw_cmd setup 8 "nvme_setup"

say "etherbone test:"
python3 test_block.py --csr-csv "$CSR" >>"$LOG" 2>&1

say "DONE"; hw_stop; echo "BLOCK_DONE" >> "$LOG"
