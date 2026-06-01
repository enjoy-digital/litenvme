#!/usr/bin/env bash
# Verify the 512B-MPS gateware: program BOTH ends to 512B and measure 4KiB reads.
#   Boot A (baseline): leave MPS at 128B default -> read (A/B reference).
#   Boot B (512B): rootmps set 2 (our root port, via cfg_mgmt) THEN nvme_mps setmps 2 (SSD,
#     first bar0_assign = clean cfg window) -> read. nvme_mps MUST precede any engine traffic.
# Records board-printed numbers only. -> /tmp/mps512.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/mps512.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
SRV=""

boot_board(){
  for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 2
  python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mps512_load.log 2>&1 || { say "LOAD FAIL"; return 2; }
  sleep 5
  ping -c2 -W1 192.168.1.50 >/dev/null 2>&1 || say "PING FAIL"
  ( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mps512_server.log 2>&1 ) & SRV=$!
  sleep 4
  : > /tmp/mps512_boot.log
  ( script -qfc "timeout 220 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mps512_boot.log >/dev/null 2>&1 ) &
  for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mps512_boot.log 2>/dev/null && break; sleep 1; done
  grep -aq "Executing booted program" /tmp/mps512_boot.log || { say "BOOT FAIL"; return 3; }
  sleep 3
  for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 1
  python3 engine_console.py boot dummy >/tmp/mps512_bootchk.txt 2>&1
  grep -q "ok=1" /tmp/mps512_bootchk.txt || { say "NOT AT PROMPT"; return 4; }
  return 0
}
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mps512_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mps512_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }

# ---- Boot A: 128B baseline ----
say "==================== BOOT A: 128B baseline ===================="
if boot_board; then
  run a_rootmps   12 "rootmps"
  run a_rd4k      20 "nvme_engine_bench read 0xe0000000 1 1000000 8 1000 8"
  [ -n "$SRV" ] && kill $SRV 2>/dev/null; sleep 1
else say "boot A failed"; [ -n "$SRV" ] && kill $SRV 2>/dev/null; fi

# ---- Boot B: 512B both ends ----
say "==================== BOOT B: 512B both ends ===================="
if boot_board; then
  run b_root_before 12 "rootmps"
  run b_root_set    12 "rootmps set 2"
  run b_ssd_set     15 "nvme_mps setmps 2"
  run b_rd4k_a      20 "nvme_engine_bench read 0xe0000000 1 1000000 8 1000 8"
  run b_rd4k_b      20 "nvme_engine_bench read 0xe0000000 1 2000000 8 1000 8"
  run b_rd8k        20 "nvme_engine_bench read 0xe0000000 1 0 16 1000 16"
  [ -n "$SRV" ] && kill $SRV 2>/dev/null; sleep 1
else say "boot B failed"; [ -n "$SRV" ] && kill $SRV 2>/dev/null; fi
say "DONE"; echo "MPS512_DONE" >> "$LOG"
