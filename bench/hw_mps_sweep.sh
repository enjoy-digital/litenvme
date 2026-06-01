#!/usr/bin/env bash
# Lever B sweep: for each MPS encoding (0=128B,1=256B,2=512B) do a FRESH boot, set the SSD's
# DevCtl.MPS as the FIRST command (clean cfg window), verify, then run 4KiB + 8KiB read benches.
# The engine 'errors' count is the safety net: if our root complex cannot receive the larger
# MemWr payload, reads will error -> that level is rejected. Record ONLY board-printed numbers.
# -> /tmp/mpssweep.log  (+ per-level captures /tmp/mpssweep_<enc>_<cmd>.txt)
set -u
cd "$(dirname "$0")"
LOG=/tmp/mpssweep.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

boot_board(){
  for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 2
  python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mpssweep_load.log 2>&1 || { say "LOAD FAIL"; return 2; }
  sleep 5
  ping -c2 -W1 192.168.1.50 >/dev/null 2>&1 || say "PING FAIL"
  ( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpssweep_server.log 2>&1 ) & SRV=$!
  sleep 4
  : > /tmp/mpssweep_boot.log
  ( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpssweep_boot.log >/dev/null 2>&1 ) &
  for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpssweep_boot.log 2>/dev/null && break; sleep 1; done
  grep -aq "Executing booted program" /tmp/mpssweep_boot.log || { say "BOOT FAIL"; return 3; }
  sleep 3
  for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 1
  python3 engine_console.py boot dummy >/tmp/mpssweep_bootchk.txt 2>&1
  grep -q "ok=1" /tmp/mpssweep_bootchk.txt || { say "NOT AT PROMPT"; return 4; }
  return 0
}
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpssweep_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpssweep_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }

SRV=""
for enc in 0 1 2; do
  say "==================== MPS enc=$enc ===================="
  boot_board || { say "boot failed at enc=$enc, skipping"; [ -n "$SRV" ] && kill $SRV 2>/dev/null; continue; }
  # FIRST command: set MPS (clean cfg window). It self-verifies: prints the before line
  # (mps_supported / devctl_mps / mrrs) and the after line (new devctl_mps) in the same window.
  # A separate 'nvme_mps' show would run in the dead-cfg window (2nd cmd) so it is omitted.
  run ${enc}_set 15 "nvme_mps set $enc"
  # Now measure reads at this MPS (two repeats of 4KiB for reproducibility, plus 8KiB).
  run ${enc}_rd4k_a 20 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
  run ${enc}_rd4k_b 20 "nvme_engine_bench read 0xe0000000 1 1000000 8 1000 8"
  run ${enc}_rd8k   20 "nvme_engine_bench read 0xe0000000 1 0 16 1000 16"
  [ -n "$SRV" ] && kill $SRV 2>/dev/null
  sleep 1
done
say "DONE"; echo "MPSSWEEP_DONE" >> "$LOG"
