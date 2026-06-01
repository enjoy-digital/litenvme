#!/usr/bin/env bash
# Run nvme_mps as the FIRST command after boot: its nvme_bar0_assign is then a cache-miss (full
# assign) and cfg access works in that clean window (before any admin/engine traffic steals the
# shared crossbar completions). dump+walk should now read real cfg and find the Express Cap.
# NO cfg writes here (read-only validation). -> /tmp/mpsfirst.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/mpsfirst.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mpsfirst_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo MPSFIRST_DONE >>"$LOG"; exit 2; }
sleep 5
ping -c2 -W1 192.168.1.50 >/dev/null 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpsfirst_server.log 2>&1 ) & SRV=$!
say "server pid=$SRV"; sleep 4
: > /tmp/mpsfirst_boot.log
( script -qfc "timeout 180 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpsfirst_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpsfirst_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/mpsfirst_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/mpsfirst_bootchk.txt 2>&1
grep -q "ok=1" /tmp/mpsfirst_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo MPSFIRST_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpsfirst_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpsfirst_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
# FIRST command = nvme_mps dump (cache-miss assign -> cfg works in clean window).
run dump 15 "nvme_mps dump"
run show 12 "nvme_mps"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "MPSFIRST_DONE" >> "$LOG"
