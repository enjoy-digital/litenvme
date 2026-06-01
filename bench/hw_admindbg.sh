#!/usr/bin/env bash
# Admin-path diagnostic at 256b: boot, then nvme_engine_diag (config readback + SQ/CQ dump) and a
# debug nvme_read, to see WHERE Create IO CQ fails. Reuses the loaded gateware (no reload).
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/admindbg.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/admindbg_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo ADMINDBG_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/admindbg_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/admindbg_boot.log
( script -qfc "timeout 240 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/admindbg_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/admindbg_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/admindbg_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/admindbg_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/admindbg_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run dbg   8  "nvme_debug 1"
run diag  30 "nvme_engine_diag read 0xe0000000 1 0 8 4"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "ADMINDBG_DONE" >> "$LOG"
