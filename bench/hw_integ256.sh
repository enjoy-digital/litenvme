#!/usr/bin/env bash
# Data-integrity check at 256b: write a known pattern to a high LBA, read it back, dump. Confirms
# the completer write/read datapaths are bit-correct (not just errors=0). -> /tmp/integ256.log
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/integ256.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/integ256_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo INTEG_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/integ256_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/integ256_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/integ256_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/integ256_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/integ256_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/integ256_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/integ256_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
# write+read-back the same LBA, dump 16 dwords: data must equal the firmware fill pattern.
run wrb 25 "nvme_write_readback 0xe0000000 1 5500000 8 16"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "INTEG_DONE" >> "$LOG"
