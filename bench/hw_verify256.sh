#!/usr/bin/env bash
# Strong data-integrity check at 256b: distinctive pattern 0xDEADBEEF (distinct bytes -> catches any
# byte/dword shuffle in the completer datapath). Set pattern, write, read-back dump, then verify.
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/verify256.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/verify256_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo VERIFY_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/verify256_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/verify256_boot.log
( script -qfc "timeout 220 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/verify256_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/verify256_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/verify256_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/verify256_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/verify256_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run fill 8  "nvme_fill 0xDEADBEEF"
run wrb  25 "nvme_write_readback 0xe0000000 1 5600000 8 16"
run vrfy 25 "nvme_verify 0xe0000000 1 5600000 8 128"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "VERIFY_DONE" >> "$LOG"
