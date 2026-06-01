#!/usr/bin/env bash
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/prpdbg.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv; FW=firmware/firmware.bin
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/prpdbg_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo PRPDBG_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/prpdbg_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/prpdbg_boot.log
( script -qfc "timeout 120 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/prpdbg_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/prpdbg_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/prpdbg_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run2(){ python3 engine_console.py cmd "$1" "$2" "/tmp/prpdbg_$3.txt" >>"$LOG" 2>&1; echo "----- $3 -----" >>"$LOG"; cat "/tmp/prpdbg_$3.txt" >>"$LOG"; echo "----- end -----" >>"$LOG"; }
run2 "nvme_identify" 15 id
say "CMD: nvme_engine_bench read nlb=32"
python3 engine_console.py cmd "nvme_engine_bench read 0xe0000000 1 2200000 32 100 32" 35 /tmp/prpdbg_r.txt >>"$LOG" 2>&1
echo "----- diag -----" >>"$LOG"; cat /tmp/prpdbg_r.txt >>"$LOG"; echo "----- end -----" >>"$LOG"
kill $SRV 2>/dev/null; say "DONE"; echo "PRPDBG_DONE" >> "$LOG"
