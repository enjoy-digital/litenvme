#!/usr/bin/env bash
# Verify MPS auto-config: a PLAIN engine read (no manual rootmps/nvme_mps) should auto-raise both
# ends to 512B and hit ~1422 MB/s. Firmware-only (existing 512B-capable bitstream). -> /tmp/mpsauto.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/mpsauto.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mpsauto_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo MPSAUTO_DONE >>"$LOG"; exit 2; }
sleep 5
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpsauto_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/mpsauto_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpsauto_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpsauto_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/mpsauto_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/mpsauto_bootchk.txt 2>&1
grep -q "ok=1" /tmp/mpsauto_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo MPSAUTO_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpsauto_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpsauto_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
# FIRST command is a plain read: bar0_assign should auto-set MPS=512 in its clean window.
run rd4k_a 20 "nvme_engine_bench read 0xe0000000 1 1000000 8 1000 8"
run rd4k_b 20 "nvme_engine_bench read 0xe0000000 1 2000000 8 1000 8"
run rootmps 12 "rootmps"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "MPSAUTO_DONE" >> "$LOG"
