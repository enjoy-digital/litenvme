#!/usr/bin/env bash
# Lever 1 experiment: can we raise PCIe MPS (read-data TLP size) above 512B on this SSD, and does
# read throughput move? Reads the SSD DevCap, attempts MPS=1024B (enc 3), re-measures. Also sweeps
# transfer size. Firmware-only (no synth). -> /tmp/mpsexp.log
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/mpsexp.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mpsexp_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo MPSEXP_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpsexp_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/mpsexp_boot.log
( script -qfc "timeout 260 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpsexp_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpsexp_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/mpsexp_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpsexp_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpsexp_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run mps0   12 "nvme_mps"
run rd_base 20 "nvme_engine_bench read 0xe0000000 1 1000000 8 1000 8"
run set3   12 "nvme_mps set 3"
run rd_aft  20 "nvme_engine_bench read 0xe0000000 1 1500000 8 1000 8"
run rd_2k   20 "nvme_engine_bench read 0xe0000000 1 1600000 4 1000 4"
run rd_8k   20 "nvme_engine_bench read 0xe0000000 1 1700000 16 1000 16"
run rd_16k  20 "nvme_engine_bench read 0xe0000000 1 1800000 32 1000 32"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "MPSEXP_DONE" >> "$LOG"
