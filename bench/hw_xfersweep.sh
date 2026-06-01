#!/usr/bin/env bash
# Read transfer-size sweep on the current Gen3-256b build (MPS is SSD-capped at 512B; transfer size
# is the live lever). Measures 4/8/16/32/64 KiB reads. Throughput bench only (overlapping buffers).
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/xfer.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv; FW=firmware/firmware.bin
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/xfer_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo XFER_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/xfer_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/xfer_boot.log
( script -qfc "timeout 260 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/xfer_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/xfer_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/xfer_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/xfer_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/xfer_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run r4   20 "nvme_engine_bench read 0xe0000000 1 2000000 8 1000 8"
run r8   20 "nvme_engine_bench read 0xe0000000 1 2100000 16 1000 16"
run r16  20 "nvme_engine_bench read 0xe0000000 1 2200000 32 1000 32"
run r32  20 "nvme_engine_bench read 0xe0000000 1 2300000 64 500 64"
run r64  20 "nvme_engine_bench read 0xe0000000 1 2400000 128 500 128"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "XFER_DONE" >> "$LOG"
