#!/usr/bin/env bash
# Diagnose the 256-bit Gen3 admin-queue wedge (first run reaches Create IO CQ, then SSD wedges).
# Fresh load -> boot -> nvme_debug 1 -> MMIO integrity -> firmware-path single read (shared
# nvme_io_setup, PROVEN pcie_mmio doorbell). If the firmware read ALSO dies at Create IO CQ it
# is the shared admin/MMIO path at 256b, not the engine. Records board-printed output only.
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/diag256.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/diag256_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo DIAG_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/diag256_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/diag256_boot.log
( script -qfc "timeout 260 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/diag256_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/diag256_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/diag256_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/diag256_bootchk.txt 2>&1
grep -q "ok=1" /tmp/diag256_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo DIAG_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/diag256_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/diag256_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run dbg     8  "nvme_debug 1"
run mmiot   15 "nvme_mmiotest"
run fwread  25 "nvme_read 0xe0000000 1 0 1"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "DIAG_DONE" >> "$LOG"
