#!/usr/bin/env bash
# Load the hmwrite-probe bitstream, boot firmware, capture the SSD's completer MemWr of the admin
# Create IO CQ CQE (to ACQ+16 = 0x10001010) to see address/byte-enables/data placement at 256b.
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/hmcap.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --litescope-probe hmwrite --load >/tmp/hmcap_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo HMCAP_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/hmcap_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/hmcap_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/hmcap_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/hmcap_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/hmcap_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
say "capture:"
python3 /tmp/ls_hmwrite_cap.py >>"$LOG" 2>&1
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "HMCAP_DONE" >> "$LOG"
