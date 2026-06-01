#!/usr/bin/env bash
# Load the mmio-probe (LiteScope) bitstream, boot firmware, then capture ONE MMIO CAP read on
# the sys-domain probe: raw cmp_source.dat[0:128] (header+data) vs memcmp_dat_lo (accessor latch).
# Pinpoints where the 256-bit MMIO read data is lost. -> /tmp/mmiocap.log + /tmp/mmio_cap.csv
set -u
cd "$(dirname "$0")"
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/mmiocap.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --litescope-probe mmio --load >/tmp/mmiocap_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo MMIOCAP_DONE >>"$LOG"; exit 2; }
sleep 6
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mmiocap_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/mmiocap_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mmiocap_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mmiocap_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/mmiocap_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
say "capture:"
python3 /tmp/ls_mmio_cap.py >>"$LOG" 2>&1
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "MMIOCAP_DONE" >> "$LOG"
