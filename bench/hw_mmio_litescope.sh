#!/usr/bin/env bash
# Load the litescope-mmio bitstream, boot firmware, then capture the MMIO request/completion
# TLP signals on HW while nvme_mmiotest runs. -> /tmp/mmiols.log + /tmp/mmio_cap.csv
set -u
cd /home/florent/dev/litex/litenvme/bench
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
CSR=build/alibaba_xcku3p/csr.csv; FW=firmware/firmware.bin; LOG=/tmp/mmiols.log; : > "$LOG"
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --litescope-probe=mmio --load >/tmp/mmiols_load.log 2>&1 || { echo LOADFAIL>>"$LOG"; echo MMIOLS_DONE>>"$LOG"; exit 2; }
sleep 6
litex_server --udp --udp-ip 192.168.1.50 >/tmp/mmiols_srv.log 2>&1 & SRV=$!
sleep 4
( script -qfc "timeout 150 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mmiols_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mmiols_boot.log 2>/dev/null && break; sleep 1; done
echo "boot: $(grep -aq 'Executing booted program' /tmp/mmiols_boot.log && echo ok || echo FAIL)" >>"$LOG"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/mmiols_bootchk.txt 2>&1
grep -q "ok=1" /tmp/mmiols_bootchk.txt || { echo "not at prompt">>"$LOG"; kill $SRV; echo MMIOLS_DONE>>"$LOG"; exit 4; }
python3 /tmp/ls_capture.py >>"$LOG" 2>&1
kill $SRV 2>/dev/null
echo MMIOLS_DONE >>"$LOG"
