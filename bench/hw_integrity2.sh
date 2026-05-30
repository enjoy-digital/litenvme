#!/usr/bin/env bash
# End-to-end data-integrity for the LiteNVMe RTL engine (NO firmware rebuild).
# Round-trip: engine writes LBA0 from the host buffer -> capture buffer (= bytes sent to SSD)
# -> clobber the host buffer -> engine reads LBA0 back into the buffer -> compare. Read-back
# must EQUAL what was written, and must DIFFER from the clobber (proving the read really
# happened). Firmware already at the litenvme> prompt. Single reader at a time. -> /tmp/integ2.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/integ2.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
[ -f "$CSR" ] || CSR=../build/alibaba_xcku3p/csr.csv
mkdir -p build/alibaba_xcku3p; [ -f build/alibaba_xcku3p/csr.csv ] || cp "$CSR" build/alibaba_xcku3p/csr.csv
BUF=0x10010000

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/integ2_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4

python3 engine_console.py boot dummy >/tmp/integ2_bootchk.txt 2>&1
grep -q "ok=1" /tmp/integ2_bootchk.txt || { say "firmware NOT at prompt -> abort"; kill $SRV 2>/dev/null; echo "INTEG2_COMPLETE" >>"$LOG"; exit 3; }
say "firmware at prompt"

py(){ python3 hostmem_tool.py "$@" >>"$LOG" 2>&1; }
uart(){ say "UART: $1"; python3 engine_console.py cmd "$1" "$2" "/tmp/integ2_$3.txt" >>"$LOG" 2>&1; cat "/tmp/integ2_$3.txt" >>"$LOG" 2>&1; }

say "STEP1 pre-seed host buffer with 0xC0DE0000+i"
py fill $BUF 0xC0DE0000
say "STEP2 engine WRITE LBA0 (count=1 nlb=8): buffer -> SSD"
uart "nvme_engine_bench write 0xe0000000 1 0 8 1 1" 12 wr
say "STEP3 capture buffer = bytes actually sent to SSD"
py dump $BUF /tmp/integ2_pw.json
say "STEP4 clobber host buffer with 0xDEAD0000+i"
py fill $BUF 0xDEAD0000
py dump $BUF /tmp/integ2_clob.json
say "STEP5 engine READ LBA0 (count=1 nlb=8): SSD -> buffer"
uart "nvme_engine_bench read 0xe0000000 1 0 8 1 1" 12 rd
say "STEP6 capture buffer = bytes read back from SSD"
py dump $BUF /tmp/integ2_pr.json
say "STEP7 verdicts"
echo "--- read-back vs written (expect EQUAL) ---" >>"$LOG"; py compare /tmp/integ2_pw.json /tmp/integ2_pr.json
echo "--- read-back vs clobber (expect DIFFER) ---" >>"$LOG"; py differ /tmp/integ2_clob.json /tmp/integ2_pr.json

{
echo "=== SUMMARY ==="
echo "wr_completed=$(grep -aoE 'completed: [0-9]+' /tmp/integ2_wr.txt 2>/dev/null|head -1) wr_errors=$(grep -aoE 'errors: [0-9]+' /tmp/integ2_wr.txt 2>/dev/null|head -1)"
echo "rd_completed=$(grep -aoE 'completed: [0-9]+' /tmp/integ2_rd.txt 2>/dev/null|head -1) rd_errors=$(grep -aoE 'errors: [0-9]+' /tmp/integ2_rd.txt 2>/dev/null|head -1)"
echo "$(grep -aoE 'VERDICT_EQUAL=[A-Z]+' "$LOG"|tail -1)  $(grep -aoE 'VERDICT_DIFFER=[A-Z]+' "$LOG"|tail -1)"
} >>"$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_INTEG2"; echo "INTEG2_COMPLETE" >>"$LOG"
