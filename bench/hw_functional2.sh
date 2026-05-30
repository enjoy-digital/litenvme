#!/usr/bin/env bash
# Self-contained HW functional test v2 for the LiteNVMe RTL engine.
# Robust to "firmware already running": detect the litenvme> prompt first; only run the
# litex_term serialboot upload if the board is still at the BIOS. Single crossover-UART
# reader at all times. EVERYTHING goes to /tmp/func2.log with a SUMMARY block at the end.
set -u
cd "$(dirname "$0")"
LOG=/tmp/func2.log
: > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
[ -f "$CSR" ] || CSR=../build/alibaba_xcku3p/csr.csv
[ -f "$FW" ]  || FW=../firmware/firmware.bin
say "CSR=$CSR FW=$FW"
mkdir -p build/alibaba_xcku3p; [ -f build/alibaba_xcku3p/csr.csv ] || cp "$CSR" build/alibaba_xcku3p/csr.csv

# 1. one litex_server
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do
  say "kill stale $p"; kill "$p" 2>/dev/null; done
sleep 2
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/func2_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4

# 2. integrity gate
python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b=RemoteClient(csr_csv=sys.argv[1]); b.open()
def rd(r): b.regs.__getattr__(r).read(); return b.regs.__getattr__(r).read()
link=rd("pcie_phy_phy_link_status") if hasattr(b.regs,"pcie_phy_phy_link_status") else None
ok=None
r=b.regs.nvme_gen_buf_stride
for v in (0xdead1234,0x5678beef):
    r.write(v); x=r.read(); x=r.read(); ok=(x==v); print("RT w=0x%08x r=0x%08x %s"%(v,x,"OK" if ok else "BAD"))
print("LINK=%s INTEG=%s"%(hex(link) if link is not None else "n/a","PASS" if ok else "FAIL"))
b.close()
PY
grep -q "INTEG=PASS" "$LOG" || { say "INTEGRITY FAIL -- abort"; kill $SRV 2>/dev/null; { echo "=== SUMMARY ==="; echo "RESULT=INTEGRITY_FAIL"; } >>"$LOG"; exit 2; }

# 3. is firmware already running? (engine_console boot-check looks for litenvme> prompt)
python3 engine_console.py boot "$FW" >/tmp/func2_bootchk.txt 2>&1
RUNNING=0; grep -q "ok=1" /tmp/func2_bootchk.txt && RUNNING=1
say "prompt-check: $(cat /tmp/func2_bootchk.txt | tr -cd '[:print:]') -> running=$RUNNING"

# 4. if not running, serialboot upload via litex_term then kill it
if [ "$RUNNING" = "0" ]; then
  say "firmware not at prompt -> serialboot upload via litex_term"
  : > /tmp/func2_boot.log
  ( script -qfc "timeout 90 litex_term crossover --csr-csv build/alibaba_xcku3p/csr.csv --kernel $FW" /tmp/func2_boot.log >/dev/null 2>&1 ) &
  for i in $(seq 1 80); do grep -aq "Executing booted program" /tmp/func2_boot.log 2>/dev/null && break; sleep 1; done
  if grep -aq "Executing booted program" /tmp/func2_boot.log 2>/dev/null; then say "boot banner seen"; RUNNING=1; else
    say "BOOT_FAIL banner not seen; boot tail:"; tail -c 600 /tmp/func2_boot.log | tr -cd '[:print:]\n' >>"$LOG"; fi
  sleep 3
  for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 1
fi

# 5. functional commands (sole reader) -- only if running
run_cmd(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/func2_${n}.txt" >>"$LOG" 2>&1
  echo "----- /tmp/func2_${n}.txt -----" >>"$LOG"; cat "/tmp/func2_${n}.txt" >>"$LOG" 2>&1; echo "----- end ${n} -----" >>"$LOG"; }

if [ "$RUNNING" = "1" ]; then
  run_cmd diag 10 "nvme_engine_diag read 0xe0000000 1 0 8 4"
  run_cmd benchr 14 "nvme_engine_bench read 0xe0000000 1 0 8 16 8"
else
  say "skip functional cmds: firmware not running"
fi

# 6. summary
{
  echo "=== SUMMARY ==="
  echo "INTEG=$(grep -o 'INTEG=[A-Z]*' "$LOG" | tail -1)"
  echo "LINK=$(grep -o 'LINK=0x[0-9a-f]*' "$LOG" | tail -1)"
  echo "RUNNING=$RUNNING"
  echo "diag_valid=$(grep -c 'litenvme>' /tmp/func2_diag.txt 2>/dev/null)"
  echo "bench_valid=$(grep -c 'litenvme>' /tmp/func2_benchr.txt 2>/dev/null)"
  echo "diag_completed_line=$(grep -aiE 'completed|cmp|errors|err=' /tmp/func2_diag.txt 2>/dev/null | tr -cd '[:print:]\n' | tr '\n' '|')"
  echo "bench_result_line=$(grep -aiE 'completed|MB/s|throughput|errors|iops' /tmp/func2_benchr.txt 2>/dev/null | tr -cd '[:print:]\n' | tr '\n' '|')"
} >> "$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_V2"
echo "FUNC2_COMPLETE" >> "$LOG"
