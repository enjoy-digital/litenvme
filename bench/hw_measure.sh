#!/usr/bin/env bash
# HW measurement battery for the LiteNVMe RTL engine. Firmware is assumed ALREADY running
# (board at the litenvme> prompt). Single crossover-UART reader (engine_console.py). All
# output -> /tmp/meas.log with per-command captures and a SUMMARY. Records nothing itself;
# I read the file and only then record reproduced numbers.
set -u
cd "$(dirname "$0")"
LOG=/tmp/meas.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
[ -f "$CSR" ] || CSR=../build/alibaba_xcku3p/csr.csv
mkdir -p build/alibaba_xcku3p; [ -f build/alibaba_xcku3p/csr.csv ] || cp "$CSR" build/alibaba_xcku3p/csr.csv

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do
  say "kill stale $p"; kill "$p" 2>/dev/null; done
sleep 2
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/meas_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4

# integrity gate
python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b=RemoteClient(csr_csv=sys.argv[1]); b.open()
def rd(r): b.regs.__getattr__(r).read(); return b.regs.__getattr__(r).read()
link=rd("pcie_phy_phy_link_status")
ok=None; r=b.regs.nvme_gen_buf_stride
for v in (0xcafe0001,0x0ddba11):
    r.write(v); x=r.read(); x=r.read(); ok=(x==v)
print("LINK=%s INTEG=%s"%(hex(link),"PASS" if ok else "FAIL")); b.close()
PY
grep -q "INTEG=PASS" "$LOG" || { say "INTEG FAIL abort"; kill $SRV 2>/dev/null; echo "MEAS_COMPLETE" >>"$LOG"; exit 2; }

python3 engine_console.py boot dummy >/tmp/meas_bootchk.txt 2>&1
grep -q "ok=1" /tmp/meas_bootchk.txt || { say "firmware NOT at prompt -> abort (boot it first)"; kill $SRV 2>/dev/null; echo "MEAS_COMPLETE" >>"$LOG"; exit 3; }
say "firmware at prompt; starting battery"

run(){ local name="$1" settle="$2"; shift 2; say "CMD $name: $*"
  python3 engine_console.py cmd "$*" "$settle" "/tmp/meas_${name}.txt" >>"$LOG" 2>&1
  echo "----- $name : $* -----" >>"$LOG"; cat "/tmp/meas_${name}.txt" >>"$LOG" 2>&1; echo "----- end $name -----" >>"$LOG"; }

# reads: 512B(nlb1) / 4KiB(nlb8) / 8KiB(nlb16), count=1000, two passes for nlb8
run r_nlb1_a  18 "nvme_engine_bench read 0xe0000000 1 0 1 1000 1"
run r_nlb8_a  18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r_nlb8_b  18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r_nlb16_a 18 "nvme_engine_bench read 0xe0000000 1 0 16 1000 16"
# writes (destructive to the test SSD LBAs, same as firmware qd sweep)
run w_nlb1_a  18 "nvme_engine_bench write 0xe0000000 1 0 1 1000 1"
run w_nlb8_a  18 "nvme_engine_bench write 0xe0000000 1 0 8 1000 8"
run w_nlb8_b  18 "nvme_engine_bench write 0xe0000000 1 0 8 1000 8"
run w_nlb16_a 18 "nvme_engine_bench write 0xe0000000 1 0 16 1000 16"

{
echo "=== SUMMARY (name | completed | submitted | errors | throughput | iops) ==="
for f in r_nlb1_a r_nlb8_a r_nlb8_b r_nlb16_a w_nlb1_a w_nlb8_a w_nlb8_b w_nlb16_a; do
  C=$(grep -aoE 'completed: [0-9]+' /tmp/meas_${f}.txt 2>/dev/null | head -1)
  S=$(grep -aoE 'submitted: [0-9]+' /tmp/meas_${f}.txt 2>/dev/null | head -1)
  E=$(grep -aoE 'errors: [0-9]+' /tmp/meas_${f}.txt 2>/dev/null | head -1)
  T=$(grep -aoE 'throughput: [0-9.]+ MB/s' /tmp/meas_${f}.txt 2>/dev/null | head -1)
  I=$(grep -aoE 'iops: [0-9.]+' /tmp/meas_${f}.txt 2>/dev/null | head -1)
  V=$(grep -c 'litenvme>' /tmp/meas_${f}.txt 2>/dev/null)
  echo "$f | valid=$V | $C | $S | $E | $T | $I"
done
} >> "$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_MEAS"; echo "MEAS_COMPLETE" >> "$LOG"
