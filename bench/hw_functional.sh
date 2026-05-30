#!/usr/bin/env bash
# Self-contained HW functional test for the LiteNVMe RTL engine.
# Single-reader discipline: litex_server up, litex_term boots fw then EXITS, then
# engine_console.py is the SOLE crossover-UART reader. Everything logged to /tmp.
#
# Usage: hw_functional.sh   (run in background; read /tmp/func.log when done)
set -u
cd "$(dirname "$0")"
LOG=/tmp/func.log
: > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

# --- 0. discover assets ---------------------------------------------------
CSR=""
for c in build/alibaba_xcku3p/csr.csv ../build/alibaba_xcku3p/csr.csv csr.csv; do
  [ -f "$c" ] && CSR="$c" && break
done
BIT=""
for b in build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit ../build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit; do
  [ -f "$b" ] && BIT="$b" && break
done
FW=""
for f in firmware/firmware.bin ../firmware/firmware.bin firmware.bin; do
  [ -f "$f" ] && FW="$f" && break
done
say "CSR=$CSR BIT=$BIT FW=$FW"
[ -z "$CSR" ] && { say "FATAL no csr.csv"; exit 1; }
[ -z "$FW" ]  && { say "FATAL no firmware.bin"; exit 1; }
# make sure engine_console.py's hardcoded path resolves too
mkdir -p build/alibaba_xcku3p
[ -f build/alibaba_xcku3p/csr.csv ] || cp "$CSR" build/alibaba_xcku3p/csr.csv

# --- 1. kill stale readers ------------------------------------------------
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do
  say "kill stale pid $p"; kill "$p" 2>/dev/null
done
sleep 2

# --- 2. ping --------------------------------------------------------------
if ping -c2 -W1 192.168.1.50 >/tmp/func_ping.txt 2>&1; then
  say "PING ok ($(grep -o '[0-9]*% packet loss' /tmp/func_ping.txt))"
else
  say "PING FAIL"; tail -3 /tmp/func_ping.txt | tee -a "$LOG"
fi

# --- 3. start ONE litex_server --------------------------------------------
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/func_server.log 2>&1 ) &
SRV=$!
say "litex_server pid=$SRV"
sleep 4

# --- 4. integrity gate (host RemoteClient) --------------------------------
python3 - "$CSR" >/tmp/func_integ.txt 2>&1 <<'PY'
import sys, time
from litex import RemoteClient
b = RemoteClient(csr_csv=sys.argv[1]); b.open()
def rd(reg):  # double-read: this bridge can lag one transaction
    b.regs.__getattr__(reg).read(); return b.regs.__getattr__(reg).read()
link = rd("pcie_phy_phy_link_status") if hasattr(b.regs,"pcie_phy_phy_link_status") else None
# roundtrip on a spare engine CSR
ok_rt = None
for spare in ("nvme_gen_buf_stride","nvme_gen_buf_base","nvme_engine_engine_sq_db"):
    if hasattr(b.regs, spare):
        r = b.regs.__getattr__(spare)
        for v in (0xa5a5f00d, 0x12345678):
            r.write(v); rb = r.read(); rb = r.read()
            ok_rt = (rb == v)
            print("RT %s w=0x%08x r=0x%08x %s" % (spare, v, rb, "OK" if ok_rt else "MISMATCH"))
        break
print("LINK=%s" % (hex(link) if link is not None else "n/a"))
print("INTEG=%s" % ("PASS" if ok_rt else "FAIL"))
b.close()
PY
cat /tmp/func_integ.txt | tee -a "$LOG"
if ! grep -q "INTEG=PASS" /tmp/func_integ.txt; then
  say "INTEGRITY GATE FAILED -- aborting before any test (would be untrustworthy)"
  kill "$SRV" 2>/dev/null
  exit 2
fi

# --- 5. boot firmware single-reader ---------------------------------------
say "booting firmware (litex_term under pty, will exit after boot)"
: > /tmp/func_boot.log
( script -qfc "timeout 90 litex_term crossover --csr-csv build/alibaba_xcku3p/csr.csv --kernel $FW" /tmp/func_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 70); do
  grep -aq "Executing booted program" /tmp/func_boot.log 2>/dev/null && break
  sleep 1
done
if ! grep -aq "Executing booted program" /tmp/func_boot.log 2>/dev/null; then
  say "BOOT_FAIL (no Executing banner); tail:"; tail -c 400 /tmp/func_boot.log | tr -cd '[:print:]\n' | tee -a "$LOG"
  kill "$SRV" 2>/dev/null; exit 3
fi
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
if ps -eo args 2>/dev/null | grep -q "[l]itex_term crossover"; then
  say "BOOT_FAIL litex_term still alive"; kill "$SRV" 2>/dev/null; exit 3
fi
say "BOOT_OK single reader free"

# --- 6. functional commands (sole reader) ---------------------------------
run_cmd(){  # name settle command
  local name="$1" settle="$2"; shift 2
  say "CMD $name: $*"
  python3 engine_console.py cmd "$*" "$settle" "/tmp/func_${name}.txt" >>"$LOG" 2>&1
  echo "----- /tmp/func_${name}.txt -----" >>"$LOG"
  cat "/tmp/func_${name}.txt" >>"$LOG" 2>&1
  echo "----- end ${name} -----" >>"$LOG"
}

run_cmd diag 8  "nvme_engine_diag"
run_cmd ebench_r 12 "nvme_engine_bench read 0xe0000000 1 0 8 16 8 1"

say "DONE functional run"
kill "$SRV" 2>/dev/null
say "litex_server stopped; ALL CLEAN"
