#!/usr/bin/env bash
# End-to-end data-integrity probe for the LiteNVMe RTL engine.
# Strategy (no firmware rebuild): use the host-side hostmem debug port to place a known
# pattern in the host buffer, have the ENGINE write it to the SSD, clobber the host buffer,
# have the ENGINE read it back from the SSD, then read the host buffer and compare.
# If the host cannot reach hostmem, say so clearly (=> integrity needs a firmware command).
# Single crossover-UART reader; all output -> /tmp/integ.log.
set -u
cd "$(dirname "$0")"
LOG=/tmp/integ.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
[ -f "$CSR" ] || CSR=../build/alibaba_xcku3p/csr.csv
mkdir -p build/alibaba_xcku3p; [ -f build/alibaba_xcku3p/csr.csv ] || cp "$CSR" build/alibaba_xcku3p/csr.csv

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/integ_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4

# Step 1: discover the hostmem host-access method.
python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b=RemoteClient(csr_csv=sys.argv[1]); b.open()
names=[n for n in dir(b.regs) if "hostmem" in n.lower()]
print("HOSTMEM_REGS=%r" % names)
# Is hostmem wishbone-mapped for direct b.read? Try the buffer base.
BUF=0x10010000
direct=None
try:
    v=b.read(BUF); direct=v
    print("DIRECT_READ BUF=0x%08x -> 0x%08x" % (BUF, v))
except Exception as e:
    print("DIRECT_READ FAILED: %r" % e)
print("MEMS=%r" % [m for m in (getattr(b,'mems',None).__dict__.keys() if getattr(b,'mems',None) else [])])
b.close()
PY
say "discovery done"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_INTEG"; echo "INTEG_COMPLETE" >> "$LOG"
