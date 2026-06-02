#!/usr/bin/env bash
# Shared helpers for the LiteNVMe HW harnesses. Source this, set LOG, then use:
#   hw_load [probe]   - kill stale servers, load the bitstream (optional --litescope-probe).
#   hw_serve          - start one litex_server (UDP); sets $SRV.
#   hw_link_decode    - print PCIe link status (rate/width/ltssm/MPS) over etherbone.
#   hw_boot           - upload firmware over the crossover UART, then stop litex_term so
#                       engine_console.py is the sole reader (single-reader discipline).
#   hw_cmd <n> <s> .. - run one firmware command via engine_console.py, capture -> LOG.
#   hw_stop           - kill the litex_server started by hw_serve.
# Conventions: run from bench/, LOG is set by the caller, IP/CSR/FW below.

set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

IP=192.168.1.50
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
SOC_ARGS="--with-cpu --cpu-boot=bios --with-etherbone --with-io-engine"

say() { echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }

hw_kill_servers() {
    for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do
        kill "$p" 2>/dev/null
    done
}

hw_load() {  # $1: optional litescope probe name.
    # --csr-csv writes the complete csr.csv (uart_xover, abs addresses); build/.../csr.csv can be
    # a stale partial artifact, so make the complete one canonical.
    [ -f csr.csv ] && cp -f csr.csv "$CSR"
    hw_kill_servers; sleep 2
    local probe=""; [ -n "${1:-}" ] && probe="--litescope-probe $1"
    python3 alibaba_xcku3p.py $SOC_ARGS $probe --load >/tmp/hw_load.log 2>&1 \
        && say "load OK" || { say "LOAD FAIL"; return 2; }
    sleep 6
}

hw_serve() {
    ( litex_server --udp --udp-ip $IP >/tmp/hw_server.log 2>&1 ) & SRV=$!
    sleep 4
}

hw_link_decode() {
    say "link decode:"
    python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b = RemoteClient(csr_csv=sys.argv[1]); b.open()
ls = b.regs.pcie_phy_phy_link_status.read()
rate  = {0:"Gen1",1:"Gen2",2:"Gen3"}.get((ls>>4)&3, "?")
width = {0:"x1",1:"x2",2:"x4",3:"x8",4:"x16"}.get((ls>>6)&7, "?")
print("  link_status=0x%04x  up=%d  rate=%s  width=%s  ltssm=0x%02x" % (ls, ls&1, rate, width, (ls>>9)&0x3f))
print("  RC MPS=%dB MRRS=%dB" % (b.regs.pcie_phy_phy_max_payload_size.read(),
                                 b.regs.pcie_phy_phy_max_request_size.read()))
b.close()
PY
}

hw_boot() {
    : > /tmp/hw_boot.log
    # litex_term needs a PTY -> run under `script -qfc`; it must FULLY EXIT before engine_console.py
    # (the sole crossover-UART reader) runs any command.
    ( script -qfc "timeout 220 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/hw_boot.log >/dev/null 2>&1 ) &
    local i
    for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/hw_boot.log 2>/dev/null && break; sleep 1; done
    say "boot: $(grep -aq 'Executing booted program' /tmp/hw_boot.log && echo ok || echo FAIL)"
    sleep 3
    for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
    sleep 1
    python3 engine_console.py boot dummy >/tmp/hw_bootchk.txt 2>&1
    grep -q "ok=1" /tmp/hw_bootchk.txt || { say "not at prompt"; return 4; }
}

hw_cmd() {  # $1: label, $2: settle seconds, $3..: command.
    local n="$1" s="$2"; shift 2
    say "CMD $n: $*"
    python3 engine_console.py cmd "$*" "$s" "/tmp/hw_${n}.txt" >>"$LOG" 2>&1
    echo "----- $n -----" >>"$LOG"; cat "/tmp/hw_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"
}

hw_stop() { kill "${SRV:-0}" 2>/dev/null; }
