#!/usr/bin/env bash
# Confirmation experiment (no synth): set ONLY the SSD's MPS to 512B (MRRS untouched) as the
# FIRST command, print both ends' operational MPS, then read. If reads fail while RC op_mps=128B
# and the link stays up, the root complex is rejecting the >128B MemWr -> root-port MPS must be
# raised in gateware. Also reads link_status + EP/RC MPS over etherbone after the failed read.
# -> /tmp/mpsconfirm.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/mpsconfirm.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/mpsconfirm_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo MPSCONFIRM_DONE >>"$LOG"; exit 2; }
sleep 5
ping -c2 -W1 192.168.1.50 >/dev/null 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpsconfirm_server.log 2>&1 ) & SRV=$!
say "server pid=$SRV"; sleep 4
: > /tmp/mpsconfirm_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpsconfirm_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpsconfirm_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/mpsconfirm_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/mpsconfirm_bootchk.txt 2>&1
grep -q "ok=1" /tmp/mpsconfirm_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo MPSCONFIRM_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpsconfirm_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpsconfirm_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
csr_snapshot(){ local tag="$1"
  say "CSR snapshot ($tag):"
  python3 - "$tag" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b = RemoteClient(csr_csv="build/alibaba_xcku3p/csr.csv"); b.open()
print("  RC  MPS=%dB MRRS=%dB" % (b.regs.pcie_phy_phy_max_payload_size.read(), b.regs.pcie_phy_phy_max_request_size.read()))
print("  EP  MPS=%dB MRRS=%dB" % (b.regs.pcie_endpoint_phy_max_payload_size.read(), b.regs.pcie_endpoint_phy_max_request_size.read()))
print("  link_status=0x%04x" % b.regs.pcie_phy_phy_link_status.read())
b.close()
PY
}
# Baseline at 128B first (proves reads work), then MPS-only=512, then read again.
csr_snapshot "after boot, pre-set"
run setmps512 15 "nvme_mps setmps 2"
csr_snapshot "after setmps 512 (SSD)"
run rd_short 20 "nvme_engine_bench read 0xe0000000 1 0 8 64 8"
csr_snapshot "after read attempt"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "MPSCONFIRM_DONE" >> "$LOG"
