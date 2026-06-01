#!/usr/bin/env bash
# Verify Gen3 x4 + 256-bit datapath: confirm the link trains at 8.0 GT/s x4, then measure
# read/write throughput (auto-MPS 512B). Records board-printed numbers only. -> /tmp/gen3.log
set -u
cd "$(dirname "$0")"
# The build writes the complete csr.csv (with uart_xover, abs addresses) to ./csr.csv via
# --csr-csv; build/.../csr.csv can be a stale/partial artifact. Make the complete one canonical.
[ -f csr.csv ] && cp -f csr.csv build/alibaba_xcku3p/csr.csv
LOG=/tmp/gen3.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/gen3_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo GEN3_DONE >>"$LOG"; exit 2; }
sleep 6
ping -c2 -W1 192.168.1.50 >/dev/null 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/gen3_server.log 2>&1 ) & SRV=$!
sleep 4
# Decode the PCIe link status over etherbone BEFORE booting firmware (link trains at power-on).
say "link decode:"
python3 - >>"$LOG" 2>&1 <<'PY'
from litex import RemoteClient
b = RemoteClient(csr_csv="build/alibaba_xcku3p/csr.csv"); b.open()
ls = b.regs.pcie_phy_phy_link_status.read()
rate = {0:"Gen1",1:"Gen2",2:"Gen3"}.get((ls>>4)&3, "?")
width = {0:"x1",1:"x2",2:"x4",3:"x8",4:"x16"}.get((ls>>6)&7, "?")
print("  link_status=0x%04x  up=%d  rate=%s  width=%s  ltssm=0x%02x" % (
    ls, ls&1, rate, width, (ls>>9)&0x3f))
print("  RC MPS=%dB MRRS=%dB" % (b.regs.pcie_phy_phy_max_payload_size.read(), b.regs.pcie_phy_phy_max_request_size.read()))
b.close()
PY
: > /tmp/gen3_boot.log
( script -qfc "timeout 220 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/gen3_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/gen3_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/gen3_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/gen3_bootchk.txt 2>&1
grep -q "ok=1" /tmp/gen3_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo GEN3_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/gen3_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/gen3_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
# Headline reads/writes at 8 KiB (nlb=16) -- the throughput sweet spot at the SSD's 512B MPS
# ceiling (~+22% over 4 KiB). Auto-MPS 512B in the first bar0_assign. 4 KiB kept as a reference.
run rd8k_a  20 "nvme_engine_bench read  0xe0000000 1 1000000 16 1000 16"
run rd8k_b  20 "nvme_engine_bench read  0xe0000000 1 2000000 16 1000 16"
run rd4k    20 "nvme_engine_bench read  0xe0000000 1 1500000 8 1000 8"
run wr8k_a  20 "nvme_engine_bench write 0xe0000000 1 3000000 16 1000 16"
run wr8k_b  20 "nvme_engine_bench write 0xe0000000 1 4000000 16 1000 16"
run rootmps 12 "rootmps"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "GEN3_DONE" >> "$LOG"
