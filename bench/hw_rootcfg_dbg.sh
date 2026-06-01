#!/usr/bin/env bash
# Debug why cfg_mgmt writes to the root DevCtl don't apply. Firmware-only (existing bitstream).
# Tests whether ANY DevCtl field changes via cfg_mgmt write. -> /tmp/rootcfgdbg.log
set -u
cd "$(dirname "$0")"
LOG=/tmp/rootcfgdbg.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2
python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/rootcfgdbg_load.log 2>&1 && say "load OK" || { say "LOAD FAIL"; echo ROOTCFGDBG_DONE >>"$LOG"; exit 2; }
sleep 5
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/rootcfgdbg_server.log 2>&1 ) & SRV=$!
sleep 4
: > /tmp/rootcfgdbg_boot.log
( script -qfc "timeout 200 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/rootcfgdbg_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/rootcfgdbg_boot.log 2>/dev/null && break; sleep 1; done
say "boot: $(grep -aq 'Executing booted program' /tmp/rootcfgdbg_boot.log && echo ok || echo FAIL)"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/rootcfgdbg_bootchk.txt 2>&1
grep -q "ok=1" /tmp/rootcfgdbg_bootchk.txt || { say "not at prompt"; kill $SRV 2>/dev/null; echo ROOTCFGDBG_DONE >>"$LOG"; exit 4; }
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/rootcfgdbg_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/rootcfgdbg_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
# DevCtl is at dword 0x1e (cap 0x70 + 0x08 = 0x78, /4). Before; try changing MRRS (bits14:12)
# to 1 and MPS (bits7:5) to 2 with full byte-enable; then byte0-only; then read back.
run rd_before  10 "rootcfg rd 0x1e"
run wr_mrrs1   10 "rootcfg wr 0x1e 0x00001040 0xf"
run wr_be1     10 "rootcfg wr 0x1e 0x00002040 0x1"
run wr_mps_be3 10 "rootcfg wr 0x1e 0x00002000 0x3"
run rd_after   10 "rootcfg rd 0x1e"
# Sanity: also try writing the Link Control register (cap+0x10 = 0x80 -> dword 0x20) RW bits.
run lnkctl_rd  10 "rootcfg rd 0x20"
run lnkctl_wr  10 "rootcfg wr 0x20 0x00000040 0x1"
run lnkctl_rd2 10 "rootcfg rd 0x20"
say "stopping"; kill $SRV 2>/dev/null
say "DONE"; echo "ROOTCFGDBG_DONE" >> "$LOG"
