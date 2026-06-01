#!/usr/bin/env bash
# Load the instrumented bitstream, boot fw, capture SSD identify + read battery with the new
# hostmem read-path duty/gap counters. Single crossover-UART reader. All to /tmp/readgap.log.
set -u
cd "$(dirname "$0")"
LOG=/tmp/readgap.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
BIT=build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit
say "bit: $(ls -la $BIT 2>&1 | tr -cd '[:print:]')"

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2

say "loading bitstream..."
if python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/readgap_load.log 2>&1; then
  say "load OK"
else
  say "load via target failed; openFPGALoader fallback"
  openFPGALoader -c digilent_hs2 "$BIT" >>/tmp/readgap_load.log 2>&1 && say "openFPGALoader OK" || { say "LOAD FAILED"; echo READGAP_DONE >> "$LOG"; exit 2; }
fi
sleep 5

ping -c2 -W1 192.168.1.50 >/tmp/readgap_ping.txt 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/readgap_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4
python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b=RemoteClient(csr_csv=sys.argv[1]); b.open()
def rd(r): b.regs.__getattr__(r).read(); return b.regs.__getattr__(r).read()
link=rd("pcie_phy_phy_link_status")
ok=None; r=b.regs.nvme_gen_buf_stride
for v in (0xa5a5f00d,0x12345678):
    r.write(v); x=r.read(); x=r.read(); ok=(x==v)
print("LINK=%s INTEG=%s"%(hex(link),"PASS" if ok else "FAIL")); b.close()
PY
grep -q "INTEG=PASS" "$LOG" || { say "INTEG FAIL -- abort"; kill $SRV 2>/dev/null; echo READGAP_DONE >> "$LOG"; exit 3; }

say "serialboot firmware..."
: > /tmp/readgap_boot.log
( script -qfc "timeout 100 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/readgap_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/readgap_boot.log 2>/dev/null && break; sleep 1; done
grep -aq "Executing booted program" /tmp/readgap_boot.log 2>/dev/null && say "boot banner seen" || say "BOOT_FAIL banner"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1

python3 engine_console.py boot dummy >/tmp/readgap_bootchk.txt 2>&1
grep -q "ok=1" /tmp/readgap_bootchk.txt || { say "fw not at prompt -- abort"; kill $SRV 2>/dev/null; echo READGAP_DONE >> "$LOG"; exit 4; }
say "firmware at prompt; running identify + read battery"

run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/readgap_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/readgap_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }

# SSD identity (model/serial/fw-rev) for the spec cross-check.
run identify 10 "nvme_identify"
# Read battery with the new duty/gap counters: 4KiB x2, cache-busting high-LBA, 8KiB; + a write ref.
run r8_a    18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r8_b    18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r8_lba9m 18 "nvme_engine_bench read 0xe0000000 1 9000000 8 1000 8"
run r16_a   18 "nvme_engine_bench read 0xe0000000 1 0 16 1000 16"
run w8_a    18 "nvme_engine_bench write 0xe0000000 1 0 8 1000 8"

{
echo "=== READGAP SUMMARY ==="
for f in r8_a r8_b r8_lba9m r16_a w8_a; do
  echo "[$f] $(grep -aoE 'throughput: [0-9.]+ MB/s|errors: [0-9]+|max_inflight: [0-9]+|hostmem_rd_present_cycles: [0-9]+|hostmem_rd_stall_cycles: [0-9]+|hostmem_rd_tlps: [0-9]+|hostmem_rd_gap_max: [0-9]+|hostmem_rd_duty_pct: [0-9.]+ %|cycles: [0-9]+' /tmp/readgap_${f}.txt 2>/dev/null | tr '\n' ' ')"
done
echo "[identify] $(grep -aiE 'model|mn=|serial|firmware' /tmp/readgap_identify.txt 2>/dev/null | tr -cd '[:print:]\n' | tr '\n' '|')"
} >> "$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_READGAP"; echo "READGAP_DONE" >> "$LOG"
