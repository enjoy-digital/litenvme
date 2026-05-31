#!/usr/bin/env bash
# Post-synthesis bring-up + measurement for the coalesced-doorbell engine.
# 1) load the freshly-built bitstream, 2) ping + one litex_server + integrity gate,
# 3) serialboot the firmware (board is at BIOS after a fresh load) as the SINGLE reader,
# 4) run the read/write battery. All to /tmp/postsynth.log; records nothing itself.
set -u
cd "$(dirname "$0")"
LOG=/tmp/postsynth.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
BIT=build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit

say "bit: $(ls -la $BIT 2>&1 | tr -cd '[:print:]')"

# 0. stop everything that talks to the board.
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2

# 1. load the new bitstream (JTAG, volatile).
say "loading bitstream..."
if python3 alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone --with-io-engine --load >/tmp/postsynth_load.log 2>&1; then
  say "load OK"
else
  say "load via target FAILED; tail:"; tail -5 /tmp/postsynth_load.log | tr -cd '[:print:]\n' >> "$LOG"
  say "trying openFPGALoader digilent_hs2..."
  openFPGALoader -c digilent_hs2 "$BIT" >>/tmp/postsynth_load.log 2>&1 && say "openFPGALoader OK" || { say "LOAD FAILED -- abort"; echo POSTSYNTH_DONE >> "$LOG"; exit 2; }
fi
sleep 5   # let the link re-train / host settle.

# 2. ping + server + integrity gate.
ping -c2 -W1 192.168.1.50 >/tmp/postsynth_ping.txt 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/postsynth_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4
python3 - "$CSR" >>"$LOG" 2>&1 <<'PY'
import sys
from litex import RemoteClient
b=RemoteClient(csr_csv=sys.argv[1]); b.open()
def rd(r): b.regs.__getattr__(r).read(); return b.regs.__getattr__(r).read()
try:
    link=rd("pcie_phy_phy_link_status");
except Exception as e:
    print("LINK read failed: %r"%e); link=None
ok=None
try:
    r=b.regs.nvme_gen_buf_stride
    for v in (0xa5a5f00d,0x12345678):
        r.write(v); x=r.read(); x=r.read(); ok=(x==v)
except Exception as e:
    print("RT failed: %r"%e)
print("LINK=%s INTEG=%s"%(hex(link) if link is not None else "n/a","PASS" if ok else "FAIL"))
b.close()
PY
if ! grep -q "INTEG=PASS" "$LOG"; then say "INTEGRITY FAIL after load -- abort"; kill $SRV 2>/dev/null; echo POSTSYNTH_DONE >> "$LOG"; exit 3; fi

# 3. serialboot firmware (fresh board -> at BIOS) as the single reader.
say "serialboot firmware..."
: > /tmp/postsynth_boot.log
( script -qfc "timeout 100 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/postsynth_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/postsynth_boot.log 2>/dev/null && break; sleep 1; done
if grep -aq "Executing booted program" /tmp/postsynth_boot.log 2>/dev/null; then say "boot banner seen"; else
  say "BOOT_FAIL banner; tail:"; tail -c 500 /tmp/postsynth_boot.log | tr -cd '[:print:]\n' >> "$LOG"; fi
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1

# 4. confirm prompt then run the read/write battery (4KiB nlb8, count=1000, two passes) +
#    a couple of cache-busting high-LBA reads. engine_console is the sole reader.
python3 engine_console.py boot dummy >/tmp/postsynth_bootchk.txt 2>&1
grep -q "ok=1" /tmp/postsynth_bootchk.txt || { say "firmware not at prompt -- abort"; kill $SRV 2>/dev/null; echo POSTSYNTH_DONE >> "$LOG"; exit 4; }
say "firmware at prompt; running battery"
run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/postsynth_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/postsynth_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }
run diag    10 "nvme_engine_diag read 0xe0000000 1 0 8 4"
run r8_a    18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r8_b    18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run r8_lba9m 18 "nvme_engine_bench read 0xe0000000 1 9000000 8 1000 8"
run w8_a    18 "nvme_engine_bench write 0xe0000000 1 0 8 1000 8"

{
echo "=== SUMMARY (coalesced engine) ==="
for f in r8_a r8_b r8_lba9m w8_a; do
  echo "$f | $(grep -aoE 'completed: [0-9]+' /tmp/postsynth_${f}.txt|head -1) | $(grep -aoE 'errors: [0-9]+' /tmp/postsynth_${f}.txt|head -1) | $(grep -aoE 'throughput: [0-9.]+ MB/s' /tmp/postsynth_${f}.txt|head -1) | $(grep -aoE 'latency_avg: [0-9.]+ us' /tmp/postsynth_${f}.txt|head -1)"
done
echo "diag: $(grep -aoE 'final sub=[0-9]+ cmp=[0-9]+ err=[0-9]+.*' /tmp/postsynth_diag.txt 2>/dev/null | head -1)"
} >> "$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_POSTSYNTH"; echo "POSTSYNTH_DONE" >> "$LOG"
