#!/usr/bin/env bash
# Lever-B experiment, built on the PROVEN hw_readgap bring-up (reload bitstream + serialboot).
# Reloads the bitstream (clean controller), boots fw, baseline read (confirms controller live),
# raises SSD MPS+MRRS from the host over the SAME live link, re-measures. Single UART reader.
set -u
cd "$(dirname "$0")"
LOG=/tmp/mpsexp.log; : > "$LOG"
say(){ echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }
CSR=build/alibaba_xcku3p/csr.csv
FW=firmware/firmware.bin
BIT=build/alibaba_xcku3p/gateware/alibaba_xcku3p.bit

for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_server|[l]itex_term|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 2

# Bitstream is already resident on the board (loaded by the prior readgap run; volatile but the
# board was not power-cycled). The `--load` target import is broken in the current env
# (SoCCore export drift), and we don't need it -- just boot fw + poke cfg over the live link.
say "using already-resident bitstream"
ping -c2 -W1 192.168.1.50 >/tmp/mpsexp_ping.txt 2>&1 && say "PING ok" || say "PING FAIL"
( litex_server --udp --udp-ip 192.168.1.50 >/tmp/mpsexp_server.log 2>&1 ) & SRV=$!
say "litex_server pid=$SRV"; sleep 4

say "serialboot firmware..."
: > /tmp/mpsexp_boot.log
( script -qfc "timeout 120 litex_term crossover --csr-csv $CSR --kernel $FW" /tmp/mpsexp_boot.log >/dev/null 2>&1 ) &
for i in $(seq 1 90); do grep -aq "Executing booted program" /tmp/mpsexp_boot.log 2>/dev/null && break; sleep 1; done
grep -aq "Executing booted program" /tmp/mpsexp_boot.log 2>/dev/null && say "boot ok" || say "BOOT_FAIL"
sleep 3
for p in $(ps -eo pid,args 2>/dev/null | grep -E "[l]itex_term crossover|[s]cript -qfc" | awk '{print $1}'); do kill "$p" 2>/dev/null; done
sleep 1
python3 engine_console.py boot dummy >/tmp/mpsexp_bootchk.txt 2>&1
grep -q "ok=1" /tmp/mpsexp_bootchk.txt || { say "fw not at prompt -- abort"; kill $SRV 2>/dev/null; echo MPSEXP_DONE >> "$LOG"; exit 4; }

run(){ local n="$1" s="$2"; shift 2; say "CMD $n: $*"
  python3 engine_console.py cmd "$*" "$s" "/tmp/mpsexp_${n}.txt" >>"$LOG" 2>&1
  echo "----- $n -----" >>"$LOG"; cat "/tmp/mpsexp_${n}.txt" >>"$LOG" 2>&1; echo "----- end $n -----" >>"$LOG"; }

# Baseline read brings the controller fully live (admin init + IO setup happen here).
run base_r8 18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
if ! grep -aq "completed: 1000" /tmp/mpsexp_base_r8.txt; then
  say "baseline read did not complete -- controller not live, abort MPS poke"; kill $SRV 2>/dev/null; echo MPSEXP_DONE >> "$LOG"; exit 5; fi

say "=== host: inspect SSD MPS ==="; python3 /tmp/mps_experiment.py >>"$LOG" 2>&1
say "=== host: set SSD MPS enc=1 (256B) ==="; python3 /tmp/mps_experiment.py 1 >>"$LOG" 2>&1
run mps256_r8_a 18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
run mps256_r8_b 18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"
say "=== host: set SSD MPS enc=2 (512B) ==="; python3 /tmp/mps_experiment.py 2 >>"$LOG" 2>&1
run mps512_r8 18 "nvme_engine_bench read 0xe0000000 1 0 8 1000 8"

{
echo "=== MPSEXP SUMMARY ==="
for f in base_r8 mps256_r8_a mps256_r8_b mps512_r8; do
  echo "[$f] $(grep -aoE 'throughput: [0-9.]+ MB/s|hostmem_rd_tlps: [0-9]+|hostmem_rd_duty_pct: [0-9.]+ %|errors: [0-9]+|completed: [0-9]+' /tmp/mpsexp_${f}.txt 2>/dev/null | tr '\n' ' ')"
done
} >> "$LOG"
say "stopping litex_server"; kill $SRV 2>/dev/null
say "DONE_MPSEXP"; echo "MPSEXP_DONE" >> "$LOG"
