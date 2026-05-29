#!/usr/bin/env python3
"""Send one firmware-console command over the crossover UART and capture the reply.

Usage: uart_cmd.py "<command>" <settle_seconds> <outfile>
Reads come from the soft CPU's UART (CSR-backed), so the firmware's printed output is the
source of truth here; the host only ferries bytes. Routes the captured text to <outfile>.
"""
import sys, time
from litex import RemoteClient

cmd     = sys.argv[1]
settle  = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
outfile = sys.argv[3] if len(sys.argv) > 3 else "/tmp/uart_out.txt"

b = RemoteClient(csr_csv="build/alibaba_xcku3p/csr.csv"); b.open()

# Drain any pending RX.
while not b.regs.uart_xover_rxempty.read():
    b.regs.uart_xover_rxtx.read()

# Send the command + CR.
for ch in cmd + "\r":
    while b.regs.uart_xover_txfull.read():
        pass
    b.regs.uart_xover_rxtx.write(ord(ch))

# Collect output until it goes quiet for ~1.5 s or the settle window elapses.
buf = bytearray()
t0 = time.time()
last_rx = time.time()
while time.time() - t0 < settle:
    if not b.regs.uart_xover_rxempty.read():
        buf.append(b.regs.uart_xover_rxtx.read() & 0xff)
        last_rx = time.time()
    else:
        if buf and (time.time() - last_rx) > 1.5:
            break
        time.sleep(0.01)

b.close()
text = buf.decode("utf-8", "replace")
with open(outfile, "w") as f:
    f.write(text)
print("CAPTURED %d bytes -> %s" % (len(buf), outfile))
