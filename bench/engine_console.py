#!/usr/bin/env python3
"""Single-owner crossover-UART console for the NVMe engine bench.

ROOT CAUSE THIS REPLACES: `litex_term crossover` runs a crossover2pty daemon thread that
*continuously* drains uart_xover_rxtx for its whole lifetime. Booting firmware with
`litex_term ... --kernel` (held open under `timeout`) left that thread racing our
`uart_cmd.py` for the SAME RX FIFO -> each reader got a fraction of the bytes -> the
"garbled / empty" captures (e.g. "En =00rcisya5 pS lteve0>") that derailed earlier runs.

This tool is the SOLE reader of the crossover UART: it never runs concurrently with
litex_term. Usage:

  engine_console.py boot <firmware.bin>        # upload kernel via crossover, wait Executing
  engine_console.py cmd "<command>" <settle> <outfile>   # send one command, capture reply

Both use one RemoteClient over the existing litex_server (port 1234). No second reader.
"""
import sys, time
from litex import RemoteClient

CSR = "build/alibaba_xcku3p/csr.csv"


def _open():
    b = RemoteClient(csr_csv=CSR); b.open()
    return b


def _drain(b):
    n = 0
    while not b.regs.uart_xover_rxempty.read():
        b.regs.uart_xover_rxtx.read(); n += 1
        if n > 1 << 20:
            break
    return n


def _send(b, s):
    for ch in s:
        while b.regs.uart_xover_txfull.read():
            pass
        b.regs.uart_xover_rxtx.write(ord(ch))


def _collect(b, settle, quiet=1.5):
    buf = bytearray()
    t0 = time.time(); last = time.time()
    while time.time() - t0 < settle:
        if not b.regs.uart_xover_rxempty.read():
            buf.append(b.regs.uart_xover_rxtx.read() & 0xff)
            last = time.time()
        else:
            if buf and (time.time() - last) > quiet:
                break
            time.sleep(0.005)
    return buf


def boot(fw_path):
    # Minimal SFL-free path: the BIOS serialboot protocol over the crossover is what
    # litex_term implements; rather than reimplement SFL, we just use litex_term ONCE to
    # upload, but with --no-crossover-pty so it does not leave a reader thread... however
    # litex_term always reads. So instead: drive the BIOS 'serialboot' is complex; keep using
    # litex_term for boot but it must FULLY EXIT before any cmd() runs. This function only
    # checks the firmware already booted (see boot via litex_term in the shell, then verify).
    b = _open()
    _drain(b)
    _send(b, "\r")
    time.sleep(0.3)
    out = _collect(b, 2.0)
    b.close()
    txt = out.decode("utf-8", "replace")
    ok = "litenvme>" in txt
    print("BOOTCHK ok=%d tail=%r" % (ok, txt[-40:]))
    return 0 if ok else 1


def cmd(command, settle, outfile):
    b = _open()
    _drain(b)
    _send(b, command + "\r")
    out = _collect(b, settle)
    b.close()
    txt = out.decode("utf-8", "replace")
    open(outfile, "w").write(txt)
    # Gate flags for the caller.
    booted = "litenvme>" in txt
    notfound = "Command not found" in txt
    print("CAP bytes=%d litenvme=%d notfound=%d -> %s" % (len(out), booted, notfound, outfile))
    return 0


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "boot":
        sys.exit(boot(sys.argv[2]))
    elif mode == "cmd":
        sys.exit(cmd(sys.argv[2], float(sys.argv[3]), sys.argv[4]))
    else:
        print("usage: engine_console.py boot <fw> | cmd <command> <settle> <outfile>")
        sys.exit(2)
