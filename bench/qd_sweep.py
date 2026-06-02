#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Self-contained queue-depth sweep over the firmware nvme_bench console command.

Runs entirely in one process with a single Etherbone RemoteClient (via the crossover
UART CSRs), so it does not depend on cwd/console.py imports or many short-lived shell
calls. Writes a parsed result table to stdout and to qd_sweep_result.txt.

Prereqs: bitstream loaded, `litex_server --udp` running, firmware booted (persists in
main_ram after litex_term detaches).
"""

import re
import sys
import time

from litex import RemoteClient

CSR_CSV = "build/alibaba_xcku3p/csr.csv"


class Console:
    def __init__(self, bus):
        self.bus = bus

    def _txfull(self):
        return self.bus.regs.uart_xover_txfull.read()

    def _rxempty(self):
        return self.bus.regs.uart_xover_rxempty.read()

    def drain(self, quiet_s=0.2, max_s=1.0):
        out = bytearray()
        last = time.time()
        start = time.time()
        while True:
            if not self._rxempty():
                out.append(self.bus.regs.uart_xover_rxtx.read() & 0xff)
                last = time.time()
            else:
                if time.time() - last > quiet_s:
                    break
                if time.time() - start > max_s:
                    break
                time.sleep(0.003)
        return out.decode("utf-8", "replace")

    def cmd(self, line, settle=30.0, quiet_s=0.5):
        self.drain(quiet_s=0.1, max_s=0.5)
        for ch in line:
            while self._txfull():
                pass
            self.bus.regs.uart_xover_rxtx.write(ord(ch))
        while self._txfull():
            pass
        self.bus.regs.uart_xover_rxtx.write(ord("\r"))
        out = bytearray()
        last = time.time()
        deadline = time.time() + settle
        while time.time() < deadline:
            if not self._rxempty():
                out.append(self.bus.regs.uart_xover_rxtx.read() & 0xff)
                last = time.time()
            else:
                if out and (time.time() - last > quiet_s):
                    break
                time.sleep(0.003)
        return out.decode("utf-8", "replace")


def parse(out):
    def g(key, cast=float):
        m = re.search(rf"{key}:\s*([0-9.]+)", out)
        return cast(m.group(1)) if m else None
    return {
        "qd":         (lambda m: int(m.group(1)) if m else None)(re.search(r"qd=(\d+)", out)),
        "throughput": g("throughput"),
        "iops":       g("iops"),
        "errors":     g("errors", int),
        "timeout":    "WINDOW TIMEOUT" in out,
        "raw":        out,
    }


def main():
    bus = RemoteClient(csr_csv=CSR_CSV)
    bus.open()
    con = Console(bus)

    # Sanity: console responds.
    h = con.cmd("help", settle=5)
    if "nvme_bench" not in h:
        print("ERROR: firmware console not responding to 'help'. Got:\n" + h[-300:])
        bus.close()
        sys.exit(2)

    qds = [1, 2, 4, 8, 16, 32, 63]
    lines = []
    for op, lba in (("read", 0), ("write", 1024)):
        for qd in qds:
            cmd = f"nvme_bench {op} 0xe0000000 1 {lba} 8 1000 8 1 {qd}"
            out = con.cmd(cmd, settle=40)
            r = parse(out)
            tag = "TIMEOUT" if r["timeout"] else f"err={r['errors']}"
            line = (f"{op:5s} qd={qd:<3d} "
                    f"tp={r['throughput']} MB/s  iops={r['iops']}  {tag}")
            print(line)
            lines.append(line)
            if r["timeout"]:
                # Capture one full diagnostic block for the first failure.
                lines.append("  --- raw (first timeout) ---")
                lines.append("  " + out.strip().replace("\n", "\n  "))
                # keep going; later QDs may still inform.
    with open("qd_sweep_result.txt", "w") as f:
        f.write("\n".join(lines) + "\n")
    bus.close()
    print("\nwrote qd_sweep_result.txt")


if __name__ == "__main__":
    main()
