#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import time
import argparse

from litex import RemoteClient

# Constants ---------------------------------------------------------------------------------------

REQ_OP_READ     = 0
REQ_OP_WRITE    = 1
REQ_OP_IDENTIFY = 2

REQ_STATUS_BUSY  = 1 << 0
REQ_STATUS_DONE  = 1 << 1
REQ_STATUS_ERROR = 1 << 2

# Helpers -----------------------------------------------------------------------------------------

def _op_from_str(op):
    op = op.lower()
    if op == "read":
        return REQ_OP_READ
    if op == "write":
        return REQ_OP_WRITE
    if op == "identify":
        return REQ_OP_IDENTIFY
    raise ValueError(f"unknown op: {op}")


def req_submit(bus, op, nsid, lba, nlb, buf, bar0, timeout_s=1.0):
    regs = bus.regs

    # Program request.
    regs.nvme_req_req_op.write(op)
    regs.nvme_req_req_nsid.write(nsid)
    regs.nvme_req_req_lba_lo.write(lba & 0xffffffff)
    regs.nvme_req_req_lba_hi.write((lba >> 32) & 0xffffffff)
    regs.nvme_req_req_nlb.write(nlb)
    regs.nvme_req_req_buf_lo.write(buf & 0xffffffff)
    regs.nvme_req_req_buf_hi.write((buf >> 32) & 0xffffffff)
    regs.nvme_req_req_bar0_lo.write(bar0 & 0xffffffff)
    regs.nvme_req_req_bar0_hi.write((bar0 >> 32) & 0xffffffff)

    # Clear status and trigger.
    regs.nvme_req_req_status.write(0)
    regs.nvme_req_req_cqe_status.write(0)
    regs.nvme_req_req_ctrl.write(1)

    # Poll status.
    t0 = time.time()
    while True:
        status = regs.nvme_req_req_status.read()
        if status & REQ_STATUS_DONE:
            break
        if (time.time() - t0) > timeout_s:
            raise TimeoutError("request timed out")
        time.sleep(0.01)

    cqe_status = regs.nvme_req_req_cqe_status.read()
    return status, cqe_status


def main():
    parser = argparse.ArgumentParser(description="LiteNVMe CSR request test (firmware-driven)")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR CSV file")
    parser.add_argument("--host",    default="localhost", help="Etherbone host")
    parser.add_argument("--port",    default=1234, type=int, help="Etherbone port")
    parser.add_argument("--op",      default="read", help="Operation: read/write/identify")
    parser.add_argument("--nsid",    default=1, type=int, help="Namespace ID")
    parser.add_argument("--lba",     default=0, type=lambda x: int(x, 0), help="Start LBA")
    parser.add_argument("--nlb",     default=1, type=int, help="Number of blocks")
    parser.add_argument("--buf",     default=0x10005000, type=lambda x: int(x, 0), help="PRP1 buffer address")
    parser.add_argument("--bar0",    default=0xe0000000, type=lambda x: int(x, 0), help="BAR0 base address")
    parser.add_argument("--timeout", default=1.0, type=float, help="Timeout (s)")
    args = parser.parse_args()

    op = _op_from_str(args.op)

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    try:
        status, cqe = req_submit(
            bus,
            op=op,
            nsid=args.nsid,
            lba=args.lba,
            nlb=args.nlb,
            buf=args.buf,
            bar0=args.bar0,
            timeout_s=args.timeout,
        )
        err = "yes" if (status & REQ_STATUS_ERROR) else "no"
        print(f"done: status=0x{status:08x} err={err} cqe=0x{cqe:08x}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
