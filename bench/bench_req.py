#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import time

from litex import RemoteClient

from test_req import (
    REQ_OP_IDENTIFY,
    REQ_OP_READ,
    REQ_OP_WRITE,
    REQ_STATUS_ERROR,
    req_submit,
)


def _op_from_str(op):
    op = op.lower()
    if op == "read":
        return REQ_OP_READ
    if op == "write":
        return REQ_OP_WRITE
    if op == "identify":
        return REQ_OP_IDENTIFY
    raise ValueError(f"unknown op: {op}")


def _read_optional(regs, name):
    reg = getattr(regs, name, None)
    return reg.read() if reg is not None else None


def main():
    parser = argparse.ArgumentParser(description="LiteNVMe CSR request benchmark")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR CSV file")
    parser.add_argument("--host", default="localhost", help="Etherbone host")
    parser.add_argument("--port", default=1234, type=int, help="Etherbone port")
    parser.add_argument("--op", default="read", help="Operation: read/write/identify")
    parser.add_argument("--nsid", default=1, type=int, help="Namespace ID")
    parser.add_argument("--lba", default=0, type=lambda x: int(x, 0), help="Start LBA")
    parser.add_argument("--nlb", default=1, type=int, help="Number of blocks")
    parser.add_argument("--buf", default=0x10005000, type=lambda x: int(x, 0), help="PRP1 buffer address")
    parser.add_argument("--bar0", default=0xe0000000, type=lambda x: int(x, 0), help="BAR0 base address")
    parser.add_argument("--timeout", default=1.0, type=float, help="Per-request timeout (s)")
    parser.add_argument("--count", default=100, type=int, help="Measured request count")
    parser.add_argument("--warmup", default=1, type=int, help="Warmup request count")
    parser.add_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency in Hz")
    args = parser.parse_args()

    op = _op_from_str(args.op)

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    try:
        regs = bus.regs
        rd_before = _read_optional(regs, "hostmem_csr_dma_rd_count")
        wr_before = _read_optional(regs, "hostmem_csr_dma_wr_count")

        for _ in range(args.warmup):
            status, cqe, cycles, bytes_done = req_submit(
                bus, op=op, nsid=args.nsid, lba=args.lba, nlb=args.nlb,
                buf=args.buf, bar0=args.bar0, timeout_s=args.timeout,
            )
            if status & REQ_STATUS_ERROR:
                raise RuntimeError(f"warmup request failed: cqe=0x{cqe:08x}")

        total_cycles = 0
        total_bytes = 0
        t0 = time.perf_counter()
        for _ in range(args.count):
            status, cqe, cycles, bytes_done = req_submit(
                bus, op=op, nsid=args.nsid, lba=args.lba, nlb=args.nlb,
                buf=args.buf, bar0=args.bar0, timeout_s=args.timeout,
            )
            if status & REQ_STATUS_ERROR:
                raise RuntimeError(f"request failed: cqe=0x{cqe:08x}")
            if cycles is not None:
                total_cycles += cycles
            if bytes_done is not None:
                total_bytes += bytes_done
        elapsed_s = time.perf_counter() - t0

        rd_after = _read_optional(regs, "hostmem_csr_dma_rd_count")
        wr_after = _read_optional(regs, "hostmem_csr_dma_wr_count")

        avg_latency_ms = (elapsed_s * 1e3 / args.count) if args.count else 0.0
        payload_rate = ((total_bytes / 1e6) / elapsed_s) if elapsed_s > 0 else 0.0

        print(f"op={args.op} count={args.count} warmup={args.warmup} nlb={args.nlb}")
        print(f"host_time_total_s={elapsed_s:.6f}")
        print(f"host_avg_latency_ms={avg_latency_ms:.6f}")
        print(f"payload_bytes_total={total_bytes}")
        print(f"payload_rate_MBps={payload_rate:.3f}")

        if total_cycles:
            avg_cycles = total_cycles / args.count
            fw_latency_us = (avg_cycles / args.sys_clk_freq) * 1e6
            fw_rate = ((total_bytes / 1e6) / (total_cycles / args.sys_clk_freq))
            print(f"fw_cycles_total={total_cycles}")
            print(f"fw_cycles_avg={avg_cycles:.1f}")
            print(f"fw_avg_latency_us={fw_latency_us:.3f}")
            print(f"fw_payload_rate_MBps={fw_rate:.3f}")

        if rd_before is not None and rd_after is not None:
            print(f"hostmem_dma_rd_beats={rd_after - rd_before}")
        if wr_before is not None and wr_after is not None:
            print(f"hostmem_dma_wr_beats={wr_after - wr_before}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
