#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""Etherbone host test for the LiteNVMe engine + block streamer.

Drives, over Etherbone (RemoteClient / litex_server), the two engine front-ends of the test
SoC (bench/alibaba_xcku3p.py --with-io-engine --with-block-streamer):

  - throughput : the RTL request generator (front-end sel=0) — HW-measured cycles -> MB/s,
  - correctness: the block streamer + BIST (front-end sel=1) — write a per-beat counter to an
                 LBA range, read it back, and PASS when the BIST sees 0 mismatches.

Prerequisite: the firmware is booted and `nvme_setup` has been run on the board (controller
enabled, IO SQ/CQ created, engine programmed + enabled, init_done=1). The hw_block.sh
harness does this. Usage: ./test_block.py [--csr-csv build/alibaba_xcku3p/csr.csv]
"""

import time
import argparse

from litex import RemoteClient

CLK_FREQ = 125e6  # sys clock (256b @ 125 MHz).


def w64(lo, hi, v):
    lo.write(v & 0xffffffff)
    hi.write((v >> 32) & 0xffffffff)


def wait_done(read_status, done_bit=0, timeout=15.0):
    # `done` is registered (1 at idle, cleared within a sys cycle of `start` — far faster than
    # the Etherbone round-trip), so polling done==1 after a start is race-free here.
    t0 = time.time()
    while time.time() - t0 < timeout:
        if read_status() & (1 << done_bit):
            return True
    return False


def block_xfer(r, write, sector, count, bist):
    """Program + run one block-streamer transfer. `bist`: 0b01=wr_en, 0b10=rd_en, 0=off."""
    w64(r.nvme_block_sector_lo, r.nvme_block_sector_hi, sector)
    r.nvme_block_count.write(count)
    r.nvme_block_nsid.write(1)
    r.nvme_block_bist_control.write(bist)
    r.nvme_block_ctrl.write(0b11 if write else 0b01)  # start (| write).
    return wait_done(r.nvme_block_status.read)


def main():
    p = argparse.ArgumentParser(description="LiteNVMe Etherbone block test.")
    p.add_argument("--csr-csv", default="build/alibaba_xcku3p/csr.csv")
    p.add_argument("--lba",     default="0x100000", help="Start LBA.")
    p.add_argument("--nlb",     default=16, type=int, help="Sectors per command (<=16 => no PRP list).")
    p.add_argument("--count",   default=1000, type=int, help="Commands for the throughput run.")
    p.add_argument("--isolated", action="store_true",
                   help="Isolated write proof: write, scrub staging with a different LBA, read back.")
    p.add_argument("--lba-b",   default="0x300000", help="Scrub LBA for --isolated (must differ from --lba).")
    args = p.parse_args()
    lba = int(args.lba, 0)

    bus = RemoteClient(csr_csv=args.csr_csv)
    bus.open()
    r = bus.regs

    if not r.nvme_ctrl_init_done.read():
        print("ERROR: init_done=0 — run `nvme_setup` on the board first.")
        bus.close()
        return 1
    print("init_done=1")

    # --- Throughput: request generator (front-end sel=0). --------------------------------
    r.nvme_block_bist_sel.write(0)
    r.nvme_gen_op.write(0)  # read.
    r.nvme_gen_nsid.write(1)
    w64(r.nvme_gen_base_lba_lo, r.nvme_gen_base_lba_hi, lba)
    r.nvme_gen_nlb.write(args.nlb)
    r.nvme_gen_lba_step.write(args.nlb)
    r.nvme_gen_count.write(args.count)
    w64(r.nvme_gen_buf_base_lo, r.nvme_gen_buf_base_hi, 0x1001_0000)  # IO_BUF_BASE.
    r.nvme_gen_buf_stride.write(0x1000)
    r.nvme_gen_qmod.write(64)
    r.nvme_gen_ctrl.write(1)  # start.
    if not wait_done(r.nvme_gen_status.read):
        print("ERROR: request_gen run timed out.")
        bus.close()
        return 1
    cycles    = r.nvme_gen_cycles.read()
    completed = r.nvme_gen_completed.read()
    errors    = r.nvme_gen_errors.read()
    payload   = args.count * args.nlb * 512
    mbps      = (payload * CLK_FREQ / (cycles * 1e6)) if cycles else 0.0
    print(f"throughput(read {args.nlb*512} B x {args.count}): {mbps:8.1f} MB/s  "
          f"completed={completed} errors={errors} cycles={cycles}")

    # --- Correctness: block streamer + BIST (front-end sel=1). ---------------------------
    r.nvme_block_bist_sel.write(1)
    sector, count = lba, args.nlb
    exp_beats = count * 512 // 32

    # Write: BIST counter -> wr_sink -> staging -> SSD.
    wr_ok    = block_xfer(r, write=1, sector=sector, count=count, bist=0b01)
    wr_beats = r.nvme_block_bist_wr_beats.read()

    if args.isolated:
        # Scrub staging by reading a DIFFERENT LBA (checker on; mismatch vs the counter is
        # expected). Staging then holds LBA-B data, so the final read of `sector` can only
        # match the counter if the write truly persisted to disk -- not stale staging.
        lba_b = int(args.lba_b, 0)
        block_xfer(r, write=0, sector=lba_b, count=count, bist=0b10)
        print(f"scrub(LBA {lba_b:#x}): bist_errors={r.nvme_block_bist_errors.read()} (mismatch expected)")

    # Read back `sector` (errors reset on this start) -> BIST checker.
    rd_ok    = block_xfer(r, write=0, sector=sector, count=count, bist=0b10)
    rd_beats = r.nvme_block_bist_rd_beats.read()
    bist_err = r.nvme_block_bist_errors.read()
    strm_err = (r.nvme_block_status.read() >> 2) & 1

    ok = wr_ok and rd_ok and (bist_err == 0) and (strm_err == 0) and (rd_beats == exp_beats)
    label = "correctness(isolated)" if args.isolated else "correctness"
    print(f"{label}(LBA {sector:#x} x {count} sectors): wr_beats={wr_beats} rd_beats={rd_beats}"
          f"/{exp_beats} bist_errors={bist_err} streamer_error={strm_err} -> {'PASS' if ok else 'FAIL'}")

    bus.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
