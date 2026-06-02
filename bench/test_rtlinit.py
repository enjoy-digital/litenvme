#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""Etherbone host test for the pure-RTL NVMe init sequencer (--with-rtl-init, CPU-less).

Pulses nvme_init_ctrl.start, polls nvme_init_status, then drives the RTL request generator to
read a few commands -- if errors=0, the hardware-initialized admin + IO queues work end to end
with no firmware/CPU. Usage: ./test_rtlinit.py [--csr-csv build/alibaba_xcku3p/csr.csv]
"""

import time
import argparse
from litex import RemoteClient

CLK = 125e6


def w64(lo, hi, v):
    lo.write(v & 0xffffffff); hi.write((v >> 32) & 0xffffffff)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csr-csv", default="build/alibaba_xcku3p/csr.csv")
    p.add_argument("--settle", default=3.0, type=float, help="Seconds to let the SSD boot before init.")
    args = p.parse_args()

    bus = RemoteClient(csr_csv=args.csr_csv); bus.open(); r = bus.regs

    ls = r.pcie_phy_phy_link_status.read()
    print(f"link_status=0x{ls:04x} up={ls & 1}")
    if not (ls & 1):
        print("ERROR: PCIe link down."); bus.close(); return 1

    print(f"waiting {args.settle}s for the SSD to boot...")
    time.sleep(args.settle)

    # --- Pure-RTL bring-up. ---
    r.nvme_init_ctrl.write(1)  # start (pulse).
    t0 = time.time()
    st = 0
    while time.time() - t0 < 15:
        st = r.nvme_init_status.read()
        if st & 0b011:  # done or error.
            break
    done, err, busy = st & 1, (st >> 1) & 1, (st >> 2) & 1
    fail = (st >> 8) & 0xff
    cmd = (st >> 24) & 0x3
    print(f"rtl-init: done={done} error={err} busy={busy} fail_code={fail} cmd={cmd}")
    if not done or err:
        names = {0: "ok", 1: "cfg probe (VID/DID) failed -- config path/timing",
                 2: "CAP not responding (BAR0/window)", 3: "CSTS.RDY timeout",
                 4: "admin CQE timeout (doorbell/SQE fetch)", 5: "admin CQE status error",
                 6: "cfg write err (probe ok)"}
        print(f"  -> {names.get(fail, '?')}")
        print(f"  last CQE dw3=0x{r.nvme_init_dbg_cqe3.read():08x}  "
              f"mmio=0x{r.nvme_init_dbg_mmio.read():08x}  cfg=0x{r.nvme_init_dbg_cfg.read():08x}")
        print("RTL init FAILED."); bus.close(); return 1
    print("RTL init OK (CPU-less bring-up).")

    # --- Prove the initialized queues work: read via the request generator. ---
    count, nlb, lba = 100, 16, 0x100000
    r.nvme_gen_op.write(0); r.nvme_gen_nsid.write(1)
    w64(r.nvme_gen_base_lba_lo, r.nvme_gen_base_lba_hi, lba)
    r.nvme_gen_nlb.write(nlb); r.nvme_gen_lba_step.write(nlb); r.nvme_gen_count.write(count)
    w64(r.nvme_gen_buf_base_lo, r.nvme_gen_buf_base_hi, 0x1001_0000)
    r.nvme_gen_buf_stride.write(0x1000); r.nvme_gen_qmod.write(64)
    r.nvme_gen_ctrl.write(1)
    t0 = time.time()
    while (r.nvme_gen_status.read() & 1) == 0 and time.time() - t0 < 15:
        pass
    cyc  = r.nvme_gen_cycles.read()
    comp = r.nvme_gen_completed.read()
    errs = r.nvme_gen_errors.read()
    mbps = (count * nlb * 512 * CLK / (cyc * 1e6)) if cyc else 0.0
    ok = (errs == 0) and (comp == count)
    print(f"io read: completed={comp}/{count} errors={errs} {mbps:.1f} MB/s -> {'PASS' if ok else 'FAIL'}")
    bus.close()
    return 0 if (done and not err and ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
