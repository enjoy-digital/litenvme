#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import time
import argparse

from litex import RemoteClient

from nvme_host import (
    NVMeHost,
    nvme_cmd_read,
    nvme_cmd_write,
    nvme_cmd_create_iocq,
    nvme_cmd_create_iosq,
    nvme_cmd_identify_ns_list,
    nvme_cmd_set_features_num_queues,
    cqe_ok,
    cap_decode,
)

# Constants  ---------------------------------------------------------------------------------------

CFG_CTRL_START = 0
CFG_CTRL_WE    = 1
CFG_STAT_DONE  = 0
CFG_STAT_ERR   = 1
LINK_STATUS_UP = 0

# CFG helpers --------------------------------------------------------------------------------------

def cfg_bdf_pack(bus, dev, fn, reg, ext=0):
    v  = (bus & 0xff) << 0
    v |= (dev & 0x1f) << 8
    v |= (fn  & 0x07) << 13
    v |= (reg & 0x3f) << 16
    v |= (ext & 0x07) << 22
    return v

def cfg_rd0(bus, b, d, f, reg_dword, ext=0, timeout_ms=100):
    for _ in range(2):
        bus.regs.cfg_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
        bus.regs.cfg_cfg_ctrl.write(1 << CFG_CTRL_START)
        bus.regs.cfg_cfg_ctrl.write(0)

        deadline = time.time() + (timeout_ms / 1000.0)
        while True:
            stat = bus.regs.cfg_cfg_stat.read()
            done = (stat >> CFG_STAT_DONE) & 1
            err  = (stat >> CFG_STAT_ERR) & 1
            if done:
                return bus.regs.cfg_cfg_rdata.read(), err
            if time.time() > deadline:
                break
        time.sleep(0.01)
    raise TimeoutError("CFG read timeout (done=0).")

def cfg_wr0(bus, b, d, f, reg_dword, wdata, ext=0, timeout_ms=100):
    for _ in range(2):
        bus.regs.cfg_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
        bus.regs.cfg_cfg_wdata.write(wdata & 0xffffffff)
        bus.regs.cfg_cfg_ctrl.write((1 << CFG_CTRL_START) | (1 << CFG_CTRL_WE))
        bus.regs.cfg_cfg_ctrl.write(0)

        deadline = time.time() + (timeout_ms / 1000.0)
        while True:
            stat = bus.regs.cfg_cfg_stat.read()
            done = (stat >> CFG_STAT_DONE) & 1
            err  = (stat >> CFG_STAT_ERR) & 1
            if done:
                return err
            if time.time() > deadline:
                break
        time.sleep(0.01)
    raise TimeoutError("CFG write timeout (done=0).")

def cfg_set_command(bus, b, d, f, set_mem=True, set_bme=True, timeout_ms=100):
    try:
        v, err = cfg_rd0(bus, b, d, f, 1, timeout_ms=timeout_ms)
    except TimeoutError:
        print("WARN: CFG access timed out; leaving Command register unchanged.")
        return False

    if err:
        print("WARN: CFG read CMD/STS returned err=1, cannot update.")
        return False

    cmd = v & 0xffff
    new_cmd = cmd
    if set_mem:
        new_cmd |= (1 << 1)
    if set_bme:
        new_cmd |= (1 << 2)

    if new_cmd == cmd:
        return True

    new_v = (v & 0xffff0000) | new_cmd
    try:
        werr = cfg_wr0(bus, b, d, f, 1, new_v, timeout_ms=timeout_ms)
    except TimeoutError:
        print("WARN: CFG write timed out; Command register not updated.")
        return False
    if werr:
        print("WARN: CFG write CMD returned err=1.")
        return False
    return True

def discover_bar0(bus, b, d, f, timeout_ms=100):
    bar0, e0 = cfg_rd0(bus, b, d, f, 4, timeout_ms=timeout_ms)
    if e0:
        raise RuntimeError("CFG read BAR0 failed (err=1).")
    if bar0 & 1:
        raise RuntimeError("BAR0 is I/O, unexpected for NVMe.")
    bar_type = (bar0 >> 1) & 0x3
    is_64 = (bar_type == 0x2)

    base_lo = bar0 & 0xfffffff0
    if is_64:
        bar1, e1 = cfg_rd0(bus, b, d, f, 5, timeout_ms=timeout_ms)
        if e1:
            raise RuntimeError("CFG read BAR1 failed (err=1).")
        base = ((bar1 & 0xffffffff) << 32) | base_lo
    else:
        base = base_lo
    return base, is_64, bar0

def wait_link_up(bus, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while True:
        v = bus.regs.pcie_phy_phy_link_status.read()
        link = (v >> LINK_STATUS_UP) & 1
        if link:
            return True
        if time.time() > deadline:
            return False
        time.sleep(0.05)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteNVMe IO helper (read/write with optional setup).")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",    default="1234",    help="Host bind port.")
    parser.add_argument("--wait-link", action="store_true", help="Wait for PCIe link up before CFG/MMIO.")

    parser.add_argument("--b", default="0", help="Bus number.")
    parser.add_argument("--d", default="0", help="Device number.")
    parser.add_argument("--f", default="0", help="Function number.")

    parser.add_argument("--setup", action="store_true", help="Do full setup (CFG/MEM/BME + Admin + IO queues).")
    parser.add_argument("--bar0", default=None, help="BAR0 base address (hex). If omitted, discover from config space.")
    parser.add_argument("--cfg-timeout-ms", default="200", help="CFG transaction timeout (ms).")
    parser.add_argument("--timeout-ms", default="200", help="MMIO transaction timeout (ms).")

    parser.add_argument("--read",  action="store_true", help="Submit a Read command.")
    parser.add_argument("--write", action="store_true", help="Submit a Write command.")
    parser.add_argument("--write-verify", action="store_true", help="Read back and compare first 64 dwords.")

    parser.add_argument("--nsid", default="1", help="Namespace ID.")
    parser.add_argument("--slba", default="0", help="Start LBA.")
    parser.add_argument("--nlb",  default="1", help="Number of LBAs (start with 1).")

    parser.add_argument("--hostmem-base", default="0x10000000", help="Hostmem window base.")
    parser.add_argument("--asq-addr", default="0x10000000", help="ASQ base (4K aligned).")
    parser.add_argument("--acq-addr", default="0x10001000", help="ACQ base (4K aligned).")
    parser.add_argument("--q-entries", default="16", help="Admin queue entries (>=2).")

    parser.add_argument("--io-cq-addr", default="0x10003000", help="IO CQ1 base (4K aligned).")
    parser.add_argument("--io-sq-addr", default="0x10004000", help="IO SQ1 base (4K aligned).")
    parser.add_argument("--io-q-entries", default="4", help="IO queue entries (>=2).")

    parser.add_argument("--rd-buf", default="0x10005000", help="Read buffer PRP1 (4K aligned).")
    parser.add_argument("--wr-buf", default="0x10006000", help="Write buffer PRP1 (4K aligned).")

    args = parser.parse_args()

    if not (args.read or args.write):
        raise ValueError("Select at least one of --read or --write.")

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)
    timeout_ms = int(args.timeout_ms, 0)
    cfg_timeout_ms = int(args.cfg_timeout_ms, 0)

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    if args.bar0 is not None:
        bar0 = int(args.bar0, 0)
    else:
        bar0, is_64, bar0_raw = discover_bar0(bus, b, d, f, timeout_ms=cfg_timeout_ms)
        print(f"Discovered BAR0: 0x{bar0:016x} (64b={is_64}) raw=0x{bar0_raw:08x}")

    if bar0 == 0:
        raise RuntimeError("BAR0 base is 0. Assign BAR0 in config space before running.")

    hostmem_base = int(args.hostmem_base, 0)
    host = NVMeHost(bus, bar0, hostmem_base=hostmem_base, timeout_ms=timeout_ms)

    asq_addr  = int(args.asq_addr, 0)
    acq_addr  = int(args.acq_addr, 0)
    q_entries = int(args.q_entries, 0)

    io_cq_addr = int(args.io_cq_addr, 0)
    io_sq_addr = int(args.io_sq_addr, 0)
    io_q_entries = int(args.io_q_entries, 0)

    rd_buf = int(args.rd_buf, 0)
    wr_buf = int(args.wr_buf, 0)
    nsid   = int(args.nsid, 0)
    slba   = int(args.slba, 0)
    nlb    = int(args.nlb, 0)

    if args.setup:
        cfg_set_command(bus, b, d, f, set_mem=True, set_bme=True, timeout_ms=cfg_timeout_ms)

        host.admin_init(asq_addr, acq_addr, q_entries=q_entries, timeout_ms=timeout_ms)
        host.admin_setup(asq_addr, acq_addr, q_entries)

        cap = host.read_core()["cap"]
        capd = cap_decode(cap)
        max_entries = capd["mqes"] + 1
        if io_q_entries > max_entries:
            print(f"WARNING: Requested {io_q_entries} entries exceeds CAP.MQES+1={max_entries}. Clamping.")
            io_q_entries = max_entries

        cmd = nvme_cmd_set_features_num_queues(cid=0x20, nsqr=1, ncqr=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Set Features (Number of Queues) failed.")

        cmd = nvme_cmd_create_iocq(cid=0x21, prp1_cq=io_cq_addr, qid=1,
                                   qsize_minus1=io_q_entries-1, iv=0, ien=0, pc=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Create IO CQ failed.")

        cmd = nvme_cmd_create_iosq(cid=0x22, prp1_sq=io_sq_addr, qid=1,
                                   qsize_minus1=io_q_entries-1, cqid=1, qprio=0, pc=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Create IO SQ failed.")

        ns_buf = hostmem_base + 0x6000
        host.hostmem_fill(ns_buf, 0x1000, value=0)
        cmd = nvme_cmd_identify_ns_list(cid=0x30, prp1=ns_buf)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Identify NS List failed.")

        ns_list = host.hostmem_read_dwords(ns_buf, 16)
        active_nsid = 0
        for x in ns_list:
            if x != 0:
                active_nsid = x
                break
        if active_nsid != 0 and active_nsid != nsid:
            print(f"Using NSID={active_nsid} instead of NSID={nsid}.")
            nsid = active_nsid

    host.io_setup(io_sq_addr, io_cq_addr, io_q_entries)

    if args.write:
        pat = 0xA5A5A5A5
        host.hostmem_fill(wr_buf, 0x1000, value=pat)
        cmd = nvme_cmd_write(cid=0x40, nsid=nsid, prp1_data=wr_buf, slba=slba, nlb_minus1=nlb-1)
        cqe = host.io_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Write failed.")
        print("Write OK.")

    if args.read or args.write_verify:
        pat = 0xA5A5A5A5
        host.hostmem_fill(rd_buf, 0x1000, value=pat)
        before = host.hostmem_read_dwords(rd_buf, 64)

        cmd = nvme_cmd_read(cid=0x41, nsid=nsid, prp1_data=rd_buf, slba=slba, nlb_minus1=nlb-1)
        cqe = host.io_submit(cmd, timeout_s=2.0)
        if cqe is None or not cqe_ok(cqe):
            raise RuntimeError("Read failed.")
        print("Read OK.")

        after = host.hostmem_read_dwords(rd_buf, 64)
        changed = sum(1 for i in range(64) if after[i] != before[i])
        still_pat = sum(1 for i in range(64) if after[i] == pat)
        all_zero  = sum(1 for i in range(64) if after[i] == 0)

        print(f"Read buffer changed dwords: {changed}/64")
        print(f"After read: {still_pat}/64 still pattern, {all_zero}/64 are zero")

        if args.write and args.write_verify:
            wr_before = host.hostmem_read_dwords(wr_buf, 64)
            wr_after  = host.hostmem_read_dwords(rd_buf, 64)
            mism = sum(1 for i in range(64) if wr_before[i] != wr_after[i])
            print(f"Write verify: mismatched dwords {mism}/64")

    bus.close()


if __name__ == "__main__":
    main()
