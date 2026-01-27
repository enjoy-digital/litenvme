#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import struct
import time
import argparse

from litex import RemoteClient

from nvme_host import (
    NVMeHost,
    nvme_cmd_identify_controller,
    nvme_cmd_create_iocq,
    nvme_cmd_create_iosq,
    nvme_cmd_read,
    nvme_cmd_write,
    nvme_cmd_identify_ns_list,
    nvme_cmd_set_features_num_queues,
    cqe_ok,
    cap_decode,
    vs_decode,
    csts_rdy,
    csts_cfs,
    csts_shst,
    cc_en,
    cap_to_ms,
    doorbell_stride_bytes,
    nvme_db_addr,
)

# Constants  ---------------------------------------------------------------------------------------

CFG_CTRL_START = 0
CFG_CTRL_WE    = 1
CFG_STAT_DONE  = 0
CFG_STAT_ERR   = 1
LINK_STATUS_UP = 0

# Helpers ------------------------------------------------------------------------------------------

def _bit(v, n):
    return (v >> n) & 1

def _bits(v, lo, hi):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)

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

# NVMe Identify/CQE decoding ----------------------------------------------------------------------

def _dwords_to_bytes_le(dws):
    return b"".join(struct.pack("<I", x & 0xffffffff) for x in dws)

def _u16(b, off): return struct.unpack_from("<H", b, off)[0]
def _u32(b, off): return struct.unpack_from("<I", b, off)[0]

def _get_ascii(b, off, n):
    s = b[off:off+n].decode("ascii", errors="replace")
    return s.rstrip(" \x00")

def _ver_to_str(ver):
    mjr = (ver >> 16) & 0xffff
    mnr = (ver >> 8) & 0xff
    ter = ver & 0xff
    return f"{mjr}.{mnr}.{ter}"

def decode_identify_controller_from_dwords(dws):
    b = _dwords_to_bytes_le(dws)

    vid   = _u16(b, 0x00)
    ssvid = _u16(b, 0x02)
    sn    = _get_ascii(b, 0x04, 20)
    mn    = _get_ascii(b, 0x18, 40)
    fr    = _get_ascii(b, 0x40, 8)

    rab  = b[0x48]
    oui  = b[0x49:0x4c]  # 3 bytes
    ieee_oui = f"{oui[0]:02x}-{oui[1]:02x}-{oui[2]:02x}"

    cmic   = b[0x4c]
    mdts   = b[0x4d]
    cntlid = _u16(b, 0x4e)
    ver    = _u32(b, 0x50)
    oaes   = _u32(b, 0x58)

    return {
        "vid": vid,
        "ssvid": ssvid,
        "sn": sn,
        "mn": mn,
        "fr": fr,
        "rab": rab,
        "ieee_oui": ieee_oui,
        "cmic": cmic,
        "mdts": mdts,
        "cntlid": cntlid,
        "ver": ver,
        "ver_str": _ver_to_str(ver),
        "oaes": oaes,
    }

def pretty_print_identify_controller(info):
    print("Identify Controller (decoded):")
    print(f"  VID      : 0x{info['vid']:04x}")
    print(f"  SSVID    : 0x{info['ssvid']:04x}")
    print(f"  SN       : {info['sn']!r}")
    print(f"  MN       : {info['mn']!r}")
    print(f"  FR       : {info['fr']!r}")
    print(f"  VER      : {info['ver_str']} (0x{info['ver']:08x})")
    print(f"  CNTLID   : 0x{info['cntlid']:04x}")
    print(f"  IEEE OUI : {info['ieee_oui']}")
    print(f"  RAB      : {info['rab']}")
    print(f"  CMIC     : 0x{info['cmic']:02x}")
    print(f"  MDTS     : {info['mdts']}")
    print(f"  OAES     : 0x{info['oaes']:08x}")

def pretty_print_cqe(cqe, name="CQE"):
    print(f"{name} decoded CQE:")
    print(f"  SQ Head : {cqe['sq_head']}")
    print(f"  SQ ID   : {cqe['sq_id']}")
    print(f"  CID     : {cqe['cid']}")
    print(f"  Status  : P={cqe['p']} SCT={cqe['sct']} SC={cqe['sc']} M={cqe['m']} DNR={cqe['dnr']}")
    print(f"  DW0/DW1 : 0x{cqe['dw0']:08x} 0x{cqe['dw1']:08x}")

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

def cfg_decode_command(cmd):
    return {
        "io":     _bit(cmd, 0),
        "mem":    _bit(cmd, 1),
        "bme":    _bit(cmd, 2),
        "intdis": _bit(cmd, 10),
    }

def cfg_check_command(bus, b, d, f, timeout_ms=100):
    v, err = cfg_rd0(bus, b, d, f, 1, timeout_ms=timeout_ms)  # 0x04
    if err:
        print("WARN: CFG read CMD/STS returned err=1.")
        return None

    cmd = v & 0xffff
    sts = (v >> 16) & 0xffff
    cd  = cfg_decode_command(cmd)
    print(f"CFG CMD/STS   = 0x{cmd:04x} / 0x{sts:04x} (MEM={cd['mem']} BME={cd['bme']} INTDIS={cd['intdis']})")

    if cd["mem"] == 0:
        print("WARN: Command.MEM=0 (MMIO may be blocked).")
    if cd["bme"] == 0:
        print("NOTE: Command.BME=0 (DMA will be blocked; Identify needs BME=1).")

    return {"cmd": cmd, "sts": sts, "bits": cd}

def cfg_set_command(bus, b, d, f, set_mem=True, set_bme=True, timeout_ms=100):
    try:
        v, err = cfg_rd0(bus, b, d, f, 1, timeout_ms=timeout_ms)  # CMD/STS @0x04
    except TimeoutError:
        print("WARN: CFG access timed out; leaving Command register unchanged.")
        return False

    if err:
        print("WARN: CFG read CMD/STS returned err=1, cannot update.")
        return False

    cmd = v & 0xffff
    new_cmd = cmd
    if set_mem:
        new_cmd |= (1 << 1)  # MEM
    if set_bme:
        new_cmd |= (1 << 2)  # BME

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

    try:
        v2, err2 = cfg_rd0(bus, b, d, f, 1, timeout_ms=timeout_ms)
    except TimeoutError:
        print("WARN: CFG readback timed out.")
        return False

    if err2:
        print("WARN: CFG readback CMD returned err=1.")
        return False
    cmd2 = v2 & 0xffff
    print(f"CFG CMD updated: 0x{cmd:04x} -> 0x{cmd2:04x}")
    return True

# NVMe registers -----------------------------------------------------------------------------------

NVME_CAP   = 0x0000  # 64
NVME_VS    = 0x0008  # 32
NVME_INTMS = 0x000C  # 32
NVME_INTMC = 0x0010  # 32
NVME_CC    = 0x0014  # 32
NVME_CSTS  = 0x001C  # 32
NVME_NSSR  = 0x0020  # 32 (optional)
NVME_AQA   = 0x0024  # 32
NVME_ASQ   = 0x0028  # 64
NVME_ACQ   = 0x0030  # 64

# NVMe actions -------------------------------------------------------------------------------------

def nvme_read_core(host):
    info = host.read_core()
    cap = info["cap"]
    vs = info["vs"]

    capd = cap_decode(cap)
    vsd  = vs_decode(vs)

    print(f"NVMe BAR0      0x{host.bar0:016x}")
    print(f"CAP            0x{cap:016x}")
    print(f"  MQES         {capd['mqes']} (=> max {capd['mqes'] + 1} entries)")
    print(f"  CQR          {capd['cqr']}")
    print(f"  AMS          0x{capd['ams']:x}")
    print(f"  TO           {capd['to']} (=> {cap_to_ms(capd['to'])} ms)")
    print(f"  DSTRD        {capd['dstrd']} (=> {doorbell_stride_bytes(capd['dstrd'])} bytes)")
    print(f"  NSSRS        {capd['nssrs']}")
    print(f"  CSS          0x{capd['css']:x}")
    print(f"  MPSMIN/MAX   {capd['mpsmin']} / {capd['mpsmax']} (=> {1<<(12+capd['mpsmin'])}..{1<<(12+capd['mpsmax'])} bytes)")
    print(f"VS             {vsd['major']}.{vsd['minor']}.{vsd['ter']} (0x{vs:08x})")
    print(f"INTMS          0x{info['intms']:08x}")
    print(f"INTMC          0x{info['intmc']:08x}")
    print(f"CC             0x{info['cc']:08x} (EN={cc_en(info['cc'])})")
    print(f"CSTS           0x{info['csts']:08x} (RDY={csts_rdy(info['csts'])} CFS={csts_cfs(info['csts'])} SHST=0x{csts_shst(info['csts']):x})")
    print(f"AQA            0x{info['aqa']:08x}")
    print(f"ASQ            0x{info['asq']:016x}")
    print(f"ACQ            0x{info['acq']:016x}")

    if info["errs"]:
        print("NOTE: Some MMIO ops returned err=1 (UR/CA/etc). Check MEM/BME, BAR assignment, and offsets.")

    return info

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteNVMe progressive tester (MMIO-first, CSR-driven) + Identify via BRAM hostmem.")
    parser.add_argument("--csr-csv",   default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",      default="1234",    help="Host bind port.")
    parser.add_argument("--wait-link", action="store_true", help="Wait for PCIe link up before MMIO/CFG.")

    parser.add_argument("--b", default="0", help="Bus number.")
    parser.add_argument("--d", default="0", help="Device number.")
    parser.add_argument("--f", default="0", help="Function number.")

    parser.add_argument("--bar0", default=None, help="BAR0 base address (hex). If omitted, read BAR0/BAR1 from config space.")

    parser.add_argument("--timeout-ms", default="200", help="MMIO transaction timeout (ms).")
    parser.add_argument("--cfg-timeout-ms", default="200", help="CFG transaction timeout (ms).")

    # Actions.
    parser.add_argument("--info",        action="store_true", help="Read/decode CAP/VS/INTMS/INTMC/CC/CSTS/AQA/ASQ/ACQ.")
    parser.add_argument("--dump",        default=None, help="Dump first N bytes of BAR0 regs (hex/dec).")
    parser.add_argument("--disable",     action="store_true", help="Clear CC.EN and wait CSTS.RDY=0.")
    parser.add_argument("--mmio-check",  action="store_true", help="Run safe MMIO tests that do not require DMA.")
    parser.add_argument("--doorbells",   action="store_true", help="Read a few SQ/CQ doorbells (read-only sanity).")
    parser.add_argument("--max-q",       default="4", help="Max queue id to probe for --doorbells.")

    parser.add_argument("--identify", action="store_true", help="Send Admin Identify (controller) via BRAM hostmem.")

    parser.add_argument("--read", action="store_true", help="Create IO queues (CQ1/SQ1) and do a first Read.")
    parser.add_argument("--write", action="store_true", help="Create IO queues (CQ1/SQ1) and do a first Write.")
    parser.add_argument("--io-cq-addr", default="0x10003000", help="IO CQ1 base (4K aligned).")
    parser.add_argument("--io-sq-addr", default="0x10004000", help="IO SQ1 base (4K aligned).")
    parser.add_argument("--rd-buf",     default="0x10005000", help="Read buffer PRP1 (4K aligned).")
    parser.add_argument("--wr-buf",     default="0x10006000", help="Write buffer PRP1 (4K aligned).")
    parser.add_argument("--write-verify", action="store_true", help="Read back to verify write data.")
    parser.add_argument("--nsid",       default="1", help="Namespace ID.")
    parser.add_argument("--slba",       default="0", help="Start LBA.")
    parser.add_argument("--nlb",        default="1", help="Number of LBAs (start with 1).")
    parser.add_argument("--io-q-entries", default="2", help="IO queue entries (>=2, start with 2).")

    parser.add_argument("--hostmem-base", default="0x10000000", help="Hostmem window base (must match gateware).")
    parser.add_argument("--asq-addr", default="0x10000000", help="ASQ base (4K aligned).")
    parser.add_argument("--acq-addr", default="0x10001000", help="ACQ base (4K aligned).")
    parser.add_argument("--id-buf",   default="0x10002000", help="Identify buffer (4K aligned).")
    parser.add_argument("--cid",      default="1", help="Command ID.")
    parser.add_argument("--q-entries", default="2", help="Admin queue entries (>=2).")

    args = parser.parse_args()

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)
    timeout_ms = int(args.timeout_ms, 0)
    cfg_timeout_ms = int(args.cfg_timeout_ms, 0)
    max_q = int(args.max_q, 0)

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    if not (args.info or args.dump or args.disable or args.mmio_check or args.doorbells or args.identify or args.read or args.write):
        args.info = True
        args.mmio_check = True

    cfg_check_command(bus, b, d, f, timeout_ms=cfg_timeout_ms)

    if args.bar0 is not None:
        bar0 = int(args.bar0, 0)
    else:
        bar0, is_64, bar0_raw = discover_bar0(bus, b, d, f, timeout_ms=cfg_timeout_ms)
        print(f"Discovered BAR0: 0x{bar0:016x} (64b={is_64}) raw=0x{bar0_raw:08x}")

    if bar0 == 0:
        raise RuntimeError("BAR0 base is 0. Assign BAR0 in config space before running MMIO tests.")

    hostmem_base = int(args.hostmem_base, 0)
    host = NVMeHost(bus, bar0, hostmem_base=hostmem_base, timeout_ms=timeout_ms)

    info = None
    if args.info or args.disable or args.mmio_check or args.doorbells or args.identify or args.read or args.write:
        info = nvme_read_core(host)

    if args.doorbells:
        for q in range(0, max_q + 1):
            for is_cq in [False, True]:
                db = nvme_db_addr(bar0, info["cap"], qid=q, is_cq=is_cq)
                v, err = host.mmio_rd32(db, timeout_ms=timeout_ms)
                which = "CQ" if is_cq else "SQ"
                print(f"DB {which}{q} @0x{db:08x}: 0x{v:08x} err={err}")

    asq_addr     = int(args.asq_addr, 0)
    acq_addr     = int(args.acq_addr, 0)
    cid          = int(args.cid, 0)
    q_entries    = int(args.q_entries, 0)

    if args.identify:
        id_buf = int(args.id_buf, 0)

        host.hostmem_fill(acq_addr, 16, value=0)
        host.hostmem_fill(id_buf,  0x100, value=0)

        cfg_set_command(bus, b, d, f, set_mem=True, set_bme=True, timeout_ms=cfg_timeout_ms)

        ok = host.admin_init(asq_addr, acq_addr, q_entries=q_entries, timeout_ms=timeout_ms)
        host.admin_setup(asq_addr, acq_addr, q_entries)

        csts, _ = host.mmio_rd32(bar0 + NVME_CSTS, timeout_ms=timeout_ms)
        print(f"Admin init: RDY={'1' if csts_rdy(csts) else '0'} CFS={csts_cfs(csts)} (poll_ok={'1' if ok else '0'})")

        cmd = nvme_cmd_identify_controller(cid=cid, prp1=id_buf, nsid=0)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        print("ACQ[0] raw:")
        if cqe is None:
            print("  (no completion observed)")
        else:
            print(f"  {cqe['dw0']:08x} {cqe['dw1']:08x} {cqe['dw2']:08x} {cqe['dw3']:08x}")
            pretty_print_cqe(cqe, name="ACQ[0]")

        print("Identify buffer (first 0x100):")
        host.hostmem_dump(id_buf, 0x100)

        id_dws = host.hostmem_read_dwords(id_buf, 0x100//4)
        id_info = decode_identify_controller_from_dwords(id_dws)
        pretty_print_identify_controller(id_info)

    if args.read or args.write:
        io_cq_addr = int(args.io_cq_addr, 0)
        io_sq_addr = int(args.io_sq_addr, 0)
        rd_buf     = int(args.rd_buf, 0)
        wr_buf     = int(args.wr_buf, 0)
        nsid       = int(args.nsid, 0)
        slba       = int(args.slba, 0)
        nlb        = int(args.nlb, 0)
        io_q_entries = int(args.io_q_entries, 0)
        if io_q_entries < 2:
            raise ValueError("--io-q-entries must be >=2")

        # Validate queue size against controller capabilities
        capd = cap_decode(info["cap"])
        max_entries = capd["mqes"] + 1
        if io_q_entries > max_entries:
            print(f"WARNING: Requested {io_q_entries} entries exceeds CAP.MQES+1={max_entries}. Clamping.")
            io_q_entries = max_entries

        for name, a in [("io_cq_addr", io_cq_addr), ("io_sq_addr", io_sq_addr), ("rd_buf", rd_buf), ("wr_buf", wr_buf)]:
            if (a & 0xfff) != 0:
                raise ValueError(f"{name} must be 4K-aligned, got 0x{a:x}")

        host.hostmem_fill(io_cq_addr, 0x1000, value=0)
        host.hostmem_fill(io_sq_addr, 0x1000, value=0)
        host.hostmem_fill(rd_buf,     0x1000, value=0)
        host.hostmem_fill(wr_buf,     0x1000, value=0)

        # Scratch 4K page for Identify NS list.
        ns_buf = hostmem_base + 0x6000
        host.hostmem_fill(ns_buf, 0x1000, value=0)

        # Prime MMIO engine.
        host.mmio_rd32_safe(bar0 + NVME_VS, timeout_ms=max(timeout_ms, 500), retries=8)

        # Bring controller up.
        ok = False
        for attempt in range(3):
            try:
                ok = host.admin_init(asq_addr, acq_addr, q_entries=q_entries, disable=False,
                                     timeout_ms=max(timeout_ms, 500))
                break
            except TimeoutError:
                time.sleep(0.05)

        if not ok:
            print("Admin init failed (MMIO timeouts).")
            bus.close()
            return

        host.admin_setup(asq_addr, acq_addr, q_entries)

        csts, _ = host.mmio_rd32_safe(bar0 + NVME_CSTS, timeout_ms=max(timeout_ms, 500), retries=8)
        print(f"Admin init (for read): RDY={'1' if csts_rdy(csts) else '0'} CFS={csts_cfs(csts)} (poll_ok={'1' if ok else '0'})")
        if not ok:
            bus.close()
            return

        # 1) Set Features: Number of Queues (request 1 SQ + 1 CQ).
        cmd = nvme_cmd_set_features_num_queues(cid=cid + 20, nsqr=1, ncqr=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        print("Set Features (Number of Queues):")
        if cqe is None:
            print("  timeout")
            bus.close()
            return
        pretty_print_cqe(cqe, name="ACQ")
        if not cqe_ok(cqe):
            print("  ERROR: Set Features (Number of Queues) failed.")
            bus.close()
            return

        # 2) Create IO CQ1.
        cmd = nvme_cmd_create_iocq(cid=cid + 1, prp1_cq=io_cq_addr, qid=1,
                                   qsize_minus1=io_q_entries-1, iv=0, ien=0, pc=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        print("Create IO CQ1:")
        if cqe is None:
            print("  timeout")
            bus.close()
            return
        pretty_print_cqe(cqe, name="ACQ")
        if not cqe_ok(cqe):
            print("  ERROR: Create IO CQ failed.")
            bus.close()
            return

        # 3) Create IO SQ1.
        cmd = nvme_cmd_create_iosq(cid=cid + 2, prp1_sq=io_sq_addr, qid=1,
                                   qsize_minus1=io_q_entries-1, cqid=1, qprio=0, pc=1)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        print("Create IO SQ1:")
        if cqe is None:
            print("  timeout")
            bus.close()
            return
        pretty_print_cqe(cqe, name="ACQ")
        if not cqe_ok(cqe):
            print("  ERROR: Create IO SQ failed.")
            bus.close()
            return

        # 4) Identify Namespace List (CNS=2) to select NSID.
        host.hostmem_fill(ns_buf, 0x1000, value=0)
        cmd = nvme_cmd_identify_ns_list(cid=cid + 10, prp1=ns_buf)
        cqe = host.admin_submit(cmd, timeout_s=2.0)
        print("Identify NS List:")
        if cqe is None:
            print("  timeout")
            bus.close()
            return
        pretty_print_cqe(cqe, name="ACQ")
        if not cqe_ok(cqe):
            print("  ERROR: Identify NS List failed.")
            bus.close()
            return

        ns_list = host.hostmem_read_dwords(ns_buf, 16)
        print("NSID list (first 16 dwords):")
        print("  " + " ".join(f"{x:08x}" for x in ns_list))

        # ns_list already contains the first 16 NSIDs (enough for most drives).
        active_nsid = 0
        for x in ns_list:
            if x != 0:
                active_nsid = x
                break

        if active_nsid == 0:
            # If somehow the first 16 are all zero, scan a bit more but not 1024.
            more = host.hostmem_read_dwords(ns_buf + 16*4, 64)  # +64 dwords
            for x in more:
                if x != 0:
                    active_nsid = x
                    break

        if active_nsid == 0:
            print("  ERROR: no active namespace reported in the first 80 entries.")
            bus.close()
            return

        if active_nsid != nsid:
            print(f"Using NSID={active_nsid} instead of NSID={nsid}.")
            nsid = active_nsid


        host.io_setup(io_sq_addr, io_cq_addr, io_q_entries)

        if args.write:
            pat = 0xA5A5A5A5
            host.hostmem_fill(wr_buf, 0x1000, value=pat)

            print(f"Submitting WRITE: NSID={nsid} SLBA={slba} NLB={nlb} PRP1=0x{wr_buf:08x}")
            write_cmd = nvme_cmd_write(cid=cid + 3, nsid=nsid, prp1_data=wr_buf, slba=slba, nlb_minus1=nlb-1)
            print("WRITE SQE:", " ".join(f"{w:08x}" for w in write_cmd))

            cqe = host.io_submit(write_cmd, timeout_s=2.0)
            print("Write CQ1:")
            if cqe is None:
                print("  timeout")
                bus.close()
                return
            pretty_print_cqe(cqe, name="CQ1")
            if not cqe_ok(cqe):
                print("  ERROR: Write failed.")
                bus.close()
                return

        if args.read or args.write_verify:
            # Prefill destination buffer with a non-zero pattern so we can tell if the SSD wrote anything.
            pat = 0xA5A5A5A5
            host.hostmem_fill(rd_buf, 0x1000, value=pat)

            # Snapshot a small prefix (64 dwords = 256B) before the read.
            before = host.hostmem_read_dwords(rd_buf, 64)

            print(f"Submitting READ: NSID={nsid} SLBA={slba} NLB={nlb} PRP1=0x{rd_buf:08x}")
            read_cmd = nvme_cmd_read(cid=cid + 4, nsid=nsid, prp1_data=rd_buf, slba=slba, nlb_minus1=nlb-1)
            print("READ SQE:", " ".join(f"{w:08x}" for w in read_cmd))

            cqe = host.io_submit(read_cmd, timeout_s=2.0)
            print("Read CQ1:")
            if cqe is None:
                print("  timeout")
                bus.close()
                return
            pretty_print_cqe(cqe, name="CQ1")
            if not cqe_ok(cqe):
                print("  ERROR: Read failed.")
                bus.close()
                return

            # Snapshot after the read and report what changed.
            after = host.hostmem_read_dwords(rd_buf, 64)

            changed = sum(1 for i in range(64) if after[i] != before[i])
            still_pat = sum(1 for i in range(64) if after[i] == pat)
            all_zero  = sum(1 for i in range(64) if after[i] == 0)

            print(f"Read buffer changed dwords: {changed}/64")
            print(f"After read: {still_pat}/64 still pattern, {all_zero}/64 are zero")

            print("Read buffer (first 0x200):")
            host.hostmem_dump(rd_buf, 0x200)

            sig = host.hostmem_rd32(rd_buf + 0x1FC)
            sig16 = (sig >> 16) & 0xffff
            print(f"MBR sig16: 0x{sig16:04x} (expect aa55)")

            if args.write and args.write_verify:
                wr_before = host.hostmem_read_dwords(wr_buf, 64)
                wr_after  = host.hostmem_read_dwords(rd_buf, 64)
                mism = sum(1 for i in range(64) if wr_before[i] != wr_after[i])
                print(f"Write verify: mismatched dwords {mism}/64")


    bus.close()

if __name__ == "__main__":
    main()
