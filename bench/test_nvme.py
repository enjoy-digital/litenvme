#!/usr/bin/env python3

# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import time
import argparse
import struct

from litex import RemoteClient

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

def decode_cqe(d0, d1, d2, d3):
    sq_head = d2 & 0xffff
    sq_id   = (d2 >> 16) & 0xffff

    cid = d3 & 0xffff
    sts = (d3 >> 16) & 0xffff

    p   = sts & 0x1
    sc  = (sts >> 1) & 0xff
    sct = (sts >> 9) & 0x7
    m   = (sts >> 14) & 0x1
    dnr = (sts >> 15) & 0x1

    return {
        "dw0": d0, "dw1": d1, "dw2": d2, "dw3": d3,
        "sq_head": sq_head, "sq_id": sq_id,
        "cid": cid,
        "p": p, "sc": sc, "sct": sct, "m": m, "dnr": dnr,
    }

def pretty_print_cqe(cqe, name="CQE"):
    print(f"{name} decoded CQE:")
    print(f"  SQ Head : {cqe['sq_head']}")
    print(f"  SQ ID   : {cqe['sq_id']}")
    print(f"  CID     : {cqe['cid']}")
    print(f"  Status  : P={cqe['p']} SCT={cqe['sct']} SC={cqe['sc']} M={cqe['m']} DNR={cqe['dnr']}")
    print(f"  DW0/DW1 : 0x{cqe['dw0']:08x} 0x{cqe['dw1']:08x}")

def cqe_ok(cqe):
    return (cqe["sct"] == 0) and (cqe["sc"] == 0)

# HostMem CSR helpers -----------------------------------------------------------------------------

def hostmem_set_adr(bus, dword_adr):
    bus.regs.hostmem_csr_csr_adr.write(dword_adr & 0xffffffff)

def hostmem_wr32(bus, addr, data, base=0x10000000):
    dword = (addr - base) >> 2
    hostmem_set_adr(bus, dword)
    bus.regs.hostmem_csr_csr_wdata.write(data & 0xffffffff)
    bus.regs.hostmem_csr_csr_we.write(1)
    bus.regs.hostmem_csr_csr_we.write(0)

def hostmem_rd32(bus, addr, base=0x10000000):
    dword = (addr - base) >> 2
    hostmem_set_adr(bus, dword)
    return bus.regs.hostmem_csr_csr_rdata.read()

def hostmem_fill(bus, addr, length, value=0, base=0x10000000):
    for off in range(0, length, 4):
        hostmem_wr32(bus, addr + off, value, base=base)

def hostmem_dump(bus, addr, length, base=0x10000000):
    for off in range(0, length, 4):
        v = hostmem_rd32(bus, addr + off, base=base)
        print(f"0x{off:04x}: 0x{v:08x}")

def hostmem_read_dwords(bus, addr, dword_count, base=0x10000000):
    return [hostmem_rd32(bus, addr + 4*i, base=base) for i in range(dword_count)]

def hostmem_wr_cmd(bus, asq_base, slot, cmd_dwords, hostmem_base=0x10000000):
    addr = asq_base + slot*64
    for i, w in enumerate(cmd_dwords):
        hostmem_wr32(bus, addr + i*4, w, base=hostmem_base)

def hostmem_rd_cqe_slot(bus, cq_base, slot, hostmem_base=0x10000000):
    addr = cq_base + slot*16
    d0 = hostmem_rd32(bus, addr + 0x0, base=hostmem_base)
    d1 = hostmem_rd32(bus, addr + 0x4, base=hostmem_base)
    d2 = hostmem_rd32(bus, addr + 0x8, base=hostmem_base)
    d3 = hostmem_rd32(bus, addr + 0xc, base=hostmem_base)
    return (d0, d1, d2, d3)

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

# MEM/MMIO helpers ---------------------------------------------------------------------------------

def mem_set_addr(bus, addr):
    bus.regs.mmio_mem_adr_l.write(addr & 0xffffffff)
    bus.regs.mmio_mem_adr_h.write((addr >> 32) & 0xffffffff)

def mem_start(bus, we, wsel=0xf, length=1):
    ctrl  = 0
    ctrl |= 1 << 0
    ctrl |= (1 if we else 0) << 1
    ctrl |= (wsel & 0xf) << 4
    ctrl |= (length & 0x3ff) << 8
    bus.regs.mmio_mem_ctrl.write(ctrl)
    bus.regs.mmio_mem_ctrl.write(0)

def mem_wait_done(bus, timeout_ms=200):
    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.mmio_mem_stat.read()
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            return err
        if time.time() > deadline:
            raise TimeoutError("MEM transaction timeout (done=0).")

def mmio_rd32(bus, addr, timeout_ms=200):
    mem_set_addr(bus, addr)
    mem_start(bus, we=0, wsel=0xf, length=1)
    err = mem_wait_done(bus, timeout_ms=timeout_ms)
    val = bus.regs.mmio_mem_rdata.read()
    return val, err

def mmio_wr32(bus, addr, data, wsel=0xf, timeout_ms=200):
    mem_set_addr(bus, addr)
    bus.regs.mmio_mem_wdata.write(data & 0xffffffff)
    mem_start(bus, we=1, wsel=wsel, length=1)
    err = mem_wait_done(bus, timeout_ms=timeout_ms)
    return err

def mmio_rd64(bus, addr, timeout_ms=200):
    lo, err0 = mmio_rd32(bus, addr + 0, timeout_ms=timeout_ms)
    hi, err1 = mmio_rd32(bus, addr + 4, timeout_ms=timeout_ms)
    return ((hi << 32) | lo), (err0 | err1)

def mmio_wr64(bus, addr, data, timeout_ms=200):
    lo = data & 0xffffffff
    hi = (data >> 32) & 0xffffffff
    err0 = mmio_wr32(bus, addr + 0, lo, timeout_ms=timeout_ms)
    err1 = mmio_wr32(bus, addr + 4, hi, timeout_ms=timeout_ms)
    return err0 | err1

def mmio_rd32_safe(bus, addr, timeout_ms=200, retries=5):
    last = None
    for _ in range(retries):
        try:
            return mmio_rd32(bus, addr, timeout_ms=timeout_ms)
        except TimeoutError as e:
            last = e
            time.sleep(0.01)
    raise last

def mmio_wr32_safe(bus, addr, data, wsel=0xf, timeout_ms=200, retries=5):
    last = None
    for _ in range(retries):
        try:
            return mmio_wr32(bus, addr, data, wsel=wsel, timeout_ms=timeout_ms)
        except TimeoutError as e:
            last = e
            time.sleep(0.01)
    raise last


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

NVME_DOORBELL_BASE = 0x1000

# NVMe decoders ------------------------------------------------------------------------------------

def cap_decode(cap):
    mqes   = _bits(cap, 0, 15)
    cqr    = _bit(cap, 16)
    ams    = _bits(cap, 17, 18)
    to     = _bits(cap, 24, 31)
    dstrd  = _bits(cap, 32, 35)
    nssrs  = _bit(cap, 36)
    css    = _bits(cap, 37, 44)
    mpsmin = _bits(cap, 48, 51)
    mpsmax = _bits(cap, 52, 55)
    return {
        "mqes": mqes,
        "cqr": cqr,
        "ams": ams,
        "to": to,
        "dstrd": dstrd,
        "nssrs": nssrs,
        "css": css,
        "mpsmin": mpsmin,
        "mpsmax": mpsmax,
    }

def vs_decode(vs):
    return {
        "major": _bits(vs, 16, 31),
        "minor": _bits(vs, 8, 15),
        "ter":   _bits(vs, 0, 7),
    }

def csts_rdy(csts):  return _bit(csts, 0)
def csts_cfs(csts):  return _bit(csts, 1)
def csts_shst(csts): return _bits(csts, 2, 3)
def cc_en(cc):       return _bit(cc, 0)

def cap_to_ms(to_field):
    return int(to_field) * 500

def doorbell_stride_bytes(dstrd):
    return (1 << int(dstrd)) * 4

# NVMe CC helpers ----------------------------------------------------------------------------------

def cc_make_en(iocqes=4, iosqes=6, mps=0, css=0, ams=0, shn=0):
    cc  = 0
    cc |= (1      & 0x1) << 0
    cc |= (css    & 0x7) << 4
    cc |= (mps    & 0xf) << 7
    cc |= (ams    & 0x7) << 11
    cc |= (shn    & 0x3) << 14
    cc |= (iosqes & 0xf) << 16
    cc |= (iocqes & 0xf) << 20
    return cc

def nvme_db_addr(bar0, cap, qid, is_cq=False):
    capd   = cap_decode(cap)
    stride = doorbell_stride_bytes(capd["dstrd"])
    off = NVME_DOORBELL_BASE + (qid*2 + (1 if is_cq else 0)) * stride
    return bar0 + off

# NVMe actions -------------------------------------------------------------------------------------

def nvme_read_core(bus, bar0, timeout_ms=200):
    cap,  e0 = mmio_rd64(bus, bar0 + NVME_CAP,  timeout_ms=timeout_ms)
    vs,   e1 = mmio_rd32(bus, bar0 + NVME_VS,   timeout_ms=timeout_ms)
    intms,e2 = mmio_rd32(bus, bar0 + NVME_INTMS,timeout_ms=timeout_ms)
    intmc,e3 = mmio_rd32(bus, bar0 + NVME_INTMC,timeout_ms=timeout_ms)
    cc,   e4 = mmio_rd32(bus, bar0 + NVME_CC,   timeout_ms=timeout_ms)
    csts, e5 = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
    aqa,  e6 = mmio_rd32(bus, bar0 + NVME_AQA,  timeout_ms=timeout_ms)
    asq,  e7 = mmio_rd64(bus, bar0 + NVME_ASQ,  timeout_ms=timeout_ms)
    acq,  e8 = mmio_rd64(bus, bar0 + NVME_ACQ,  timeout_ms=timeout_ms)

    errs = e0 | e1 | e2 | e3 | e4 | e5 | e6 | e7 | e8

    capd = cap_decode(cap)
    vsd  = vs_decode(vs)

    print(f"NVMe BAR0      0x{bar0:016x}")
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
    print(f"INTMS          0x{intms:08x}")
    print(f"INTMC          0x{intmc:08x}")
    print(f"CC             0x{cc:08x} (EN={cc_en(cc)})")
    print(f"CSTS           0x{csts:08x} (RDY={csts_rdy(csts)} CFS={csts_cfs(csts)} SHST=0x{csts_shst(csts):x})")
    print(f"AQA            0x{aqa:08x}")
    print(f"ASQ            0x{asq:016x}")
    print(f"ACQ            0x{acq:016x}")

    if errs:
        print("NOTE: Some MMIO ops returned err=1 (UR/CA/etc). Check MEM/BME, BAR assignment, and offsets.")

    return {
        "cap": cap,
        "vs": vs,
        "intms": intms,
        "intmc": intmc,
        "cc": cc,
        "csts": csts,
        "aqa": aqa,
        "asq": asq,
        "acq": acq,
        "errs": errs,
    }

def nvme_wait_rdy(bus, bar0, want_rdy, timeout_ms, poll_s=0.005):
    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        csts, err = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
        if err == 0 and csts_rdy(csts) == (1 if want_rdy else 0):
            return True
        if time.time() > deadline:
            return False
        time.sleep(poll_s)

def nvme_disable(bus, bar0, cap=None, timeout_ms=200):
    cc, err = mmio_rd32(bus, bar0 + NVME_CC, timeout_ms=timeout_ms)
    if err:
        print("WARN: read CC returned err=1.")
    if cc_en(cc) == 0:
        return True
    mmio_wr32(bus, bar0 + NVME_CC, cc & ~0x1, timeout_ms=timeout_ms)
    to_ms = 2000
    if cap is not None:
        to_ms = max(500, cap_to_ms(cap_decode(cap)["to"]))
    return nvme_wait_rdy(bus, bar0, want_rdy=False, timeout_ms=to_ms, poll_s=0.001)

def nvme_admin_init(bus, bar0, cap, asq_addr, acq_addr, q_entries=2, disable=True, timeout_ms=200):
    assert q_entries >= 2
    aqa = ((q_entries - 1) & 0xfff) | (((q_entries - 1) & 0xfff) << 16)

    if disable:
        nvme_disable(bus, bar0, cap=cap, timeout_ms=timeout_ms)

    e0 = mmio_wr32(bus, bar0 + NVME_AQA, aqa, timeout_ms=timeout_ms)
    e1 = mmio_wr64(bus, bar0 + NVME_ASQ, asq_addr, timeout_ms=timeout_ms)
    e2 = mmio_wr64(bus, bar0 + NVME_ACQ, acq_addr, timeout_ms=timeout_ms)
    if e0 | e1 | e2:
        print("WARN: writing AQA/ASQ/ACQ had err=1 (posted/UR/CA).")

    cc = cc_make_en(iocqes=4, iosqes=6, mps=0, css=0, ams=0)
    e3 = mmio_wr32(bus, bar0 + NVME_CC, cc, timeout_ms=timeout_ms)
    if e3:
        print("WARN: writing CC had err=1.")

    to_ms = max(500, cap_to_ms(cap_decode(cap)["to"]))
    return nvme_wait_rdy(bus, bar0, want_rdy=True, timeout_ms=to_ms, poll_s=0.001)

def nvme_ring_admin_sq(bus, bar0, cap, tail, timeout_ms=200):
    db = nvme_db_addr(bar0, cap, qid=0, is_cq=False)
    err = mmio_wr32_safe(bus, db, tail & 0xffff, timeout_ms=timeout_ms, retries=8)
    if err:
        print("WARN: SQ0 doorbell write err=1.")
    return err == 0

def nvme_ring_sq(bus, bar0, cap, qid, tail, timeout_ms=200):
    db = nvme_db_addr(bar0, cap, qid=qid, is_cq=False)
    mmio_wr32_safe(bus, db, tail & 0xffff, timeout_ms=timeout_ms)

def nvme_ring_cq(bus, bar0, cap, qid, head, timeout_ms=200):
    db = nvme_db_addr(bar0, cap, qid=qid, is_cq=True)
    mmio_wr32_safe(bus, db, head & 0xffff, timeout_ms=timeout_ms)

# NVMe commands ------------------------------------------------------------------------------------

def nvme_cmd_identify_controller(cid, prp1, nsid=0):
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = nsid & 0xffffffff                       # NSID is DW1
    cmd[6]  = prp1 & 0xffffffff
    cmd[7]  = (prp1 >> 32) & 0xffffffff
    cmd[10] = 0x00000001  # CNS=1
    return cmd

def nvme_cmd_create_iocq(cid, prp1_cq, qid=1, qsize_minus1=1, iv=0, ien=0, pc=1):
    cmd = [0]*16
    cmd[0]  = (0x05 & 0xff) | ((cid & 0xffff) << 16)  # Create IO CQ
    cmd[6]  = prp1_cq & 0xffffffff
    cmd[7]  = (prp1_cq >> 32) & 0xffffffff
    cmd[10] = (qid & 0xffff) | ((qsize_minus1 & 0xffff) << 16)

    # CDW11: [15:0]=CQ_FLAGS, [31:16]=IV
    # CQ_FLAGS: bit0=PC, bit1=IEN
    cq_flags = ((pc & 0x1) << 0) | ((ien & 0x1) << 1)
    cmd[11]  = ((iv & 0xffff) << 16) | (cq_flags & 0xffff)
    return cmd

def nvme_cmd_create_iosq(cid, prp1_sq, qid=1, qsize_minus1=1, cqid=1, qprio=0, pc=1):
    cmd = [0]*16
    cmd[0]  = (0x01 & 0xff) | ((cid & 0xffff) << 16)  # Create IO SQ
    cmd[6]  = prp1_sq & 0xffffffff
    cmd[7]  = (prp1_sq >> 32) & 0xffffffff
    cmd[10] = (qid & 0xffff) | ((qsize_minus1 & 0xffff) << 16)

    # CDW11: [15:0]=SQ_FLAGS, [31:16]=CQID
    # SQ_FLAGS: bit0=PC, bits[2:1]=QPRIO
    sq_flags = ((pc & 0x1) << 0) | ((qprio & 0x3) << 1)
    cmd[11]  = ((cqid & 0xffff) << 16) | (sq_flags & 0xffff)
    return cmd

def nvme_cmd_read(cid, nsid, prp1_data, slba, nlb_minus1):
    cmd = [0]*16
    cmd[0]  = (0x02 & 0xff) | ((cid & 0xffff) << 16)  # Read
    cmd[1]  = nsid & 0xffffffff                       # NSID is DW1
    cmd[6]  = prp1_data & 0xffffffff
    cmd[7]  = (prp1_data >> 32) & 0xffffffff
    cmd[10] = slba & 0xffffffff
    cmd[11] = (slba >> 32) & 0xffffffff
    cmd[12] = (nlb_minus1 & 0xffff)
    return cmd

def nvme_cmd_write(cid, nsid, prp1_data, slba, nlb_minus1):
    cmd = [0]*16
    cmd[0]  = (0x01 & 0xff) | ((cid & 0xffff) << 16)  # Write
    cmd[1]  = nsid & 0xffffffff                       # NSID is DW1
    cmd[6]  = prp1_data & 0xffffffff
    cmd[7]  = (prp1_data >> 32) & 0xffffffff
    cmd[10] = slba & 0xffffffff
    cmd[11] = (slba >> 32) & 0xffffffff
    cmd[12] = (nlb_minus1 & 0xffff)
    return cmd

def nvme_cmd_identify_namespace(cid, nsid, prp1):
    # Identify, CNS=0, NSID=nsid
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = nsid & 0xffffffff                       # FIXED: NSID in DW1, not DW2
    cmd[6]  = prp1 & 0xffffffff
    cmd[7]  = (prp1 >> 32) & 0xffffffff
    cmd[10] = 0x00000000  # CNS=0 (Identify Namespace)
    return cmd

def nvme_cmd_identify_ns_list(cid, prp1):
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = 0  # NSID=0 in DW1
    cmd[6]  = prp1 & 0xffffffff
    cmd[7]  = (prp1 >> 32) & 0xffffffff
    cmd[10] = 0x00000002  # CNS=2
    return cmd


def nvme_cmd_set_features_num_queues(cid, nsqr, ncqr):
    # Admin opcode 0x09 (Set Features), FID=0x07 (Number of Queues).
    # CDW10: [7:0]=FID, [15:0]=NSQR-1, [31:16]=NCQR-1
    # Fixed: Proper bitfield construction without overlap
    cmd = [0]*16
    cmd[0]  = (0x09 & 0xff) | ((cid & 0xffff) << 16)

    # Construct CDW10 carefully:
    # Bits 7:0 = FID (0x07)
    # Bits 15:0 = NSQR-1 (Number of Submission Queues Requested - 1)
    # Bits 31:16 = NCQR-1 (Number of Completion Queues Requested - 1)
    cdw10 = 0x07  # FID in bits 7:0
    cdw10 |= ((nsqr - 1) & 0xffff) << 0    # NSQR-1 in bits 15:0
    cdw10 |= ((ncqr - 1) & 0xffff) << 16   # NCQR-1 in bits 31:16
    cmd[10] = cdw10
    return cmd



# Completion polling --------------------------------------------------------------------------------

def nvme_poll_acq0(bus, acq_base, timeout_s=2.0, hostmem_base=0x10000000):
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        d0 = hostmem_rd32(bus, acq_base + 0x0, base=hostmem_base)
        d1 = hostmem_rd32(bus, acq_base + 0x4, base=hostmem_base)
        d2 = hostmem_rd32(bus, acq_base + 0x8, base=hostmem_base)
        d3 = hostmem_rd32(bus, acq_base + 0xc, base=hostmem_base)
        cur = (d0, d1, d2, d3)
        if cur != last:
            last = cur
        if (d0 | d1 | d2 | d3) != 0:
            return cur
        time.sleep(0.01)
    return last

def poll_cq_phase(bus, cq_base, head, phase, timeout_s=2.0, hostmem_base=0x10000000):
    # FIXED: Was using undefined variable 'deadline' on RHS
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        d0, d1, d2, d3 = hostmem_rd_cqe_slot(bus, cq_base, head, hostmem_base=hostmem_base)
        cqe = decode_cqe(d0, d1, d2, d3)
        if cqe["p"] == phase:
            return cqe
        time.sleep(0.001)
    return None

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

    info = None
    if args.info or args.disable or args.mmio_check or args.doorbells or args.identify or args.read:
        info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)

    if args.doorbells:
        for q in range(0, max_q + 1):
            for is_cq in [False, True]:
                db = nvme_db_addr(bar0, info["cap"], qid=q, is_cq=is_cq)
                v, err = mmio_rd32(bus, db, timeout_ms=timeout_ms)
                which = "CQ" if is_cq else "SQ"
                print(f"DB {which}{q} @0x{db:08x}: 0x{v:08x} err={err}")

    hostmem_base = int(args.hostmem_base, 0)
    asq_addr     = int(args.asq_addr, 0)
    acq_addr     = int(args.acq_addr, 0)
    cid          = int(args.cid, 0)
    q_entries    = int(args.q_entries, 0)

    if args.identify:
        id_buf = int(args.id_buf, 0)

        hostmem_fill(bus, acq_addr, 16, value=0, base=hostmem_base)
        hostmem_fill(bus, id_buf,  0x100, value=0, base=hostmem_base)

        cfg_set_command(bus, b, d, f, set_mem=True, set_bme=True, timeout_ms=cfg_timeout_ms)

        ok = nvme_admin_init(bus, bar0, info["cap"], asq_addr, acq_addr, q_entries=q_entries, timeout_ms=timeout_ms)

        csts, _ = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
        print(f"Admin init: RDY={'1' if csts_rdy(csts) else '0'} CFS={csts_cfs(csts)} (poll_ok={'1' if ok else '0'})")

        cmd = nvme_cmd_identify_controller(cid=cid, prp1=id_buf, nsid=0)
        hostmem_wr_cmd(bus, asq_base=asq_addr, slot=0, cmd_dwords=cmd, hostmem_base=hostmem_base)
        print("ASQ[0] programmed (Identify Controller).")

        nvme_ring_admin_sq(bus, bar0, info["cap"], tail=1, timeout_ms=timeout_ms)
        print("SQ0 doorbell rung (tail=1).")

        cpl = nvme_poll_acq0(bus, acq_base=acq_addr, timeout_s=2.0, hostmem_base=hostmem_base)
        print("ACQ[0] raw:")
        if cpl is None:
            print("  (no completion observed)")
        else:
            print(f"  {cpl[0]:08x} {cpl[1]:08x} {cpl[2]:08x} {cpl[3]:08x}")
            cqe = decode_cqe(cpl[0], cpl[1], cpl[2], cpl[3])
            pretty_print_cqe(cqe, name="ACQ[0]")

        print("Identify buffer (first 0x100):")
        hostmem_dump(bus, addr=id_buf, length=0x100, base=hostmem_base)

        id_dws = hostmem_read_dwords(bus, addr=id_buf, dword_count=0x100//4, base=hostmem_base)
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

        hostmem_fill(bus, io_cq_addr, 0x1000, value=0, base=hostmem_base)
        hostmem_fill(bus, io_sq_addr, 0x1000, value=0, base=hostmem_base)
        hostmem_fill(bus, rd_buf,     0x1000, value=0, base=hostmem_base)  # Fixed: was 0x200, should be page size minimum
        hostmem_fill(bus, wr_buf,     0x1000, value=0, base=hostmem_base)

        # Scratch 4K page for Identify NS list.
        ns_buf = hostmem_base + 0x6000
        hostmem_fill(bus, ns_buf, 0x1000, value=0, base=hostmem_base)

        # Prime MMIO engine.
        mmio_rd32_safe(bus, bar0 + NVME_VS, timeout_ms=max(timeout_ms, 500), retries=8)

        # Bring controller up.
        ok = False
        for attempt in range(3):
            try:
                ok = nvme_admin_init(bus, bar0, info["cap"], asq_addr, acq_addr,
                                               q_entries=q_entries, disable=False,
                                               timeout_ms=max(timeout_ms, 500))
                break
            except TimeoutError:
                time.sleep(0.05)

        if not ok:
            print("Admin init failed (MMIO timeouts).")
            bus.close()
            return

        csts, _ = mmio_rd32_safe(bus, bar0 + NVME_CSTS, timeout_ms=max(timeout_ms, 500), retries=8)
        print(f"Admin init (for read): RDY={'1' if csts_rdy(csts) else '0'} CFS={csts_cfs(csts)} (poll_ok={'1' if ok else '0'})")
        if not ok:
            bus.close()
            return

        acq_head = 0
        acq_phase = 1

        def admin_submit(cmd_dwords, slot, tail_after, timeout_s=2.0):
            nonlocal acq_head, acq_phase

            hostmem_fill(bus, acq_addr + acq_head*16, 16, value=0, base=hostmem_base)

            hostmem_wr_cmd(bus, asq_base=asq_addr, slot=slot, cmd_dwords=cmd_dwords, hostmem_base=hostmem_base)
            nvme_ring_admin_sq(bus, bar0, info["cap"], tail=tail_after, timeout_ms=timeout_ms)  # should be safe-wrapped already

            cqe = poll_cq_phase(bus, cq_base=acq_addr, head=acq_head, phase=acq_phase,
                                timeout_s=timeout_s, hostmem_base=hostmem_base)
            if cqe is None:
                return None

            acq_head = (acq_head + 1) % q_entries
            if acq_head == 0:
                acq_phase ^= 1
            nvme_ring_cq(bus, bar0, info["cap"], qid=0, head=acq_head, timeout_ms=timeout_ms)
            return cqe

        # 1) Set Features: Number of Queues (request 1 SQ + 1 CQ).
        cmd = nvme_cmd_set_features_num_queues(cid=cid + 20, nsqr=1, ncqr=1)
        cqe = admin_submit(cmd, slot=0, tail_after=1, timeout_s=2.0)
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
        cqe = admin_submit(cmd, slot=1, tail_after=2, timeout_s=2.0)
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
        cqe = admin_submit(cmd, slot=2, tail_after=3, timeout_s=2.0)
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
        hostmem_fill(bus, ns_buf, 0x1000, value=0, base=hostmem_base)
        cmd = nvme_cmd_identify_ns_list(cid=cid + 10, prp1=ns_buf)
        cqe = admin_submit(cmd, slot=3, tail_after=4, timeout_s=2.0)
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

        ns_list = hostmem_read_dwords(bus, ns_buf, 16, base=hostmem_base)
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
            more = hostmem_read_dwords(bus, ns_buf + 16*4, 64, base=hostmem_base)  # +64 dwords
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


        # 5) Submit IO commands on SQ1.
        iosq_tail  = 0
        iocq_head  = 0
        iocq_phase = 1

        def io_submit(cmd_dwords, timeout_s=2.0):
            nonlocal iosq_tail, iocq_head, iocq_phase

            hostmem_fill(bus, io_cq_addr + iocq_head*16, 16, value=0, base=hostmem_base)
            hostmem_wr_cmd(bus, asq_base=io_sq_addr, slot=iosq_tail, cmd_dwords=cmd_dwords, hostmem_base=hostmem_base)

            tail_next = (iosq_tail + 1) % io_q_entries
            nvme_ring_sq(bus, bar0, info["cap"], qid=1, tail=tail_next, timeout_ms=timeout_ms)
            iosq_tail = tail_next

            cqe = poll_cq_phase(bus, cq_base=io_cq_addr, head=iocq_head, phase=iocq_phase,
                                timeout_s=timeout_s, hostmem_base=hostmem_base)
            if cqe is None:
                return None

            iocq_head = (iocq_head + 1) % io_q_entries
            if iocq_head == 0:
                iocq_phase ^= 1
            nvme_ring_cq(bus, bar0, info["cap"], qid=1, head=iocq_head, timeout_ms=timeout_ms)
            return cqe

        if args.write:
            pat = 0xA5A5A5A5
            hostmem_fill(bus, wr_buf, 0x1000, value=pat, base=hostmem_base)

            print(f"Submitting WRITE: NSID={nsid} SLBA={slba} NLB={nlb} PRP1=0x{wr_buf:08x}")
            write_cmd = nvme_cmd_write(cid=cid + 3, nsid=nsid, prp1_data=wr_buf, slba=slba, nlb_minus1=nlb-1)
            print("WRITE SQE:", " ".join(f"{w:08x}" for w in write_cmd))

            cqe = io_submit(write_cmd, timeout_s=2.0)
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
            hostmem_fill(bus, rd_buf, 0x1000, value=pat, base=hostmem_base)

            # Snapshot a small prefix (64 dwords = 256B) before the read.
            before = hostmem_read_dwords(bus, rd_buf, 64, base=hostmem_base)

            print(f"Submitting READ: NSID={nsid} SLBA={slba} NLB={nlb} PRP1=0x{rd_buf:08x}")
            read_cmd = nvme_cmd_read(cid=cid + 4, nsid=nsid, prp1_data=rd_buf, slba=slba, nlb_minus1=nlb-1)
            print("READ SQE:", " ".join(f"{w:08x}" for w in read_cmd))

            cqe = io_submit(read_cmd, timeout_s=2.0)
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
            after = hostmem_read_dwords(bus, rd_buf, 64, base=hostmem_base)

            changed = sum(1 for i in range(64) if after[i] != before[i])
            still_pat = sum(1 for i in range(64) if after[i] == pat)
            all_zero  = sum(1 for i in range(64) if after[i] == 0)

            print(f"Read buffer changed dwords: {changed}/64")
            print(f"After read: {still_pat}/64 still pattern, {all_zero}/64 are zero")

            print("Read buffer (first 0x200):")
            hostmem_dump(bus, addr=rd_buf, length=0x200, base=hostmem_base)

            sig = hostmem_rd32(bus, rd_buf + 0x1FC, base=hostmem_base)
            sig16 = (sig >> 16) & 0xffff
            print(f"MBR sig16: 0x{sig16:04x} (expect aa55)")

            if args.write and args.write_verify:
                wr_before = hostmem_read_dwords(bus, wr_buf, 64, base=hostmem_base)
                wr_after  = hostmem_read_dwords(bus, rd_buf, 64, base=hostmem_base)
                mism = sum(1 for i in range(64) if wr_before[i] != wr_after[i])
                print(f"Write verify: mismatched dwords {mism}/64")


    bus.close()

if __name__ == "__main__":
    main()
