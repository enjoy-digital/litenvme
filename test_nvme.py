#!/usr/bin/env python3

import argparse
import time

from litex import RemoteClient

# Helpers ------------------------------------------------------------------------------------------

def _bit(v, n):
    return (v >> n) & 1

def _bits(v, lo, hi):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)

def wait_link_up(bus, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while True:
        v = bus.regs.pcie_phy_phy_link_status.read()
        link = (v >> 0) & 1
        if link:
            return True
        if time.time() > deadline:
            return False
        time.sleep(0.05)

# HostMem CSR helpers -----------------------------------------------------------------------------

def hostmem_set_adr(bus, dword_adr):
    bus.regs.hostmem_csr_adr.write(dword_adr & 0xffffffff)

def hostmem_wr32(bus, addr, data, base=0x10000000):
    # addr is absolute byte address in the hostmem window.
    dword = (addr - base) >> 2
    hostmem_set_adr(bus, dword)
    bus.regs.hostmem_csr_wdata.write(data & 0xffffffff)
    bus.regs.hostmem_csr_we.write(1)
    bus.regs.hostmem_csr_we.write(0)

def hostmem_rd32(bus, addr, base=0x10000000):
    dword = (addr - base) >> 2
    hostmem_set_adr(bus, dword)
    return bus.regs.hostmem_csr_rdata.read()

def hostmem_fill(bus, addr, length, value=0, base=0x10000000):
    for off in range(0, length, 4):
        hostmem_wr32(bus, addr + off, value, base=base)

def hostmem_dump(bus, addr, length, base=0x10000000):
    for off in range(0, length, 4):
        v = hostmem_rd32(bus, addr + off, base=base)
        print(f"0x{off:04x}: 0x{v:08x}")

# CFG helpers --------------------------------------------------------------------------------------

def cfg_bdf_pack(bus, dev, fn, reg, ext=0):
    v  = (bus & 0xff) << 0
    v |= (dev & 0x1f) << 8
    v |= (fn  & 0x07) << 13
    v |= (reg & 0x3f) << 16
    v |= (ext & 0x07) << 22
    return v

def cfg_rd0(bus, b, d, f, reg_dword, ext=0, timeout_ms=100):
    bus.regs.cfg_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
    bus.regs.cfg_cfg_ctrl.write(1)
    bus.regs.cfg_cfg_ctrl.write(0)

    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.cfg_cfg_stat.read()
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            return bus.regs.cfg_cfg_rdata.read(), err
        if time.time() > deadline:
            raise TimeoutError("CFG read timeout (done=0).")

def cfg_wr0(bus, b, d, f, reg_dword, wdata, ext=0, timeout_ms=100):
    bus.regs.cfg_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
    bus.regs.cfg_cfg_wdata.write(wdata & 0xffffffff)
    bus.regs.cfg_cfg_ctrl.write(1 | (1<<1))  # start=1, we=1
    bus.regs.cfg_cfg_ctrl.write(0)

    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.cfg_cfg_stat.read()
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            return err
        if time.time() > deadline:
            raise TimeoutError("CFG write timeout (done=0).")

def discover_bar0(bus, b, d, f, timeout_ms=100):
    # BAR0: dword 4 (0x10), BAR1: dword 5 (0x14).
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

# MEM/MMIO helpers ---------------------------------------------------------------------------------

def mem_set_addr(bus, addr):
    bus.regs.mmio_mem_adr_l.write(addr & 0xffffffff)
    bus.regs.mmio_mem_adr_h.write((addr >> 32) & 0xffffffff)

def mem_start(bus, we, wsel=0xf, length=1):
    # ctrl fields:
    # start bit0
    # we    bit1
    # wsel  bits[7:4]
    # len   bits[17:8]
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
    # NVMe CAP (64-bit).
    mqes   = _bits(cap, 0, 15)      # 0-based.
    cqr    = _bit(cap, 16)
    ams    = _bits(cap, 17, 18)
    to     = _bits(cap, 24, 31)     # 500ms units.
    dstrd  = _bits(cap, 32, 35)     # stride = 2^dstrd * 4 bytes.
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
    # VS: major[31:16], minor[15:8], tertiary[7:0]
    return {
        "major": _bits(vs, 16, 31),
        "minor": _bits(vs, 8, 15),
        "ter":   _bits(vs, 0, 7),
    }

def csts_rdy(csts):
    return _bit(csts, 0)

def csts_cfs(csts):
    return _bit(csts, 1)

def csts_shst(csts):
    return _bits(csts, 2, 3)

def cc_en(cc):
    return _bit(cc, 0)

def cap_to_ms(to_field):
    return int(to_field) * 500

def doorbell_stride_bytes(dstrd):
    return (1 << int(dstrd)) * 4

# NVMe CC helpers ----------------------------------------------------------------------------------

def cc_make_en(iocqes=4, iosqes=6, mps=0, css=0, ams=0, shn=0):
    # iocqes: 16B CQ entry => 4 (2^4)
    # iosqes: 64B SQ entry => 6 (2^6)
    cc  = 0
    cc |= (1      & 0x1) << 0    # EN
    cc |= (css    & 0x7) << 4    # CSS
    cc |= (mps    & 0xf) << 7    # MPS
    cc |= (ams    & 0x7) << 11   # AMS
    cc |= (shn    & 0x3) << 14   # SHN
    cc |= (iosqes & 0xf) << 16   # IOSQES
    cc |= (iocqes & 0xf) << 20   # IOCQES
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

def nvme_dump_regs(bus, bar0, length, stride=4, timeout_ms=200):
    for off in range(0, length, stride):
        v, err = mmio_rd32(bus, bar0 + off, timeout_ms=timeout_ms)
        s = "" if err == 0 else "  (err=1)"
        print(f"0x{off:04x}: 0x{v:08x}{s}")

def nvme_wait_rdy(bus, bar0, want_rdy, timeout_ms, poll_s=0.005):
    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        csts, err = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
        if err == 0:
            if csts_rdy(csts) == (1 if want_rdy else 0):
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

    new_cc = cc & ~0x1
    mmio_wr32(bus, bar0 + NVME_CC, new_cc, timeout_ms=timeout_ms)

    to_ms = 2000
    if cap is not None:
        to_ms = max(500, cap_to_ms(cap_decode(cap)["to"]))
    ok = nvme_wait_rdy(bus, bar0, want_rdy=False, timeout_ms=to_ms, poll_s=0.001)
    return ok

def nvme_check_mmio_basics(bus, bar0, timeout_ms=200):
    print("MMIO sanity:")

    cap, e0 = mmio_rd64(bus, bar0 + NVME_CAP,  timeout_ms=timeout_ms)
    vs,  e1 = mmio_rd32(bus, bar0 + NVME_VS,   timeout_ms=timeout_ms)
    cc,  e2 = mmio_rd32(bus, bar0 + NVME_CC,   timeout_ms=timeout_ms)
    csts,e3 = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)

    if e0 | e1 | e2 | e3:
        print("  FAIL: basic reads returned err=1.")
        return False

    print("  OK: basic reads (CAP/VS/CC/CSTS).")

    if cc_en(cc) != 0:
        print("  WARN: CC.EN=1, skipping write/readback tests (disable first).")
        return True

    # INTMS/INTMC: best-effort only.
    intms0, er0 = mmio_rd32(bus, bar0 + NVME_INTMS, timeout_ms=timeout_ms)
    if er0:
        print("  WARN: read INTMS returned err=1 (not fatal).")
    else:
        w1 = mmio_wr32(bus, bar0 + NVME_INTMS, 0x1, timeout_ms=timeout_ms)
        intms1, er1 = mmio_rd32(bus, bar0 + NVME_INTMS, timeout_ms=timeout_ms)
        w2 = mmio_wr32(bus, bar0 + NVME_INTMC, 0x1, timeout_ms=timeout_ms)
        intms2, er2 = mmio_rd32(bus, bar0 + NVME_INTMS, timeout_ms=timeout_ms)

        if w1 | er1 | w2 | er2:
            print("  WARN: write/read INTMS/INTMC returned err=1 (not fatal).")
        else:
            ok_set = (intms1 & 0x1) == 0x1
            ok_clr = (intms2 & 0x1) == 0x0
            if ok_set and ok_clr:
                print("  OK: INTMS bit0 set/clear visible.")
            else:
                print("  WARN: INTMS bit0 set/clear not visible (posted/RO/impl-defined).")

        mmio_wr32(bus, bar0 + NVME_INTMS, intms0, timeout_ms=timeout_ms)

    # AQA/ASQ/ACQ: best-effort.
    aqa0, ea0 = mmio_rd32(bus, bar0 + NVME_AQA, timeout_ms=timeout_ms)
    asq0, ea1 = mmio_rd64(bus, bar0 + NVME_ASQ, timeout_ms=timeout_ms)
    acq0, ea2 = mmio_rd64(bus, bar0 + NVME_ACQ, timeout_ms=timeout_ms)
    if ea0 | ea1 | ea2:
        print("  WARN: read AQA/ASQ/ACQ returned err=1 (not fatal).")
        return True

    aqa_t = aqa0 ^ 0x00010001
    asq_t = (asq0 ^ 0x0000000000001000) & ~0xfff
    acq_t = (acq0 ^ 0x0000000000002000) & ~0xfff

    w0 = mmio_wr32(bus, bar0 + NVME_AQA, aqa_t, timeout_ms=timeout_ms)
    w1 = mmio_wr64(bus, bar0 + NVME_ASQ, asq_t, timeout_ms=timeout_ms)
    w2 = mmio_wr64(bus, bar0 + NVME_ACQ, acq_t, timeout_ms=timeout_ms)
    r0, rr0 = mmio_rd32(bus, bar0 + NVME_AQA, timeout_ms=timeout_ms)
    r1, rr1 = mmio_rd64(bus, bar0 + NVME_ASQ, timeout_ms=timeout_ms)
    r2, rr2 = mmio_rd64(bus, bar0 + NVME_ACQ, timeout_ms=timeout_ms)

    mmio_wr32(bus, bar0 + NVME_AQA, aqa0, timeout_ms=timeout_ms)
    mmio_wr64(bus, bar0 + NVME_ASQ, asq0, timeout_ms=timeout_ms)
    mmio_wr64(bus, bar0 + NVME_ACQ, acq0, timeout_ms=timeout_ms)

    if w0 | w1 | w2 | rr0 | rr1 | rr2:
        print("  WARN: AQA/ASQ/ACQ write/readback returned err=1 (not fatal).")
        return True

    ok_aqa = (r0 == aqa_t)
    ok_asq = (r1 == asq_t)
    ok_acq = (r2 == acq_t)

    if ok_aqa and ok_asq and ok_acq:
        print("  OK: AQA/ASQ/ACQ write/readback visible.")
    else:
        print("  WARN: AQA/ASQ/ACQ write/readback not visible (impl-defined).")

    return True

def nvme_check_doorbells(bus, bar0, cap, timeout_ms=200, max_q=4):
    capd   = cap_decode(cap)
    stride = doorbell_stride_bytes(capd["dstrd"])

    print(f"Doorbells: base=0x{NVME_DOORBELL_BASE:04x} stride={stride} bytes (check reads only)")

    for qid in range(max_q):
        sq_off = NVME_DOORBELL_BASE + (qid * 2 + 0) * stride
        cq_off = NVME_DOORBELL_BASE + (qid * 2 + 1) * stride

        sq, e0 = mmio_rd32(bus, bar0 + sq_off, timeout_ms=timeout_ms)
        cq, e1 = mmio_rd32(bus, bar0 + cq_off, timeout_ms=timeout_ms)

        s0 = "" if e0 == 0 else " (err=1)"
        s1 = "" if e1 == 0 else " (err=1)"
        print(f"  Q{qid}: SQD@0x{sq_off:04x}=0x{sq:08x}{s0}  CQD@0x{cq_off:04x}=0x{cq:08x}{s1}")

# Admin init + doorbells ---------------------------------------------------------------------------

def nvme_admin_init(bus, bar0, cap, asq_addr, acq_addr, q_entries=2, timeout_ms=200):
    # AQA: ASQS (bits 11:0) and ACQS (bits 27:16), values are (N-1).
    assert q_entries >= 2
    aqa = ((q_entries - 1) & 0xfff) | (((q_entries - 1) & 0xfff) << 16)

    # Disable first (best-effort).
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
    ok = nvme_wait_rdy(bus, bar0, want_rdy=True, timeout_ms=to_ms, poll_s=0.001)
    return ok

def nvme_ring_admin_sq(bus, bar0, cap, tail, timeout_ms=200):
    db = nvme_db_addr(bar0, cap, qid=0, is_cq=False)
    err = mmio_wr32(bus, db, tail & 0xffff, timeout_ms=timeout_ms)
    if err:
        print("WARN: SQ0 doorbell write err=1.")
    return err == 0

# NVMe Admin Identify command ---------------------------------------------------------------------

def nvme_cmd_identify_controller(cid, prp1, nsid=0):
    # 64-byte NVMe command = 16 dwords.
    # CDW0: opcode[7:0]=0x06 (Identify), CID[31:16]
    # PRP1: dwords 6-7
    # CDW10: CNS[7:0]=1 (Identify Controller)
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = 0x00000000
    cmd[2]  = nsid & 0xffffffff
    cmd[3]  = 0x00000000
    cmd[4]  = 0x00000000
    cmd[5]  = 0x00000000
    cmd[6]  = prp1 & 0xffffffff
    cmd[7]  = (prp1 >> 32) & 0xffffffff
    cmd[8]  = 0x00000000
    cmd[9]  = 0x00000000
    cmd[10] = 0x00000001  # CNS=1
    cmd[11] = 0x00000000
    cmd[12] = 0x00000000
    cmd[13] = 0x00000000
    cmd[14] = 0x00000000
    cmd[15] = 0x00000000
    return cmd

def hostmem_wr_cmd(bus, asq_base, slot, cmd_dwords, hostmem_base=0x10000000):
    addr = asq_base + slot*64
    for i, w in enumerate(cmd_dwords):
        hostmem_wr32(bus, addr + i*4, w, base=hostmem_base)

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
    parser.add_argument("--cfg-timeout-ms", default="100", help="CFG transaction timeout (ms).")

    # Actions.
    parser.add_argument("--info",        action="store_true", help="Read/decode CAP/VS/INTMS/INTMC/CC/CSTS/AQA/ASQ/ACQ.")
    parser.add_argument("--dump",        default=None, help="Dump first N bytes of BAR0 regs (hex/dec).")
    parser.add_argument("--disable",     action="store_true", help="Clear CC.EN and wait CSTS.RDY=0.")
    parser.add_argument("--mmio-check",  action="store_true", help="Run safe MMIO tests that do not require DMA.")
    parser.add_argument("--doorbells",   action="store_true", help="Read a few SQ/CQ doorbells (read-only sanity).")
    parser.add_argument("--max-q",       default="4", help="Max queue id to probe for --doorbells.")

    parser.add_argument("--identify", action="store_true", help="Send Admin Identify (controller) via BRAM hostmem.")
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

    if not (args.info or args.dump or args.disable or args.mmio_check or args.doorbells or args.identify):
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
    if args.info or args.disable or args.mmio_check or args.doorbells or args.identify:
        info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)

    if args.dump is not None:
        length = int(args.dump, 0)
        nvme_dump_regs(bus, bar0, length=length, stride=4, timeout_ms=timeout_ms)

    if args.disable:
        cap = None if info is None else info["cap"]
        ok = nvme_disable(bus, bar0, cap=cap, timeout_ms=timeout_ms)
        print(f"Disable: {'OK' if ok else 'FAIL'}")

    if args.mmio_check:
        ok = nvme_check_mmio_basics(bus, bar0, timeout_ms=timeout_ms)
        print("MMIO checks: " + ("OK" if ok else "FAIL"))

    if args.doorbells:
        if info is None:
            info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)
        nvme_check_doorbells(bus, bar0, cap=info["cap"], timeout_ms=timeout_ms, max_q=max_q)

    if args.identify:
        if info is None:
            info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)

        hostmem_base = int(args.hostmem_base, 0)
        asq_addr     = int(args.asq_addr, 0)
        acq_addr     = int(args.acq_addr, 0)
        id_buf       = int(args.id_buf, 0)
        cid          = int(args.cid, 0)
        q_entries    = int(args.q_entries, 0)

        # Clear ACQ[0] + some of Identify buffer for visibility.
        hostmem_fill(bus, acq_addr, 16, value=0, base=hostmem_base)
        hostmem_fill(bus, id_buf,  0x100, value=0, base=hostmem_base)

        # Init admin queues + enable.
        ok = nvme_admin_init(bus, bar0, info["cap"], asq_addr, acq_addr, q_entries=q_entries, timeout_ms=timeout_ms)

        csts, _ = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
        print(f"Admin init: RDY={'1' if csts_rdy(csts) else '0'} CFS={csts_cfs(csts)} (poll_ok={'1' if ok else '0'})")

        # Program Identify command in ASQ[0].
        cmd = nvme_cmd_identify_controller(cid=cid, prp1=id_buf, nsid=0)
        hostmem_wr_cmd(bus, asq_base=asq_addr, slot=0, cmd_dwords=cmd, hostmem_base=hostmem_base)
        print("ASQ[0] programmed (Identify Controller).")

        # Ring SQ0 tail = 1.
        nvme_ring_admin_sq(bus, bar0, info["cap"], tail=1, timeout_ms=timeout_ms)
        print("SQ0 doorbell rung (tail=1).")

        # Poll CQE0 (NVMe should MemWr it).
        cpl = nvme_poll_acq0(bus, acq_base=acq_addr, timeout_s=2.0, hostmem_base=hostmem_base)
        print("ACQ[0] raw:")
        if cpl is None:
            print("  (no completion observed)")
        else:
            print(f"  {cpl[0]:08x} {cpl[1]:08x} {cpl[2]:08x} {cpl[3]:08x}")

        # Dump first 0x100 bytes of Identify buffer (NVMe writes 4096B).
        print("Identify buffer (first 0x100):")
        hostmem_dump(bus, addr=id_buf, length=0x100, base=hostmem_base)

if __name__ == "__main__":
    main()
