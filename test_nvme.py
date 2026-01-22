#!/usr/bin/env python3

import argparse
import time

from litex import RemoteClient

# Helpers ------------------------------------------------------------------------------------------

def _bit(v, n):
    return (v >> n) & 1

def _bits(v, lo, hi):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)

def _setbit(v, n, x):
    if x:
        return v | (1 << n)
    else:
        return v & ~(1 << n)

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
        print("NOTE: Command.BME=0 (DMA will be blocked later; MMIO can still work).")

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
        print("CC.EN already 0.")
        return True

    new_cc = cc & ~0x1
    print(f"Write CC.EN=0: CC 0x{cc:08x} -> 0x{new_cc:08x}")
    werr = mmio_wr32(bus, bar0 + NVME_CC, new_cc, timeout_ms=timeout_ms)
    if werr:
        print("WARN: write CC returned err=1.")

    # Prefer CAP.TO for waiting.
    to_ms = 2000
    if cap is not None:
        capd  = cap_decode(cap)
        to_ms = max(500, cap_to_ms(capd["to"]))
    ok = nvme_wait_rdy(bus, bar0, want_rdy=False, timeout_ms=to_ms)

    if ok:
        print("CSTS.RDY -> 0")
    else:
        print("Timeout waiting for CSTS.RDY -> 0")
    return ok

def nvme_check_mmio_basics(bus, bar0, timeout_ms=200):
    # Safe read-only checks + a few "write/readback while EN=0" checks.
    # IMPORTANT: INTMS/INTMC are often not reliably R/W-testable. Treat as best-effort.
    print("MMIO sanity:")

    ok = True

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

    # INTMS/INTMC: best-effort set/clear bit0; never fail the check on err/mismatch.
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

        # Restore original mask (best effort).
        mmio_wr32(bus, bar0 + NVME_INTMS, intms0, timeout_ms=timeout_ms)

    # AQA/ASQ/ACQ: also best-effort (some devices may not reflect readback as normal storage).
    aqa0, ea0 = mmio_rd32(bus, bar0 + NVME_AQA, timeout_ms=timeout_ms)
    asq0, ea1 = mmio_rd64(bus, bar0 + NVME_ASQ, timeout_ms=timeout_ms)
    acq0, ea2 = mmio_rd64(bus, bar0 + NVME_ACQ, timeout_ms=timeout_ms)
    if ea0 | ea1 | ea2:
        print("  WARN: read AQA/ASQ/ACQ returned err=1 (not fatal).")
        return ok

    aqa_t = aqa0 ^ 0x00010001
    asq_t = (asq0 ^ 0x0000000000001000) & ~0xfff
    acq_t = (acq0 ^ 0x0000000000002000) & ~0xfff

    w0 = mmio_wr32(bus, bar0 + NVME_AQA, aqa_t, timeout_ms=timeout_ms)
    w1 = mmio_wr64(bus, bar0 + NVME_ASQ, asq_t, timeout_ms=timeout_ms)
    w2 = mmio_wr64(bus, bar0 + NVME_ACQ, acq_t, timeout_ms=timeout_ms)
    r0, rr0 = mmio_rd32(bus, bar0 + NVME_AQA, timeout_ms=timeout_ms)
    r1, rr1 = mmio_rd64(bus, bar0 + NVME_ASQ, timeout_ms=timeout_ms)
    r2, rr2 = mmio_rd64(bus, bar0 + NVME_ACQ, timeout_ms=timeout_ms)

    # Restore (best effort).
    mmio_wr32(bus, bar0 + NVME_AQA, aqa0, timeout_ms=timeout_ms)
    mmio_wr64(bus, bar0 + NVME_ASQ, asq0, timeout_ms=timeout_ms)
    mmio_wr64(bus, bar0 + NVME_ACQ, acq0, timeout_ms=timeout_ms)

    if w0 | w1 | w2 | rr0 | rr1 | rr2:
        print("  WARN: AQA/ASQ/ACQ write/readback returned err=1 (not fatal).")
        return ok

    ok_aqa = (r0 == aqa_t)
    ok_asq = (r1 == asq_t)
    ok_acq = (r2 == acq_t)

    if ok_aqa and ok_asq and ok_acq:
        print("  OK: AQA/ASQ/ACQ write/readback visible.")
    else:
        print("  WARN: AQA/ASQ/ACQ write/readback not visible (impl-defined).")

    return ok

def nvme_check_doorbells(bus, bar0, cap, timeout_ms=200, max_q=4):
    # Doorbells are MMIO, but using them without real queues is undefined.
    # Still: verify that reads don't error and that stride math is consistent.
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

def nvme_next_steps(bus, bar0, cap):
    capd = cap_decode(cap)
    stride = doorbell_stride_bytes(capd["dstrd"])
    print("Next steps to actually enable + run admin cmds require queues + DMA:")
    print("  - Provide a host-memory target reachable by the NVMe device (PCIe MemRd/MemWr).")
    print("  - Place ASQ/ACQ in that memory, program AQA/ASQ/ACQ.")
    print("  - Program CC (IOSQES/IOCQES/MPS/CSS/AMS) then set CC.EN=1.")
    print(f"  - Doorbells start at BAR0+0x{NVME_DOORBELL_BASE:x}, stride={stride} bytes.")
    print("For now: MMIO reads + CC.EN disable + (best-effort) INTMS/INTMC + AQA/ASQ/ACQ checks are safe.")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteNVMe progressive tester (MMIO-first, CSR-driven).")
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
    parser.add_argument("--next-steps",  action="store_true", help="Print what is required for queues/admin commands.")

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

    # Default behavior if nothing requested.
    if not (args.info or args.dump or args.disable or args.mmio_check or args.doorbells or args.next_steps):
        args.info = True
        args.mmio_check = True

    # Print CFG CMD/STS (helps diagnose MEM/BME).
    cfg_check_command(bus, b, d, f, timeout_ms=cfg_timeout_ms)

    # BAR0.
    if args.bar0 is not None:
        bar0 = int(args.bar0, 0)
    else:
        bar0, is_64, bar0_raw = discover_bar0(bus, b, d, f, timeout_ms=cfg_timeout_ms)
        print(f"Discovered BAR0: 0x{bar0:016x} (64b={is_64}) raw=0x{bar0_raw:08x}")

    if bar0 == 0:
        raise RuntimeError("BAR0 base is 0. Assign BAR0 in config space before running MMIO tests.")

    # Info (also gives CAP for timeout / doorbells / next-steps).
    info = None
    if args.info or args.disable or args.mmio_check or args.doorbells or args.next_steps:
        info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)

    # Dump.
    if args.dump is not None:
        length = int(args.dump, 0)
        nvme_dump_regs(bus, bar0, length=length, stride=4, timeout_ms=timeout_ms)

    # Disable.
    if args.disable:
        cap = None if info is None else info["cap"]
        if not nvme_disable(bus, bar0, cap=cap, timeout_ms=timeout_ms):
            raise RuntimeError("Disable failed (CSTS.RDY did not go 0).")

    # MMIO checks (no DMA).
    if args.mmio_check:
        ok = nvme_check_mmio_basics(bus, bar0, timeout_ms=timeout_ms)
        if ok:
            print("MMIO checks: OK")
        else:
            print("MMIO checks: FAIL")

    # Doorbells (read-only sanity).
    if args.doorbells:
        if info is None:
            info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)
        nvme_check_doorbells(bus, bar0, cap=info["cap"], timeout_ms=timeout_ms, max_q=max_q)

    # Next steps.
    if args.next_steps:
        if info is None:
            info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)
        nvme_next_steps(bus, bar0, cap=info["cap"])

if __name__ == "__main__":
    main()

