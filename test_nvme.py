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
            #if err:
            #    raise RuntimeError("CFG read failed (err=1).")
            return bus.regs.cfg_cfg_rdata.read()
        if time.time() > deadline:
            raise TimeoutError("CFG read timeout (done=0).")

def discover_bar0(bus, b, d, f):
    # BAR0: dword 4 (0x10), BAR1: dword 5 (0x14).
    bar0 = cfg_rd0(bus, b, d, f, 4)
    if bar0 & 1:
        raise RuntimeError("BAR0 is I/O, unexpected for NVMe.")
    bar_type = (bar0 >> 1) & 0x3
    is_64 = (bar_type == 0x2)

    base_lo = bar0 & 0xfffffff0
    if is_64:
        bar1 = cfg_rd0(bus, b, d, f, 5)
        base = ((bar1 & 0xffffffff) << 32) | base_lo
    else:
        base = base_lo

    return base, is_64, bar0

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
    cc,   e2 = mmio_rd32(bus, bar0 + NVME_CC,   timeout_ms=timeout_ms)
    csts, e3 = mmio_rd32(bus, bar0 + NVME_CSTS, timeout_ms=timeout_ms)
    aqa,  e4 = mmio_rd32(bus, bar0 + NVME_AQA,  timeout_ms=timeout_ms)
    asq,  e5 = mmio_rd64(bus, bar0 + NVME_ASQ,  timeout_ms=timeout_ms)
    acq,  e6 = mmio_rd64(bus, bar0 + NVME_ACQ,  timeout_ms=timeout_ms)

    errs = e0 | e1 | e2 | e3 | e4 | e5 | e6

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
    print(f"CC             0x{cc:08x} (EN={cc_en(cc)})")
    print(f"CSTS           0x{csts:08x} (RDY={csts_rdy(csts)})")
    print(f"AQA            0x{aqa:08x}")
    print(f"ASQ            0x{asq:016x}")
    print(f"ACQ            0x{acq:016x}")

    if errs:
        print("NOTE: Some MMIO ops returned err=1 (UR/CA/etc). Check MEM/BME, BAR assignment, and offsets.")

    return {
        "cap": cap,
        "vs": vs,
        "cc": cc,
        "csts": csts,
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

def nvme_next_steps(bus, bar0, cap):
    capd = cap_decode(cap)
    stride = doorbell_stride_bytes(capd["dstrd"])
    print("Next steps to actually enable + run admin cmds require queues + DMA:")
    print("  - Provide a host-memory target reachable by the NVMe device (PCIe MemRd/MemWr).")
    print("  - Place ASQ/ACQ in that memory, program AQA/ASQ/ACQ.")
    print("  - Program CC (IOSQES/IOCQES/MPS/CSS/AMS) then set CC.EN=1.")
    print(f"  - Doorbells start at BAR0+0x{NVME_DOORBELL_BASE:x}, stride={stride} bytes.")
    print("For now: MMIO reads + CC.EN disable are safe checks.")

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

    # Actions.
    parser.add_argument("--info",      action="store_true", help="Read/decode CAP/VS/CC/CSTS/AQA/ASQ/ACQ.")
    parser.add_argument("--dump",      default=None, help="Dump first N bytes of BAR0 regs (hex/dec).")
    parser.add_argument("--disable",   action="store_true", help="Clear CC.EN and wait CSTS.RDY=0.")
    parser.add_argument("--next-steps", action="store_true", help="Print what is required for queues/admin commands.")

    args = parser.parse_args()

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)
    timeout_ms = int(args.timeout_ms, 0)

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    # Default behavior if nothing requested.
    if not (args.info or args.dump or args.disable or args.next_steps):
        args.info = True

    # BAR0.
    if args.bar0 is not None:
        bar0 = int(args.bar0, 0)
    else:
        bar0, is_64, bar0_raw = discover_bar0(bus, b, d, f)
        print(f"Discovered BAR0: 0x{bar0:016x} (64b={is_64}) raw=0x{bar0_raw:08x}")

    # Info (also gives CAP for timeout / next-steps).
    info = None
    if args.info or args.disable or args.next_steps:
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

    # Next steps.
    if args.next_steps:
        if info is None:
            info = nvme_read_core(bus, bar0, timeout_ms=timeout_ms)
        nvme_next_steps(bus, bar0, cap=info["cap"])

if __name__ == "__main__":
    main()
