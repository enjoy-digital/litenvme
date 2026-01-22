#!/usr/bin/env python3

import argparse
import time

from litex import RemoteClient

# Helpers ------------------------------------------------------------------------------------------

def cfg_bdf_pack(bus, dev, fn, reg, ext=0):
    v  = (bus & 0xff) << 0
    v |= (dev & 0x1f) << 8
    v |= (fn  & 0x07) << 13
    v |= (reg & 0x3f) << 16
    v |= (ext & 0x07) << 22
    return v

def cfg_rd0(bus, b, d, f, reg_dword, ext=0, timeout_ms=100):
    bus.regs.pcie_cfgm_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
    bus.regs.pcie_cfgm_cfg_ctrl.write(1)
    bus.regs.pcie_cfgm_cfg_ctrl.write(0)

    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.pcie_cfgm_cfg_stat.read()
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            if err:
                raise RuntimeError("CFG read failed (err=1).")
            return bus.regs.pcie_cfgm_cfg_rdata.read()
        if time.time() > deadline:
            raise TimeoutError("CFG read timeout (done=0).")

def cfg_wr0(bus, b, d, f, reg_dword, wdata, ext=0, timeout_ms=100):
    bus.regs.pcie_cfgm_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))
    bus.regs.pcie_cfgm_cfg_wdata.write(wdata & 0xffffffff)
    bus.regs.pcie_cfgm_cfg_ctrl.write(1 | (1<<1))  # start=1, we=1
    bus.regs.pcie_cfgm_cfg_ctrl.write(0)

    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.pcie_cfgm_cfg_stat.read()
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            #if err:
            #    raise RuntimeError("CFG write failed (err=1).")
            return
        if time.time() > deadline:
            raise TimeoutError("CFG write timeout (done=0).")

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

def _bit(v, n):
    return (v >> n) & 1

def _bits(v, lo, hi):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)

def _setbit(v, n, x):
    if x:
        return v | (1 << n)
    else:
        return v & ~(1 << n)

# Decoders -----------------------------------------------------------------------------------------

def decode_common_header(cfg):
    d0 = cfg[0x00//4]
    vid = _bits(d0, 0, 15)
    did = _bits(d0, 16, 31)

    d1 = cfg[0x04//4]
    cmd = _bits(d1, 0, 15)
    sts = _bits(d1, 16, 31)

    d2 = cfg[0x08//4]
    rev  = _bits(d2, 0, 7)
    prog = _bits(d2, 8, 15)
    subc = _bits(d2, 16, 23)
    cls  = _bits(d2, 24, 31)

    d3 = cfg[0x0c//4]
    cache_line = _bits(d3, 0, 7)
    lat_timer  = _bits(d3, 8, 15)
    hdr_type   = _bits(d3, 16, 23)
    bist       = _bits(d3, 24, 31)

    htype = hdr_type & 0x7f
    mfd   = (hdr_type >> 7) & 1

    print(f"VID:DID        0x{vid:04x}:0x{did:04x}")
    print(f"Class          0x{cls:02x}:0x{subc:02x}:0x{prog:02x}  rev=0x{rev:02x}")
    print(f"HeaderType     0x{htype:02x} (mfd={mfd})")
    print(f"BIST           0x{bist:02x}, CacheLine=0x{cache_line:02x}, Latency=0x{lat_timer:02x}")

    cmd_mem = _bit(cmd, 1)
    cmd_bme = _bit(cmd, 2)
    cmd_int = _bit(cmd, 10)
    print(f"Command        0x{cmd:04x} (MEM={cmd_mem} BME={cmd_bme} INTDIS={cmd_int})")

    sts_cap = _bit(sts, 4)
    print(f"Status         0x{sts:04x} (CAP={sts_cap})")

    return {
        "vid": vid, "did": did,
        "cmd": cmd, "sts": sts,
        "cls": cls, "subc": subc, "prog": prog, "rev": rev,
        "htype": htype, "mfd": mfd,
        "cap_list": sts_cap,
    }

def decode_type0_header(cfg):
    bars = []
    for i in range(6):
        bars.append(cfg[(0x10//4) + i])

    d_sub = cfg[0x2c//4]
    ssvid = _bits(d_sub, 0, 15)
    ssid  = _bits(d_sub, 16, 31)

    cap_ptr = _bits(cfg[0x34//4], 0, 7)

    d_irq = cfg[0x3c//4]
    int_line = _bits(d_irq, 0, 7)
    int_pin  = _bits(d_irq, 8, 15)

    print(f"Subsystem      SVID:SSID 0x{ssvid:04x}:0x{ssid:04x}")
    print(f"CapPtr         0x{cap_ptr:02x}")
    print(f"Interrupt      Line=0x{int_line:02x} Pin=0x{int_pin:02x}")

    for i, bar in enumerate(bars):
        if bar == 0x00000000:
            print(f"BAR{i}           0x{bar:08x} (unassigned)")
            continue
        if bar & 1:
            base = bar & 0xfffffffc
            print(f"BAR{i}           0x{bar:08x} (I/O base=0x{base:08x})")
        else:
            mem_type = (bar >> 1) & 0x3
            prefetch = (bar >> 3) & 1
            base_lo  = bar & 0xfffffff0
            if mem_type == 0x2:
                print(f"BAR{i}           0x{bar:08x} (MEM64 lo=0x{base_lo:08x}, prefetch={prefetch})")
            elif mem_type == 0x0:
                print(f"BAR{i}           0x{bar:08x} (MEM32 base=0x{base_lo:08x}, prefetch={prefetch})")
            else:
                print(f"BAR{i}           0x{bar:08x} (MEM type={mem_type}, prefetch={prefetch})")

    return cap_ptr

def cap_name(cap_id):
    return {
        0x01: "PM",
        0x03: "VPD",
        0x05: "MSI",
        0x10: "PCIe",
        0x11: "MSI-X",
        0x12: "SATA",
        0x13: "AF",
        0x14: "EA",
    }.get(cap_id, "UNKNOWN")

def dump_caps(bus, b, d, f, cap_ptr, max_caps=32):
    seen = set()
    ptr  = cap_ptr

    print("Capabilities:")
    if ptr == 0x00:
        print("  (none)")
        return

    for _ in range(max_caps):
        if ptr in seen:
            print(f"  0x{ptr:02x}: (loop detected, stop)")
            return
        seen.add(ptr)

        if (ptr & 0x3) != 0:
            print(f"  0x{ptr:02x}: (bad alignment, stop)")
            return

        reg = ptr // 4
        d0  = cfg_rd0(bus, b, d, f, reg)
        cap_id  = _bits(d0, 0, 7)
        next_ptr = _bits(d0, 8, 15)
        name = cap_name(cap_id)
        print(f"  0x{ptr:02x}: id=0x{cap_id:02x} ({name}) next=0x{next_ptr:02x}")

        if next_ptr == 0x00:
            return
        ptr = next_ptr

    print("  (cap walk limit reached)")

# NVMe/Bring-up helpers ----------------------------------------------------------------------------

def bar_size_from_mask(mask64):
    return ((~mask64) + 1) & 0xffffffffffffffff

def size_and_assign_bar0(bus, b, d, f, base_addr, dry_run=False):
    bar0 = cfg_rd0(bus, b, d, f, 4)  # 0x10
    is_io = bar0 & 1
    if is_io:
        raise RuntimeError("BAR0 is I/O, unexpected for NVMe.")
    bar_type = (bar0 >> 1) & 0x3
    is_64 = (bar_type == 0x2)

    print(f"BAR0 orig    = 0x{bar0:08x} (64b={is_64})")

    if dry_run:
        print("BAR0 sizing/assignment skipped (dry-run).")
        return 0, is_64

    # Size probe.
    cfg_wr0(bus, b, d, f, 4, 0xffffffff)
    bar0_m = cfg_rd0(bus, b, d, f, 4)
    if is_64:
        cfg_wr0(bus, b, d, f, 5, 0xffffffff)
        bar1_m = cfg_rd0(bus, b, d, f, 5)
        mask = ((bar1_m << 32) | (bar0_m & 0xfffffff0)) & 0xffffffffffffffff
    else:
        mask = (bar0_m & 0xfffffff0) & 0xffffffff

    size = bar_size_from_mask(mask)
    print(f"BAR0 size    = 0x{size:x}")

    # Assign.
    if size == 0 or (size & (size - 1)) != 0:
        raise RuntimeError("BAR size invalid (not power-of-two).")
    if base_addr & (size - 1):
        raise RuntimeError("Base address not aligned to BAR size.")

    if is_64:
        cfg_wr0(bus, b, d, f, 4, (base_addr & 0xffffffff) | (bar0 & 0xf))
        cfg_wr0(bus, b, d, f, 5, (base_addr >> 32) & 0xffffffff)
        rb0 = cfg_rd0(bus, b, d, f, 4)
        rb1 = cfg_rd0(bus, b, d, f, 5)
        print(f"BAR0/BAR1    = 0x{rb0:08x} / 0x{rb1:08x}")
    else:
        cfg_wr0(bus, b, d, f, 4, (base_addr & 0xffffffff) | (bar0 & 0xf))
        rb0 = cfg_rd0(bus, b, d, f, 4)
        print(f"BAR0         = 0x{rb0:08x}")

    return size, is_64

def enable_command_bits(bus, b, d, f, mem=None, bme=None, intdis=None, dry_run=False):
    v = cfg_rd0(bus, b, d, f, 1)  # 0x04
    cmd = v & 0xffff
    sts = (v >> 16) & 0xffff

    cur_mem    = _bit(cmd, 1)
    cur_bme    = _bit(cmd, 2)
    cur_intdis = _bit(cmd, 10)

    print(f"CMD/STS      = 0x{cmd:04x} / 0x{sts:04x} (MEM={cur_mem} BME={cur_bme} INTDIS={cur_intdis})")

    new_cmd = cmd
    if mem    is not None: new_cmd = _setbit(new_cmd, 1, 1 if mem else 0)
    if bme    is not None: new_cmd = _setbit(new_cmd, 2, 1 if bme else 0)
    if intdis is not None: new_cmd = _setbit(new_cmd,10, 1 if intdis else 0)

    if new_cmd == cmd:
        print("CMD unchanged.")
        return

    v_new = (v & 0xffff0000) | (new_cmd & 0xffff)
    print(f"CMD new      = 0x{new_cmd:04x}")

    if dry_run:
        print("CMD write skipped (dry-run).")
        return

    cfg_wr0(bus, b, d, f, 1, v_new)

    v_rb = cfg_rd0(bus, b, d, f, 1)
    cmd_rb = v_rb & 0xffff
    sts_rb = (v_rb >> 16) & 0xffff
    print(f"CMD/STS      = 0x{cmd_rb:04x} / 0x{sts_rb:04x} (MEM={_bit(cmd_rb,1)} BME={_bit(cmd_rb,2)} INTDIS={_bit(cmd_rb,10)})")

# Dumpers ------------------------------------------------------------------------------------------

def dump_cfg_dwords(bus, b, d, f, dwords=64, ext=0):
    cfg = []
    for i in range(dwords):
        cfg.append(cfg_rd0(bus, b, d, f, i, ext=ext))
    return cfg

def print_cfg_hex(cfg, base=0x00):
    for i, v in enumerate(cfg):
        off = base + (i * 4)
        print(f"  0x{off:02x}: 0x{v:08x}")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe CFG dumper + manual bring-up (UARTBone/LiteX server).")
    parser.add_argument("--csr-csv",   default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",      default="1234",    help="Host bind port.")
    parser.add_argument("--b",         default="0",       help="Bus number.")
    parser.add_argument("--d",         default="0",       help="Device number.")
    parser.add_argument("--f",         default="0",       help="Function number.")
    parser.add_argument("--wait-link", action="store_true", help="Wait for PCIe link up before issuing CFG.")

    parser.add_argument("--dump-all",  action="store_true", help="Dump config dwords + decode header.")
    parser.add_argument("--dump-len",  default="64", help="Number of dwords to dump (default: 64 = 0x00..0xff).")
    parser.add_argument("--caps",      action="store_true", help="Walk and decode capability list.")

    # Bring-up actions (manual)
    parser.add_argument("--dry-run",   action="store_true", help="Do not write anything.")
    parser.add_argument("--bar0-assign", action="store_true", help="Probe BAR0 size and assign BAR0/BAR1.")
    parser.add_argument("--bar0-base", default="0xe0000000", help="Base address for BAR0 (must be aligned).")

    parser.add_argument("--enable-mem", action="store_true", help="Set Command.MEM=1.")
    parser.add_argument("--enable-bme", action="store_true", help="Set Command.BME=1.")
    parser.add_argument("--disable-intx", action="store_true", help="Set Command.INTDIS=1 (disable legacy INTx).")

    args = parser.parse_args()

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)
    dump_len  = int(args.dump_len, 0)
    bar0_base = int(args.bar0_base, 0)

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    # Default behavior (read-only) if no actions requested.
    if not (args.dump_all or args.caps or args.bar0_assign or args.enable_mem or args.enable_bme or args.disable_intx):
        args.dump_all = True
        args.caps = True

    # Dump/decode.
    cap_ptr = 0x00
    if args.dump_all:
        cfg = dump_cfg_dwords(bus, b, d, f, dwords=dump_len)
        print("Config space:")
        print_cfg_hex(cfg)

        print("Decode:")
        info = decode_common_header(cfg)
        if info["htype"] == 0x00:
            cap_ptr = decode_type0_header(cfg)
        else:
            print(f"(HeaderType 0x{info['htype']:02x} not decoded here)")

        if args.caps and info["cap_list"]:
            dump_caps(bus, b, d, f, cap_ptr)

    # Manual bring-up steps.
    if args.bar0_assign:
        size_and_assign_bar0(bus, b, d, f, base_addr=bar0_base, dry_run=args.dry_run)

    if args.enable_mem or args.enable_bme or args.disable_intx:
        mem    = True if args.enable_mem   else None
        bme    = True if args.enable_bme   else None
        intdis = True if args.disable_intx else None
        enable_command_bits(bus, b, d, f, mem=mem, bme=bme, intdis=intdis, dry_run=args.dry_run)

if __name__ == "__main__":
    main()
