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
            if err:
                raise RuntimeError("CFG write failed (err=1).")
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

# Decoders -----------------------------------------------------------------------------------------

def decode_common_header(cfg):
    # cfg is a list of DWORDs starting at 0x00.
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

    # CMD bits (common ones)
    cmd_mem = _bit(cmd, 1)
    cmd_bme = _bit(cmd, 2)
    cmd_int = _bit(cmd, 10)
    print(f"Command        0x{cmd:04x} (MEM={cmd_mem} BME={cmd_bme} INTDIS={cmd_int})")

    # STS bits (common ones)
    sts_cap = _bit(sts, 4)
    sts_m66 = _bit(sts, 5)
    sts_fbt = _bit(sts, 7)
    sts_mdp = _bit(sts, 8)
    print(f"Status         0x{sts:04x} (CAP={sts_cap} 66MHz={sts_m66} FB2B={sts_fbt} DEVSEL={_bits(sts,9,10)} MDP={sts_mdp})")

    return {
        "vid": vid, "did": did,
        "cmd": cmd, "sts": sts,
        "cls": cls, "subc": subc, "prog": prog, "rev": rev,
        "htype": htype, "mfd": mfd,
        "cap_list": sts_cap,
    }

def decode_type0_header(cfg):
    # BAR0..BAR5
    bars = []
    for i in range(6):
        bars.append(cfg[(0x10//4) + i])

    # Subsystem IDs
    d_sub = cfg[0x2c//4]
    ssvid = _bits(d_sub, 0, 15)
    ssid  = _bits(d_sub, 16, 31)

    # Cap pointer
    cap_ptr = _bits(cfg[0x34//4], 0, 7)

    # IRQ line/pin
    d_irq = cfg[0x3c//4]
    int_line = _bits(d_irq, 0, 7)
    int_pin  = _bits(d_irq, 8, 15)

    print(f"Subsystem      SVID:SSID 0x{ssvid:04x}:0x{ssid:04x}")
    print(f"CapPtr         0x{cap_ptr:02x}")
    print(f"Interrupt      Line=0x{int_line:02x} Pin=0x{int_pin:02x}")

    # BAR decode (basic)
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

        # Minimal per-cap decoding (kept short but useful)
        if cap_id == 0x05:  # MSI
            msg_ctl = _bits(d0, 16, 31)
            ena = _bit(msg_ctl, 0)
            mmc = _bits(msg_ctl, 1, 3)
            mme = _bits(msg_ctl, 4, 6)
            is64 = _bit(msg_ctl, 7)
            print(f"           MSI: en={ena} mmc=2^{mmc} mme=2^{mme} 64b={is64}")
        elif cap_id == 0x11:  # MSI-X
            msg_ctl = _bits(d0, 16, 31)
            ena = _bit(msg_ctl, 15)
            mask = _bit(msg_ctl, 14)
            tbl_sz = _bits(msg_ctl, 0, 10) + 1
            print(f"           MSI-X: en={ena} maskall={mask} table_size={tbl_sz}")
        elif cap_id == 0x10:  # PCIe
            # Cap register is at offset +0x02 within the cap (same dword high 16)
            pcie_cap = _bits(d0, 16, 31)
            ver = _bits(pcie_cap, 0, 3)
            devport = _bits(pcie_cap, 4, 7)
            print(f"           PCIe: ver={ver} devport=0x{devport:x}")

        if next_ptr == 0x00:
            return
        ptr = next_ptr

    print("  (cap walk limit reached)")

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
    parser = argparse.ArgumentParser(description="LitePCIe CFG space dumper/decoder (UARTBone/LiteX server).")
    parser.add_argument("--csr-csv",   default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",      default="1234",    help="Host bind port.")
    parser.add_argument("--b",         default="0",       help="Bus number.")
    parser.add_argument("--d",         default="0",       help="Device number.")
    parser.add_argument("--f",         default="0",       help="Function number.")
    parser.add_argument("--wait-link", action="store_true", help="Wait for PCIe link up before issuing CFG.")

    parser.add_argument("--dump-all",  action="store_true", help="Dump config dwords + decode header.")
    parser.add_argument("--dump-len",  default="64", help="Number of dwords to dump (default: 64 = 0x00..0xff).")
    parser.add_argument("--caps",      action="store_true", help="Walk and decode capability list.")

    args = parser.parse_args()

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)
    dump_len = int(args.dump_len, 0)

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    if not (args.dump_all or args.caps):
        args.dump_all = True
        args.caps = True

    if args.dump_all:
        cfg = dump_cfg_dwords(bus, b, d, f, dwords=dump_len)
        print("Config space:")
        print_cfg_hex(cfg)

        print("Decode:")
        info = decode_common_header(cfg)
        cap_ptr = 0x00
        if info["htype"] == 0x00:
            cap_ptr = decode_type0_header(cfg)
        else:
            print(f"(HeaderType 0x{info['htype']:02x} not decoded here)")

        if args.caps and info["cap_list"]:
            dump_caps(bus, b, d, f, cap_ptr)

if __name__ == "__main__":
    main()
