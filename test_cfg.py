#!/usr/bin/env python3

import argparse
import time

from litex import RemoteClient

# Helpers ------------------------------------------------------------------------------------------

def cfg_bdf_pack(bus, dev, fn, reg, ext=0):
    # Matches LitePCIeCFGMaster CSR layout:
    # bus  [ 7: 0]
    # dev  [12: 8]
    # fn   [15:13]
    # reg  [21:16] (DWORD index)
    # ext  [24:22]
    v  = (bus & 0xff) << 0
    v |= (dev & 0x1f) << 8
    v |= (fn  & 0x07) << 13
    v |= (reg & 0x3f) << 16
    v |= (ext & 0x07) << 22
    return v

def cfg_rd0(bus, b, d, f, reg_dword, ext=0, timeout_ms=100):
    # Program request.
    bus.regs.pcie_cfgm_cfg_bdf.write(cfg_bdf_pack(b, d, f, reg_dword, ext))

    # Pulse start (1 -> 0).
    bus.regs.pcie_cfgm_cfg_ctrl.write(1)
    bus.regs.pcie_cfgm_cfg_ctrl.write(0)

    # Poll done.
    deadline = time.time() + (timeout_ms / 1000.0)
    while True:
        stat = bus.regs.pcie_cfgm_cfg_stat.read()
        print(stat)
        done = (stat >> 0) & 1
        err  = (stat >> 1) & 1
        if done:
            if err:
                raise RuntimeError("CFG read failed (err=1).")
            return bus.regs.pcie_cfgm_cfg_rdata.read()
        if time.time() > deadline:
            raise TimeoutError("CFG read timeout (done=0).")

def wait_link_up(bus, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while True:
        v = bus.regs.pcie_phy_phy_link_status.read()
        # This is a CSRStatus with fields; in csv it's a single register.
        # On LitePCIe PHY, bit0 is typically "status" (link up). Adjust if needed.
        link = (v >> 0) & 1
        if link:
            return True
        if time.time() > deadline:
            return False
        time.sleep(0.05)

# Test ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe CFG RD0 exerciser (UARTBone/LiteX server).")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",    default="1234",    help="Host bind port.")
    parser.add_argument("--b",       default="0",       help="Bus number.")
    parser.add_argument("--d",       default="0",       help="Device number.")
    parser.add_argument("--f",       default="0",       help="Function number.")
    parser.add_argument("--wait-link", action="store_true", help="Wait for PCIe link up before issuing CFG.")
    args = parser.parse_args()

    b = int(args.b, 0)
    d = int(args.d, 0)
    f = int(args.f, 0)

    bus = RemoteClient(
        csr_csv = args.csr_csv,
        port    = int(args.port, 0)
    )
    bus.open()

    # Optional: wait for link up.
    if args.wait_link:
        print("Waiting for link...")
        if not wait_link_up(bus):
            raise RuntimeError("PCIe link did not come up.")
        print("Link up.")

    # -------------------------------------------------------------------------
    # Basic smoke tests:
    # - 0x00 (DWORD 0): Vendor ID / Device ID
    # - 0x08 (DWORD 2): Class Code / Revision
    # -------------------------------------------------------------------------

    v = cfg_rd0(bus, b, d, f, reg_dword=0)  # 0x00/4
    vendor_id = (v >> 0)  & 0xffff
    device_id = (v >> 16) & 0xffff
    print(f"CFG[0x00] Vendor: 0x{vendor_id:04x}, Device: 0x{device_id:04x} (raw=0x{v:08x})")

    v = cfg_rd0(bus, b, d, f, reg_dword=2)  # 0x08/4
    rev_id    = (v >> 0)  & 0xff
    prog_if   = (v >> 8)  & 0xff
    subclass  = (v >> 16) & 0xff
    classcode = (v >> 24) & 0xff
    print(f"CFG[0x08] Class: 0x{classcode:02x}, Sub: 0x{subclass:02x}, IF: 0x{prog_if:02x}, Rev: 0x{rev_id:02x} (raw=0x{v:08x})")

if __name__ == "__main__":
    main()
