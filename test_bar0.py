#!/usr/bin/env python3

import argparse
import time

from litex import RemoteClient

# Helpers ------------------------------------------------------------------------------------------

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
            #if err:
            #    raise RuntimeError("MEM transaction failed (err=1).")
            return
        if time.time() > deadline:
            raise TimeoutError("MEM transaction timeout (done=0).")

def mem_rd32(bus, addr, timeout_ms=200):
    mem_set_addr(bus, addr)
    mem_start(bus, we=0, wsel=0xf, length=1)
    mem_wait_done(bus, timeout_ms=timeout_ms)
    return bus.regs.mmio_mem_rdata.read()

def mem_wr32(bus, addr, data, wsel=0xf, timeout_ms=200):
    mem_set_addr(bus, addr)
    bus.regs.mmio_mem_wdata.write(data & 0xffffffff)
    mem_start(bus, we=1, wsel=wsel, length=1)
    mem_wait_done(bus, timeout_ms=timeout_ms)

def dump_bar0(bus, base, length, stride=4):
    # length in bytes.
    for off in range(0, length, stride):
        v = mem_rd32(bus, base + off)
        print(f"0x{off:08x}: 0x{v:08x}")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe BAR0 MMIO tester (CSR-driven MemRd/MemWr).")
    parser.add_argument("--csr-csv", default="csr.csv", help="CSR configuration file.")
    parser.add_argument("--port",    default="1234",    help="Host bind port.")
    parser.add_argument("--addr",    default=None,      help="Absolute PCIe MMIO address (hex).")
    parser.add_argument("--read",    action="store_true", help="Read 32-bit at --addr.")
    parser.add_argument("--write",   default=None,      help="Write 32-bit value at --addr (hex).")
    parser.add_argument("--wsel",    default="0xf",     help="Write byte enable mask (hex, 0..f).")

    parser.add_argument("--bar0",    default=None,      help="BAR0 base address (hex).")
    parser.add_argument("--dump",    default=None,      help="Dump length in bytes from BAR0 base (hex/dec).")
    parser.add_argument("--stride",  default="4",       help="Dump stride in bytes (default 4).")

    args = parser.parse_args()

    bus = RemoteClient(csr_csv=args.csr_csv, port=int(args.port, 0))
    bus.open()

    wsel = int(args.wsel, 0)
    stride = int(args.stride, 0)

    if args.read or (args.write is not None):
        if args.addr is None:
            raise ValueError("--addr is required for --read/--write.")
        addr = int(args.addr, 0)

        if args.write is not None:
            data = int(args.write, 0)
            mem_wr32(bus, addr, data, wsel=wsel)
            print(f"W 0x{addr:016x} = 0x{data:08x} (wsel=0x{wsel:x})")

        if args.read:
            v = mem_rd32(bus, addr)
            print(f"R 0x{addr:016x} = 0x{v:08x}")

    if args.dump is not None:
        if args.bar0 is None:
            raise ValueError("--bar0 is required for --dump.")
        base = int(args.bar0, 0)
        length = int(args.dump, 0)
        dump_bar0(bus, base, length, stride=stride)

if __name__ == "__main__":
    main()
