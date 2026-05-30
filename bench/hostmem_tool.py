#!/usr/bin/env python3
"""Host-side access to the NVMe hostmem window via the CSR debug port.

Mirrors the firmware hostmem_rd32/wr32: the port is addressed by DWORD index relative to
HOSTMEM_BASE, and reads have a 1-transaction latency so the FIRST rdata after setting the
address is stale and must be discarded (documented in firmware). Used to prove end-to-end
engine data integrity (write a block, read it back through the SSD, compare).
"""
import sys, json
from litex import RemoteClient

HOSTMEM_BASE = 0x10000000
CSR = "build/alibaba_xcku3p/csr.csv"
# sample dword offsets within a 4 KiB (1024-dword) block
OFF = list(range(0, 32)) + [64, 128, 256, 512, 768, 1023]


def _open():
    b = RemoteClient(csr_csv=CSR); b.open(); return b


def hm_rd(b, addr):
    b.regs.hostmem_csr_csr_adr.write((addr - HOSTMEM_BASE) >> 2)
    b.regs.hostmem_csr_csr_rdata.read()          # discard stale (read-lag)
    return b.regs.hostmem_csr_csr_rdata.read() & 0xffffffff


def hm_wr(b, addr, data):
    b.regs.hostmem_csr_csr_adr.write((addr - HOSTMEM_BASE) >> 2)
    b.regs.hostmem_csr_csr_wdata.write(data & 0xffffffff)
    b.regs.hostmem_csr_csr_we.write(1)
    b.regs.hostmem_csr_csr_we.write(0)


def dump(addr, outfile):
    b = _open(); d = {"addr": addr, "words": {}}
    for i in OFF:
        d["words"][str(i)] = hm_rd(b, addr + 4 * i)
    b.close(); json.dump(d, open(outfile, "w"))
    print("DUMP addr=0x%08x w0=0x%08x w1=0x%08x w31=0x%08x w1023=0x%08x"
          % (addr, d["words"]["0"], d["words"]["1"], d["words"]["31"], d["words"]["1023"]))


def fill(addr, base):
    b = _open()
    for i in OFF:
        hm_wr(b, addr + 4 * i, (base + i) & 0xffffffff)
    b.close(); print("FILL addr=0x%08x base=0x%08x" % (addr, base))


def compare(fa, fb):  # expect EQUAL (read-back == written)
    a = json.load(open(fa)); c = json.load(open(fb))
    mism = 0; samples = []
    for k in a["words"]:
        va = a["words"][k]; vb = c["words"].get(k)
        if va != vb: mism += 1
        if int(k) < 4: samples.append("w%s A=0x%08x B=0x%08x %s" % (k, va, vb, "OK" if va == vb else "MISMATCH"))
    distinct = len(set(a["words"].values()))
    # PASS = read-back matches what was written. Non-triviality (that the read actually
    # changed the buffer, i.e. wasn't a stale read) is established separately by the DIFFER
    # check against the clobber pattern. A uniform written pattern (e.g. the firmware bench
    # fills 0xa5a5a5a5) is legitimate, so do NOT require multiple distinct values here.
    print("COMPARE_EQUAL mismatches=%d/%d distinctA=%d" % (mism, len(a["words"]), distinct))
    for s in samples: print("  " + s)
    print("VERDICT_EQUAL=%s" % ("PASS" if mism == 0 else "FAIL"))


def differ(fa, fb):  # expect DIFFERENT
    a = json.load(open(fa)); c = json.load(open(fb))
    diff = sum(1 for k in a["words"] if a["words"][k] != c["words"].get(k))
    print("COMPARE_DIFFER differ=%d/%d" % (diff, len(a["words"])))
    print("VERDICT_DIFFER=%s" % ("PASS" if diff > 0 else "FAIL"))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "dump":    dump(int(sys.argv[2], 0), sys.argv[3])
    elif cmd == "fill":  fill(int(sys.argv[2], 0), int(sys.argv[3], 0))
    elif cmd == "compare": compare(sys.argv[2], sys.argv[3])
    elif cmd == "differ":  differ(sys.argv[2], sys.argv[3])
    else: print("usage: hostmem_tool.py dump|fill|compare|differ ..."); sys.exit(2)
