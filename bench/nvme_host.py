# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

import time
import struct

# NVMe command builders ---------------------------------------------------------------------------

def nvme_cmd_identify_controller(cid, prp1, nsid=0):
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = nsid & 0xffffffff
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
    cq_flags = ((pc & 0x1) << 0) | ((ien & 0x1) << 1)
    cmd[11]  = ((iv & 0xffff) << 16) | (cq_flags & 0xffff)
    return cmd

def nvme_cmd_create_iosq(cid, prp1_sq, qid=1, qsize_minus1=1, cqid=1, qprio=0, pc=1):
    cmd = [0]*16
    cmd[0]  = (0x01 & 0xff) | ((cid & 0xffff) << 16)  # Create IO SQ
    cmd[6]  = prp1_sq & 0xffffffff
    cmd[7]  = (prp1_sq >> 32) & 0xffffffff
    cmd[10] = (qid & 0xffff) | ((qsize_minus1 & 0xffff) << 16)
    sq_flags = ((pc & 0x1) << 0) | ((qprio & 0x3) << 1)
    cmd[11]  = ((cqid & 0xffff) << 16) | (sq_flags & 0xffff)
    return cmd

def nvme_cmd_read(cid, nsid, prp1_data, slba, nlb_minus1):
    cmd = [0]*16
    cmd[0]  = (0x02 & 0xff) | ((cid & 0xffff) << 16)  # Read
    cmd[1]  = nsid & 0xffffffff
    cmd[6]  = prp1_data & 0xffffffff
    cmd[7]  = (prp1_data >> 32) & 0xffffffff
    cmd[10] = slba & 0xffffffff
    cmd[11] = (slba >> 32) & 0xffffffff
    cmd[12] = (nlb_minus1 & 0xffff)
    return cmd

def nvme_cmd_write(cid, nsid, prp1_data, slba, nlb_minus1):
    cmd = [0]*16
    cmd[0]  = (0x01 & 0xff) | ((cid & 0xffff) << 16)  # Write
    cmd[1]  = nsid & 0xffffffff
    cmd[6]  = prp1_data & 0xffffffff
    cmd[7]  = (prp1_data >> 32) & 0xffffffff
    cmd[10] = slba & 0xffffffff
    cmd[11] = (slba >> 32) & 0xffffffff
    cmd[12] = (nlb_minus1 & 0xffff)
    return cmd

def nvme_cmd_identify_ns_list(cid, prp1):
    cmd = [0]*16
    cmd[0]  = (0x06 & 0xff) | ((cid & 0xffff) << 16)
    cmd[1]  = 0
    cmd[6]  = prp1 & 0xffffffff
    cmd[7]  = (prp1 >> 32) & 0xffffffff
    cmd[10] = 0x00000002  # CNS=2
    return cmd

def nvme_cmd_set_features_num_queues(cid, nsqr, ncqr):
    cmd = [0]*16
    cmd[0]  = (0x09 & 0xff) | ((cid & 0xffff) << 16)
    cdw10 = 0x07
    cdw10 |= ((nsqr - 1) & 0xffff) << 0
    cdw10 |= ((ncqr - 1) & 0xffff) << 16
    cmd[10] = cdw10
    return cmd

# NVMe decoders -----------------------------------------------------------------------------------

def _bit(v, n):
    return (v >> n) & 1

def _bits(v, lo, hi):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)

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

def cqe_ok(cqe):
    return (cqe["sct"] == 0) and (cqe["sc"] == 0)

def cap_decode(cap):
    return {
        "mqes":   _bits(cap, 0, 15),
        "cqr":    _bit(cap, 16),
        "ams":    _bits(cap, 17, 18),
        "to":     _bits(cap, 24, 31),
        "dstrd":  _bits(cap, 32, 35),
        "nssrs":  _bit(cap, 36),
        "css":    _bits(cap, 37, 44),
        "mpsmin": _bits(cap, 48, 51),
        "mpsmax": _bits(cap, 52, 55),
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
    off = 0x1000 + (qid*2 + (1 if is_cq else 0)) * stride
    return bar0 + off

# NVMe host helper ---------------------------------------------------------------------------------

class NVMeHost:
    def __init__(self, bus, bar0, hostmem_base=0x10000000, timeout_ms=200):
        self.bus = bus
        self.bar0 = bar0
        self.hostmem_base = hostmem_base
        self.timeout_ms = timeout_ms

        self._asq_addr = None
        self._acq_addr = None
        self._q_entries = None
        self._acq_head = 0
        self._acq_phase = 1
        self._asq_tail = 0

        self._io_sq_addr = None
        self._io_cq_addr = None
        self._io_q_entries = None
        self._iosq_tail = 0
        self._iocq_head = 0
        self._iocq_phase = 1

    # Host memory access (CSR debug port) ---------------------------------------------------------

    def _hostmem_set_adr(self, dword_adr):
        self.bus.regs.hostmem_csr_csr_adr.write(dword_adr & 0xffffffff)

    def hostmem_wr32(self, addr, data):
        dword = (addr - self.hostmem_base) >> 2
        self._hostmem_set_adr(dword)
        self.bus.regs.hostmem_csr_csr_wdata.write(data & 0xffffffff)
        self.bus.regs.hostmem_csr_csr_we.write(1)
        self.bus.regs.hostmem_csr_csr_we.write(0)

    def hostmem_rd32(self, addr):
        dword = (addr - self.hostmem_base) >> 2
        self._hostmem_set_adr(dword)
        return self.bus.regs.hostmem_csr_csr_rdata.read()

    def hostmem_fill(self, addr, length, value=0):
        for off in range(0, length, 4):
            self.hostmem_wr32(addr + off, value)

    def hostmem_dump(self, addr, length):
        for off in range(0, length, 4):
            v = self.hostmem_rd32(addr + off)
            print(f"0x{off:04x}: 0x{v:08x}")

    def hostmem_read_dwords(self, addr, dword_count):
        return [self.hostmem_rd32(addr + 4*i) for i in range(dword_count)]

    def hostmem_wr_cmd(self, asq_base, slot, cmd_dwords):
        addr = asq_base + slot*64
        for i, w in enumerate(cmd_dwords):
            self.hostmem_wr32(addr + i*4, w)

    def hostmem_rd_cqe_slot(self, cq_base, slot):
        addr = cq_base + slot*16
        d0 = self.hostmem_rd32(addr + 0x0)
        d1 = self.hostmem_rd32(addr + 0x4)
        d2 = self.hostmem_rd32(addr + 0x8)
        d3 = self.hostmem_rd32(addr + 0xc)
        return (d0, d1, d2, d3)

    # MMIO access (RootPort -> NVMe BAR0) ---------------------------------------------------------

    def _mem_set_addr(self, addr):
        self.bus.regs.mmio_mem_adr_l.write(addr & 0xffffffff)
        self.bus.regs.mmio_mem_adr_h.write((addr >> 32) & 0xffffffff)

    def _mem_start(self, we, wsel=0xf, length=1):
        ctrl  = 0
        ctrl |= 1 << 0
        ctrl |= (1 if we else 0) << 1
        ctrl |= (wsel & 0xf) << 4
        ctrl |= (length & 0x3ff) << 8
        self.bus.regs.mmio_mem_ctrl.write(ctrl)
        self.bus.regs.mmio_mem_ctrl.write(0)

    def _mem_wait_done(self, timeout_ms=200):
        deadline = time.time() + (timeout_ms / 1000.0)
        while True:
            stat = self.bus.regs.mmio_mem_stat.read()
            done = (stat >> 0) & 1
            err  = (stat >> 1) & 1
            if done:
                return err
            if time.time() > deadline:
                raise TimeoutError("MEM transaction timeout (done=0).")

    def mmio_rd32(self, addr, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = self.timeout_ms
        self._mem_set_addr(addr)
        self._mem_start(we=0, wsel=0xf, length=1)
        err = self._mem_wait_done(timeout_ms=timeout_ms)
        val = self.bus.regs.mmio_mem_rdata.read()
        return val, err

    def mmio_wr32(self, addr, data, wsel=0xf, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = self.timeout_ms
        self._mem_set_addr(addr)
        self.bus.regs.mmio_mem_wdata.write(data & 0xffffffff)
        self._mem_start(we=1, wsel=wsel, length=1)
        err = self._mem_wait_done(timeout_ms=timeout_ms)
        return err

    def mmio_rd64(self, addr, timeout_ms=None):
        lo, err0 = self.mmio_rd32(addr + 0, timeout_ms=timeout_ms)
        hi, err1 = self.mmio_rd32(addr + 4, timeout_ms=timeout_ms)
        return ((hi << 32) | lo), (err0 | err1)

    def mmio_wr64(self, addr, data, timeout_ms=None):
        lo = data & 0xffffffff
        hi = (data >> 32) & 0xffffffff
        err0 = self.mmio_wr32(addr + 0, lo, timeout_ms=timeout_ms)
        err1 = self.mmio_wr32(addr + 4, hi, timeout_ms=timeout_ms)
        return err0 | err1

    def mmio_rd32_safe(self, addr, timeout_ms=None, retries=5):
        last = None
        for _ in range(retries):
            try:
                return self.mmio_rd32(addr, timeout_ms=timeout_ms)
            except TimeoutError as e:
                last = e
                time.sleep(0.01)
        raise last

    def mmio_wr32_safe(self, addr, data, wsel=0xf, timeout_ms=None, retries=5):
        last = None
        for _ in range(retries):
            try:
                return self.mmio_wr32(addr, data, wsel=wsel, timeout_ms=timeout_ms)
            except TimeoutError as e:
                last = e
                time.sleep(0.01)
        raise last

    # Core registers ------------------------------------------------------------------------------

    def read_core(self):
        cap,  e0 = self.mmio_rd64(self.bar0 + 0x0000)
        vs,   e1 = self.mmio_rd32(self.bar0 + 0x0008)
        intms,e2 = self.mmio_rd32(self.bar0 + 0x000C)
        intmc,e3 = self.mmio_rd32(self.bar0 + 0x0010)
        cc,   e4 = self.mmio_rd32(self.bar0 + 0x0014)
        csts, e5 = self.mmio_rd32(self.bar0 + 0x001C)
        aqa,  e6 = self.mmio_rd32(self.bar0 + 0x0024)
        asq,  e7 = self.mmio_rd64(self.bar0 + 0x0028)
        acq,  e8 = self.mmio_rd64(self.bar0 + 0x0030)

        errs = e0 | e1 | e2 | e3 | e4 | e5 | e6 | e7 | e8

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

    def nvme_wait_rdy(self, want_rdy, timeout_ms, poll_s=0.005):
        deadline = time.time() + (timeout_ms / 1000.0)
        while True:
            csts, err = self.mmio_rd32(self.bar0 + 0x001C, timeout_ms=timeout_ms)
            if err == 0 and csts_rdy(csts) == (1 if want_rdy else 0):
                return True
            if time.time() > deadline:
                return False
            time.sleep(poll_s)

    def nvme_disable(self, cap=None, timeout_ms=200):
        cc, err = self.mmio_rd32(self.bar0 + 0x0014, timeout_ms=timeout_ms)
        if err:
            print("WARN: read CC returned err=1.")
        if cc_en(cc) == 0:
            return True
        self.mmio_wr32(self.bar0 + 0x0014, cc & ~0x1, timeout_ms=timeout_ms)
        to_ms = 2000
        if cap is not None:
            to_ms = max(500, cap_to_ms(cap_decode(cap)["to"]))
        return self.nvme_wait_rdy(want_rdy=False, timeout_ms=to_ms, poll_s=0.001)

    def admin_init(self, asq_addr, acq_addr, q_entries=2, disable=True, timeout_ms=200):
        assert q_entries >= 2
        aqa = ((q_entries - 1) & 0xfff) | (((q_entries - 1) & 0xfff) << 16)

        cap, _ = self.mmio_rd64(self.bar0 + 0x0000, timeout_ms=timeout_ms)

        if disable:
            self.nvme_disable(cap=cap, timeout_ms=timeout_ms)

        e0 = self.mmio_wr32(self.bar0 + 0x0024, aqa, timeout_ms=timeout_ms)
        e1 = self.mmio_wr64(self.bar0 + 0x0028, asq_addr, timeout_ms=timeout_ms)
        e2 = self.mmio_wr64(self.bar0 + 0x0030, acq_addr, timeout_ms=timeout_ms)
        if e0 | e1 | e2:
            print("WARN: writing AQA/ASQ/ACQ had err=1 (posted/UR/CA).")

        cc = cc_make_en(iocqes=4, iosqes=6, mps=0, css=0, ams=0)
        e3 = self.mmio_wr32(self.bar0 + 0x0014, cc, timeout_ms=timeout_ms)
        if e3:
            print("WARN: writing CC had err=1.")

        to_ms = max(500, cap_to_ms(cap_decode(cap)["to"]))
        return self.nvme_wait_rdy(want_rdy=True, timeout_ms=to_ms, poll_s=0.001)

    # Admin queue helpers -------------------------------------------------------------------------

    def admin_setup(self, asq_addr, acq_addr, q_entries):
        self._asq_addr = asq_addr
        self._acq_addr = acq_addr
        self._q_entries = q_entries
        self._asq_tail = 0
        self._acq_head = 0
        self._acq_phase = 1

    def _poll_cq_phase(self, cq_base, head, phase, timeout_s=2.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            d0, d1, d2, d3 = self.hostmem_rd_cqe_slot(cq_base, head)
            cqe = decode_cqe(d0, d1, d2, d3)
            if cqe["p"] == phase:
                return cqe
            time.sleep(0.001)
        return None

    def admin_submit(self, cmd_dwords, timeout_s=2.0):
        if self._asq_addr is None:
            raise RuntimeError("admin_setup() must be called before admin_submit().")

        self.hostmem_fill(self._acq_addr + self._acq_head*16, 16, value=0)
        self.hostmem_wr_cmd(self._asq_addr, self._asq_tail, cmd_dwords)

        tail_next = (self._asq_tail + 1) % self._q_entries
        db = nvme_db_addr(self.bar0, self.mmio_rd64(self.bar0 + 0x0000)[0], qid=0, is_cq=False)
        err = self.mmio_wr32_safe(db, tail_next & 0xffff, timeout_ms=self.timeout_ms, retries=8)
        if err:
            print("WARN: SQ0 doorbell write err=1.")
        self._asq_tail = tail_next

        cqe = self._poll_cq_phase(self._acq_addr, self._acq_head, self._acq_phase, timeout_s=timeout_s)
        if cqe is None:
            return None

        self._acq_head = (self._acq_head + 1) % self._q_entries
        if self._acq_head == 0:
            self._acq_phase ^= 1
        db_cq = nvme_db_addr(self.bar0, self.mmio_rd64(self.bar0 + 0x0000)[0], qid=0, is_cq=True)
        self.mmio_wr32_safe(db_cq, self._acq_head & 0xffff, timeout_ms=self.timeout_ms, retries=8)
        return cqe

    # I/O queue helpers ---------------------------------------------------------------------------

    def io_setup(self, io_sq_addr, io_cq_addr, io_q_entries):
        self._io_sq_addr = io_sq_addr
        self._io_cq_addr = io_cq_addr
        self._io_q_entries = io_q_entries
        self._iosq_tail = 0
        self._iocq_head = 0
        self._iocq_phase = 1

    def io_submit(self, cmd_dwords, timeout_s=2.0):
        if self._io_sq_addr is None:
            raise RuntimeError("io_setup() must be called before io_submit().")

        self.hostmem_fill(self._io_cq_addr + self._iocq_head*16, 16, value=0)
        self.hostmem_wr_cmd(self._io_sq_addr, self._iosq_tail, cmd_dwords)

        tail_next = (self._iosq_tail + 1) % self._io_q_entries
        db = nvme_db_addr(self.bar0, self.mmio_rd64(self.bar0 + 0x0000)[0], qid=1, is_cq=False)
        self.mmio_wr32_safe(db, tail_next & 0xffff, timeout_ms=self.timeout_ms, retries=8)
        self._iosq_tail = tail_next

        cqe = self._poll_cq_phase(self._io_cq_addr, self._iocq_head, self._iocq_phase, timeout_s=timeout_s)
        if cqe is None:
            return None

        self._iocq_head = (self._iocq_head + 1) % self._io_q_entries
        if self._iocq_head == 0:
            self._iocq_phase ^= 1
        db_cq = nvme_db_addr(self.bar0, self.mmio_rd64(self.bar0 + 0x0000)[0], qid=1, is_cq=True)
        self.mmio_wr32_safe(db_cq, self._iocq_head & 0xffff, timeout_ms=self.timeout_ms, retries=8)
        return cqe
