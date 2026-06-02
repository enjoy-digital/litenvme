#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe RTL init sequencer (EXPLORATION) — NVMe bring-up entirely in hardware, no CPU.

This is a pure-RTL replacement for the firmware bring-up (`nvme_setup`/`nvme_autoinit` in
bench/firmware/main.c). It replays the same sequence as a state machine so the standalone core
can be fully CPU-less (matching commercial CPU-less host IP):

  1. Config space (cfg accessor): assign BAR0, enable Command MEM+BME.
  2. Root cfg-mgmt: open the root-port memory window so MMIO is forwarded to BAR0.
  3. Controller (mmio accessor): read CAP, write AQA/ASQ/ACQ, write CC.EN, poll CSTS.RDY.
  4. Admin commands (host-memory + admin doorbell + CQE poll): Set Features (Number of Queues),
     Create IO CQ, Create IO SQ.
  5. Program the I/O engine (queue bases, doorbells, PRP-list) and enable it; raise `init_done`.

It drives abstract interfaces (a CFG accessor, an MMIO accessor, a host-memory dword port, and a
root cfg-mgmt port) so it can be unit-tested against simple models; the SoC wires those to real
`LiteNVMePCIeCfgAccessor` / `LiteNVMePCIeMmioAccessor` (with_csr=False) / hostmem / RootCfgMgmt.

Assumptions (match the bench setup): SSD at BDF 0:1:0, 64-bit BAR0, doorbell stride DSTRD=0.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from litenvme.io_engine import LiteNVMeMemPort

# Sequencer ----------------------------------------------------------------------------------------

class LiteNVMeInitSequencer(LiteXModule):
    def __init__(self, bar0_base=0xe000_0000, hostmem_base=0x1000_0000,
                 admin_q_entries=2, io_q_entries=64, io_qid=1,
                 mem_window_dw=0x20, mem_window_val=0xe000_e000, mem_adr_width=32,
                 poll_timeout=2**22, with_csr=False):
        # Host-memory layout (must match the engine + the hostmem responder window).
        ASQ   = hostmem_base + 0x0000
        ACQ   = hostmem_base + 0x1000
        IO_CQ = hostmem_base + 0x3000
        IO_SQ = hostmem_base + 0x4000
        PRP   = hostmem_base + 0x10000 + io_q_entries * 0x1000

        # BAR0 register byte offsets + doorbells (DSTRD=0 -> 4-byte stride).
        CAP, CC, CSTS, AQA, ASQ_R, ACQ_R = 0x00, 0x14, 0x1c, 0x24, 0x28, 0x30
        adm_sq_db = bar0_base + 0x1000 + (0*2 + 0)*4
        adm_cq_db = bar0_base + 0x1000 + (0*2 + 1)*4
        io_sq_db  = bar0_base + 0x1000 + (io_qid*2 + 0)*4
        io_cq_db  = bar0_base + 0x1000 + (io_qid*2 + 1)*4

        # Control / status.
        self.start      = Signal()
        self.init_done  = Signal()
        self.init_error = Signal()
        self.busy       = Signal()

        # CFG accessor drive (BDF fixed to 0:1:0).
        self.cfg_start = Signal()
        self.cfg_we    = Signal()
        self.cfg_reg   = Signal(6)
        self.cfg_wdata = Signal(32)
        self.cfg_done  = Signal()
        self.cfg_err   = Signal()
        self.cfg_rdata = Signal(32)
        self.cfg_bus   = Signal(8)
        self.cfg_dev   = Signal(5)
        self.cfg_func  = Signal(3)
        self.comb += [self.cfg_bus.eq(0), self.cfg_dev.eq(1), self.cfg_func.eq(0)]

        # MMIO accessor drive (single DWORD; wsel=0xf, len=1 held).
        self.mmio_start = Signal()
        self.mmio_we    = Signal()
        self.mmio_adr   = Signal(64)
        self.mmio_wdata = Signal(32)
        self.mmio_wsel  = Signal(4)
        self.mmio_len   = Signal(10)
        self.mmio_done  = Signal()
        self.mmio_err   = Signal()
        self.mmio_rdata = Signal(32)
        self.comb += [self.mmio_wsel.eq(0xf), self.mmio_len.eq(1)]

        # Root cfg-mgmt drive (memory window).
        self.root_start = Signal()
        self.root_write = Signal()
        self.root_addr  = Signal(10)
        self.root_wdata = Signal(32)
        self.root_done  = Signal()

        # Host-memory dword port (SQE write / CQE read).
        self.mem = LiteNVMeMemPort(mem_adr_width)

        # I/O engine config outputs (held after bring-up).
        self.eng_enable   = Signal()
        self.eng_sq_base  = Signal(64)
        self.eng_cq_base  = Signal(64)
        self.eng_sq_db    = Signal(64)
        self.eng_cq_db    = Signal(64)
        self.eng_prp_list = Signal(64)

        # # #

        # Start pulse.
        start_d, start_pulse = Signal(), Signal()
        self.sync += start_d.eq(self.start)
        self.comb += start_pulse.eq(self.start & ~start_d)

        # ---- Step tables (constants) ----
        cfg_tab = [
            (4, (bar0_base & 0xffffffff) | 0x4),  # BAR0 lo (64-bit memory BAR).
            (5, (bar0_base >> 32) & 0xffffffff),  # BAR1 hi.
            (1, 0x0000_0006),                     # Command: MEM (bit1) + BME (bit2).
        ]
        cc_en = (1 << 0) | (6 << 16) | (4 << 20)  # cc_make_en(iocqes=4, iosqes=6, mps=0).
        aqa   = ((admin_q_entries - 1) & 0xfff) | (((admin_q_entries - 1) & 0xfff) << 16)
        mmio_tab = [
            (AQA,        aqa),
            (ASQ_R,      ASQ & 0xffffffff), (ASQ_R + 4, (ASQ >> 32) & 0xffffffff),
            (ACQ_R,      ACQ & 0xffffffff), (ACQ_R + 4, (ACQ >> 32) & 0xffffffff),
            (CC,         cc_en),
        ]
        # Admin command SQEs (16 dwords; only non-zero dwords listed).
        admin_cmds = [
            {0: 0x09 | (0x20 << 16),            10: 0x07},                                      # Set Features: Num Queues (nsqr=ncqr=1).
            {0: 0x05 | (0x21 << 16), 6: IO_CQ & 0xffffffff, 7: (IO_CQ >> 32) & 0xffffffff,
             10: (io_qid & 0xffff) | ((io_q_entries - 1) << 16), 11: 0x1},                      # Create IO CQ (pc=1).
            {0: 0x01 | (0x22 << 16), 6: IO_SQ & 0xffffffff, 7: (IO_SQ >> 32) & 0xffffffff,
             10: (io_qid & 0xffff) | ((io_q_entries - 1) << 16), 11: (io_qid << 16) | 0x1},     # Create IO SQ (cqid=io_qid, pc=1).
        ]

        # Combinational selectors.
        ci      = Signal(max=len(cfg_tab))
        mi      = Signal(max=len(mmio_tab))
        cmd_idx = Signal(2)
        dw_idx  = Signal(5)   # 0..15.
        cqw_idx = Signal(2)   # 0..3.
        cqe3    = Signal(32)
        sq_tail = Signal(max=admin_q_entries)
        cq_head = Signal(max=admin_q_entries)
        cq_phase = Signal(reset=1)
        poll    = Signal(max=poll_timeout)

        cfg_reg_v, cfg_dat_v = Signal(6), Signal(32)
        self.comb += Case(ci, {i: [cfg_reg_v.eq(r), cfg_dat_v.eq(d)] for i, (r, d) in enumerate(cfg_tab)})
        mmio_off_v, mmio_dat_v = Signal(32), Signal(32)
        self.comb += Case(mi, {i: [mmio_off_v.eq(o), mmio_dat_v.eq(d)] for i, (o, d) in enumerate(mmio_tab)})

        sqe_dw = Signal(32)
        sqe_cases = {}
        for c in range(3):
            for d in range(16):
                v = admin_cmds[c].get(d, 0)
                if v:
                    sqe_cases[(c << 4) | d] = sqe_dw.eq(v)
        self.comb += Case(Cat(dw_idx[:4], cmd_idx), dict(sqe_cases, default=sqe_dw.eq(0)))

        # ACQ-clear dword: dw3 carries the inverted phase so the poll can't match a stale entry.
        cq_clr = Signal(32)
        self.comb += If(cqw_idx == 3, cq_clr.eq((cq_phase ^ 1) << 16)).Else(cq_clr.eq(0))

        # Byte addresses of the current admin SQ slot / CQ slot.
        sq_slot = Signal(32)
        cq_slot = Signal(32)
        self.comb += [
            sq_slot.eq(ASQ + sq_tail * 64),
            cq_slot.eq(ACQ + cq_head * 16),
        ]
        sq_tail_next = Signal(max=admin_q_entries)
        cq_head_next = Signal(max=admin_q_entries)
        self.comb += [
            sq_tail_next.eq(Mux(sq_tail == (admin_q_entries - 1), 0, sq_tail + 1)),
            cq_head_next.eq(Mux(cq_head == (admin_q_entries - 1), 0, cq_head + 1)),
        ]

        # CQE status (success when SC and SCT are 0).
        cqe_phase = Signal()
        cqe_sc    = Signal(8)
        cqe_sct   = Signal(3)
        self.comb += [cqe_phase.eq(cqe3[16]), cqe_sc.eq(cqe3[17:25]), cqe_sct.eq(cqe3[25:28])]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~(fsm.ongoing("IDLE") | fsm.ongoing("READY") | fsm.ongoing("FAILED")))

        fsm.act("IDLE",
            If(start_pulse, NextValue(ci, 0), NextState("CFG-ISSUE")),
        )

        # ---- 1. Config space: BAR0 assign + Command MEM/BME. ----
        fsm.act("CFG-ISSUE",
            self.cfg_start.eq(1), self.cfg_we.eq(1),
            self.cfg_reg.eq(cfg_reg_v), self.cfg_wdata.eq(cfg_dat_v),
            NextState("CFG-WAIT"),
        )
        fsm.act("CFG-WAIT",
            self.cfg_reg.eq(cfg_reg_v), self.cfg_wdata.eq(cfg_dat_v), self.cfg_we.eq(1),
            If(self.cfg_done,
                If(self.cfg_err, NextState("FAILED")
                ).Elif(ci == (len(cfg_tab) - 1), NextState("ROOT-ISSUE")
                ).Else(NextValue(ci, ci + 1), NextState("CFG-ISSUE")),
            ),
        )

        # ---- 2. Root memory window (so MMIO is forwarded downstream). ----
        fsm.act("ROOT-ISSUE",
            self.root_start.eq(1), self.root_write.eq(1),
            self.root_addr.eq(mem_window_dw), self.root_wdata.eq(mem_window_val),
            NextState("ROOT-WAIT"),
        )
        fsm.act("ROOT-WAIT",
            self.root_addr.eq(mem_window_dw), self.root_wdata.eq(mem_window_val), self.root_write.eq(1),
            If(self.root_done, NextValue(poll, 0), NextState("CAP-ISSUE")),
        )

        # ---- 3. Controller: CAP read (confirm BAR0 alive), AQA/ASQ/ACQ, CC.EN, poll RDY. ----
        fsm.act("CAP-ISSUE",
            self.mmio_start.eq(1), self.mmio_we.eq(0), self.mmio_adr.eq(bar0_base + CAP),
            NextState("CAP-WAIT"),
        )
        fsm.act("CAP-WAIT",
            self.mmio_adr.eq(bar0_base + CAP),
            If(self.mmio_done,
                If(self.mmio_rdata != 0, NextValue(mi, 0), NextState("MMIO-ISSUE")
                ).Elif(poll == (poll_timeout - 1), NextState("FAILED")
                ).Else(NextValue(poll, poll + 1), NextState("CAP-ISSUE")),
            ),
        )
        fsm.act("MMIO-ISSUE",
            self.mmio_start.eq(1), self.mmio_we.eq(1),
            self.mmio_adr.eq(bar0_base + mmio_off_v), self.mmio_wdata.eq(mmio_dat_v),
            NextState("MMIO-WAIT"),
        )
        fsm.act("MMIO-WAIT",
            self.mmio_adr.eq(bar0_base + mmio_off_v), self.mmio_wdata.eq(mmio_dat_v), self.mmio_we.eq(1),
            If(self.mmio_done,
                If(mi == (len(mmio_tab) - 1), NextValue(poll, 0), NextState("RDY-ISSUE")
                ).Else(NextValue(mi, mi + 1), NextState("MMIO-ISSUE")),
            ),
        )
        fsm.act("RDY-ISSUE",
            self.mmio_start.eq(1), self.mmio_we.eq(0), self.mmio_adr.eq(bar0_base + CSTS),
            NextState("RDY-WAIT"),
        )
        fsm.act("RDY-WAIT",
            self.mmio_adr.eq(bar0_base + CSTS),
            If(self.mmio_done,
                If(self.mmio_rdata[0], NextValue(cmd_idx, 0), NextState("ADMIN-START")
                ).Elif(poll == (poll_timeout - 1), NextState("FAILED")
                ).Else(NextValue(poll, poll + 1), NextState("RDY-ISSUE")),
            ),
        )

        # ---- 4. Admin commands (Set Features, Create IO CQ, Create IO SQ). ----
        fsm.act("ADMIN-START",
            NextValue(dw_idx, 0), NextState("SQE-WR"),
        )
        fsm.act("SQE-WR",
            self.mem.stb.eq(1), self.mem.we.eq(1),
            self.mem.adr.eq((sq_slot >> 2) + dw_idx), self.mem.dat_w.eq(sqe_dw),
            If(self.mem.ack,
                If(dw_idx == 15, NextValue(cqw_idx, 0), NextState("CQ-CLR")
                ).Else(NextValue(dw_idx, dw_idx + 1)),
            ),
        )
        fsm.act("CQ-CLR",
            self.mem.stb.eq(1), self.mem.we.eq(1),
            self.mem.adr.eq((cq_slot >> 2) + cqw_idx), self.mem.dat_w.eq(cq_clr),
            If(self.mem.ack,
                If(cqw_idx == 3, NextState("ADMIN-RING")
                ).Else(NextValue(cqw_idx, cqw_idx + 1)),
            ),
        )
        fsm.act("ADMIN-RING",
            NextValue(sq_tail, sq_tail_next),
            self.mmio_start.eq(1), self.mmio_we.eq(1),
            self.mmio_adr.eq(adm_sq_db), self.mmio_wdata.eq(sq_tail_next),
            NextValue(poll, 0), NextState("ADMIN-RING-WAIT"),
        )
        fsm.act("ADMIN-RING-WAIT",
            self.mmio_adr.eq(adm_sq_db), self.mmio_wdata.eq(sq_tail), self.mmio_we.eq(1),
            If(self.mmio_done, NextState("CQE-RD")),
        )
        fsm.act("CQE-RD",
            self.mem.stb.eq(1), self.mem.we.eq(0),
            self.mem.adr.eq((cq_slot >> 2) + 3),
            If(self.mem.ack, NextValue(cqe3, self.mem.dat_r), NextState("CQE-CHK")),
        )
        fsm.act("CQE-CHK",
            If(cqe_phase == cq_phase, NextState("CQE-STS")
            ).Elif(poll == (poll_timeout - 1), NextState("FAILED")
            ).Else(NextValue(poll, poll + 1), NextState("CQE-RD")),
        )
        fsm.act("CQE-STS",
            If((cqe_sc != 0) | (cqe_sct != 0), NextState("FAILED")
            ).Else(
                If(cq_head == (admin_q_entries - 1), NextValue(cq_phase, ~cq_phase)),
                NextValue(cq_head, cq_head_next),
                NextState("CQDB"),
            ),
        )
        fsm.act("CQDB",
            self.mmio_start.eq(1), self.mmio_we.eq(1),
            self.mmio_adr.eq(adm_cq_db), self.mmio_wdata.eq(cq_head),
            NextState("CQDB-WAIT"),
        )
        fsm.act("CQDB-WAIT",
            self.mmio_adr.eq(adm_cq_db), self.mmio_wdata.eq(cq_head), self.mmio_we.eq(1),
            If(self.mmio_done,
                If(cmd_idx == 2, NextState("ENGINE")
                ).Else(NextValue(cmd_idx, cmd_idx + 1), NextState("ADMIN-START")),
            ),
        )

        # ---- 5. Program + enable the I/O engine; raise init_done. ----
        fsm.act("ENGINE",
            NextValue(self.eng_sq_base,  IO_SQ),
            NextValue(self.eng_cq_base,  IO_CQ),
            NextValue(self.eng_sq_db,    io_sq_db),
            NextValue(self.eng_cq_db,    io_cq_db),
            NextValue(self.eng_prp_list, PRP),
            NextValue(self.eng_enable,   1),
            NextValue(self.init_done,    1),
            NextState("READY"),
        )
        fsm.act("READY")                            # eng_*/init_done held by their registers.
        fsm.act("FAILED", NextValue(self.init_error, 1), NextState("FAILED-HOLD"))
        fsm.act("FAILED-HOLD")

        if with_csr:
            self.add_csr()

    # CSRs (for host-triggered bring-up / status over Etherbone) ------------------------------------
    def add_csr(self):
        self._ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start NVMe RTL bring-up."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("done",  size=1, offset=0, description="Bring-up complete."),
            CSRField("error", size=1, offset=1, description="Bring-up failed."),
            CSRField("busy",  size=1, offset=2, description="Bring-up in progress."),
        ])
        self.comb += [
            self.start.eq(self._ctrl.fields.start),
            self._status.fields.done.eq(self.init_done),
            self._status.fields.error.eq(self.init_error),
            self._status.fields.busy.eq(self.busy),
        ]
