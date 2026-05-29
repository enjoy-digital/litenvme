#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe hardware I/O command engine (RTL, single I/O queue, QD outstanding).

Status: NEW — authored from the validated firmware flow (bench/firmware/main.c
nvme_io_submit) and the MMIO accessor in litenvme/mem.py. Not yet simulated/HW-tested;
validate with test/test_io_engine.py (to be added) before wiring into the SoC.

Goal
----
Remove the CPU from the steady-state NVMe I/O path so that many commands can be kept in
flight (queue depth > 1) without per-command firmware overhead. The CPU/host still does
admin setup (controller enable, Create IO CQ/SQ); this engine owns the IO SQ/CQ ring at
runtime and drives:

  - SQE build into host memory (16 dwords / 64 B per command),
  - SQ doorbell ring (MMIO write via LiteNVMePCIeMmioAccessor),
  - CQE poll from host memory (phase-bit), completion emit,
  - CQ doorbell ring.

It mirrors exactly what the firmware does, but as a tight hardware loop, and lets up to
`qd` commands be outstanding (Little's law: IOPS ~= qd / latency).

Interfaces
----------
- `sink`   (stream): incoming request descriptors {op, nsid, lba, nlb, buf}.
- `source` (stream): outgoing completion descriptors {cid, status, sqhd}.
- `mmio`   : drives a LiteNVMePCIeMmioAccessor (doorbell writes). Connect in the SoC:
                self.comb += engine.connect_mmio(self.mmio)
- `mem`    : LiteNVMeMemPort, a simple 32-bit dword host-memory access port used to write
             SQEs and read CQEs. Connect to the hostmem responder backend (see note).

Host-memory access (`mem`) note
-------------------------------
The engine accesses host memory at *dword* granularity (matching the firmware's
hostmem_wr32/hostmem_rd32 used for SQE/CQE), through `LiteNVMeMemPort`:

    adr   : dword address (byte_addr >> 2) inside the host-memory window
    we    : 1=write, 0=read
    dat_w : write data (32-bit)
    stb   : strobe (asserted by engine to request a transfer; held until ack)
    ack   : acknowledge — MUST pertain to the address currently on `adr`. The engine
            holds `stb`/`adr`/`dat_w`/`we` steady and, on the cycle it samples `ack=1`,
            advances to the next address. For a single-cycle memory drive `ack` combin-
            ationally from `stb` (same cycle). For a multi-cycle/pipelined backend, keep
            `ack=0` until the access for the *presented* address completes, then pulse
            `ack=1` for exactly one cycle with `dat_r` valid that same cycle. Do NOT
            register `ack` off a held `stb` (that would ack a stale address and corrupt
            the SQE/CQE walk).
    dat_r : read data (32-bit), valid on the cycle `ack` is high.

This decouples the engine FSM from the host-memory backend. A thin adapter binds it to
the existing `hostmem` responder (e.g. reuse its CSR RMW datapath, or add a dedicated
native/AXI read-write port). Keeping it abstract here lets the engine be unit-tested with
a simple model memory.

NVMe SQE layout (read/write), matching bench/nvme_host.py / firmware:
    dw0  = opcode | (cid << 16)           # read=0x02, write=0x01
    dw1  = nsid
    dw6  = buf[31:0]                       # PRP1 (<= 4 KiB transfer => PRP1 only)
    dw7  = buf[63:32]
    dw10 = lba[31:0]
    dw11 = lba[63:32]
    dw12 = (nlb - 1) & 0xffff             # NLB is 0-based on the wire
    (all other dwords = 0)

CQE (16 B) dwords:
    dw2[15:0]  = SQ head (SQHD)
    dw3[15:0]  = CID
    dw3[16]    = Phase (P)
    dw3[17:24] = SC, dw3[25:27] = SCT     # status; success = SCT==0 & SC==0
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

# Constants ----------------------------------------------------------------------------------------

NVME_OP_WRITE = 0x01
NVME_OP_READ  = 0x02

SQE_DWORDS = 16   # 64 B submission queue entry.
CQE_DWORDS = 4    # 16 B completion queue entry.

# Descriptor layouts -------------------------------------------------------------------------------

def request_layout():
    return [
        ("op",   1),    # 0 = read, 1 = write.
        ("nsid", 32),
        ("lba",  64),
        ("nlb",  16),   # number of blocks (1-based; engine writes nlb-1 on the wire).
        ("buf",  64),   # PRP1 host-memory buffer address.
    ]

def completion_layout():
    return [
        ("cid",    16),
        ("status", 16),  # CQE status field (dw3[31:16]); success when SC/SCT == 0.
        ("sqhd",   16),
    ]

# Host-memory access port --------------------------------------------------------------------------

class LiteNVMeMemPort(Record):
    """Simple 32-bit dword host-memory access port (see module docstring)."""
    def __init__(self, adr_width=32):
        Record.__init__(self, [
            ("adr",   adr_width),  # dword address.
            ("we",    1),
            ("dat_w", 32),
            ("stb",   1),
            ("ack",   1),
            ("dat_r", 32),
        ])

# LiteNVMe I/O Engine ------------------------------------------------------------------------------

class LiteNVMeIOEngine(LiteXModule):
    """Hardware NVMe I/O command engine (single IO queue, up to `qd` outstanding).

    Parameters
    ----------
    qid          : I/O queue id (must match the IO SQ/CQ created by admin setup).
    qsize        : number of entries in the IO SQ/CQ ring (== created queue size).
    qd           : max commands outstanding (<= qsize - 1).
    mem_adr_width: width of the host-memory dword address port.

    The base addresses (SQ/CQ in host memory, doorbells in BAR0) are runtime config
    (CSR or constructor wiring) so the same gateware works regardless of the layout the
    admin/setup code chose.
    """
    def __init__(self, qid=1, qsize=64, qd=None, mem_adr_width=32, with_csr=True):
        if qd is None:
            qd = qsize - 1
        assert qd <= qsize - 1, "qd must be <= qsize-1 (one slot reserved to disambiguate full/empty)"
        self.qid   = qid
        self.qsize = qsize
        self.qd    = qd

        # Streams.
        self.sink   = stream.Endpoint(request_layout())
        self.source = stream.Endpoint(completion_layout())

        # Host-memory port (SQE write / CQE read).
        self.mem = LiteNVMeMemPort(mem_adr_width)

        # MMIO doorbell driver (connect to a LiteNVMePCIeMmioAccessor via connect_mmio()).
        self.mmio_start = Signal()
        self.mmio_we    = Signal()
        self.mmio_adr   = Signal(64)
        self.mmio_wdata = Signal(32)
        self.mmio_done  = Signal()
        self.mmio_err   = Signal()

        # Runtime config (set by CSR or directly).
        self.enable      = Signal()
        self.sq_base     = Signal(64)   # IO SQ base (host-memory byte address).
        self.cq_base     = Signal(64)   # IO CQ base (host-memory byte address).
        self.sq_db_adr   = Signal(64)   # SQ doorbell (BAR0 byte address).
        self.cq_db_adr   = Signal(64)   # CQ doorbell (BAR0 byte address).

        # Status.
        self.inflight    = Signal(max=qsize + 1)
        self.submitted   = Signal(32)
        self.completed   = Signal(32)
        self.busy        = Signal()

        # # #

        # Ring state.
        sq_tail  = Signal(max=qsize)
        cq_head  = Signal(max=qsize)
        cq_phase = Signal(reset=1)

        # Latched request being submitted.
        req_op   = Signal()
        req_nsid = Signal(32)
        req_lba  = Signal(64)
        req_nlb  = Signal(16)
        req_buf  = Signal(64)
        req_cid  = Signal(16)

        # SQE dword index + value (combinational build from latched request).
        dw_idx = Signal(max=SQE_DWORDS)
        sqe_dw = Signal(32)
        opcode = Signal(8)
        self.comb += opcode.eq(Mux(req_op, NVME_OP_WRITE, NVME_OP_READ))
        self.comb += Case(dw_idx, {
            0:  sqe_dw.eq(opcode | (req_cid << 16)),
            1:  sqe_dw.eq(req_nsid),
            6:  sqe_dw.eq(req_buf[0:32]),
            7:  sqe_dw.eq(req_buf[32:64]),
            10: sqe_dw.eq(req_lba[0:32]),
            11: sqe_dw.eq(req_lba[32:64]),
            12: sqe_dw.eq((req_nlb - 1)[0:16]),
            "default": sqe_dw.eq(0),
        })

        # CQE capture (named dwords; dynamic Array index as a NextValue target is avoided).
        cqe0 = Signal(32)
        cqe1 = Signal(32)
        cqe2 = Signal(32)
        cqe3 = Signal(32)
        cqe_idx   = Signal(max=CQE_DWORDS)
        cqe_phase = Signal()
        cqe_cid   = Signal(16)
        cqe_sts   = Signal(16)
        cqe_sqhd  = Signal(16)
        self.comb += [
            cqe_phase.eq(cqe3[16]),
            cqe_cid.eq(cqe3[0:16]),
            cqe_sts.eq(cqe3[16:32]),
            cqe_sqhd.eq(cqe2[0:16]),
        ]

        # Helpers for ring increments (wrap at qsize).
        def ring_inc(sig):
            return If(sig == (qsize - 1), NextValue(sig, 0)).Else(NextValue(sig, sig + 1))

        # Can we submit a new command? Room in the in-flight window and a request waiting.
        can_submit = Signal()
        self.comb += can_submit.eq(self.enable & self.sink.valid & (self.inflight < qd))

        # Should we try to reap? Something outstanding.
        can_reap = Signal()
        self.comb += can_reap.eq(self.enable & (self.inflight != 0))

        # Byte-address helpers.
        sq_slot_adr = Signal(64)  # byte address of current SQ slot.
        cq_slot_adr = Signal(64)  # byte address of current CQ slot.
        self.comb += [
            sq_slot_adr.eq(self.sq_base + sq_tail * (SQE_DWORDS * 4)),
            cq_slot_adr.eq(self.cq_base + cq_head * (CQE_DWORDS * 4)),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

        fsm.act("IDLE",
            If(can_submit,
                NextState("LATCH-REQ"),
            ).Elif(can_reap,
                NextState("REAP-READ"),
            )
        )

        # Latch the request and allocate a CID (== SQ slot index).
        fsm.act("LATCH-REQ",
            NextValue(req_op,   self.sink.op),
            NextValue(req_nsid, self.sink.nsid),
            NextValue(req_lba,  self.sink.lba),
            NextValue(req_nlb,  self.sink.nlb),
            NextValue(req_buf,  self.sink.buf),
            NextValue(req_cid,  sq_tail),
            NextValue(dw_idx,   0),
            NextState("SUBMIT-WRITE"),
        )

        # Write the 16 SQE dwords into host memory.
        fsm.act("SUBMIT-WRITE",
            self.mem.stb.eq(1),
            self.mem.we.eq(1),
            self.mem.adr.eq((sq_slot_adr >> 2) + dw_idx),
            self.mem.dat_w.eq(sqe_dw),
            If(self.mem.ack,
                If(dw_idx == (SQE_DWORDS - 1),
                    NextState("SUBMIT-ADVANCE"),
                ).Else(
                    NextValue(dw_idx, dw_idx + 1),
                )
            )
        )

        # Advance SQ tail, consume the request, then ring the SQ doorbell.
        fsm.act("SUBMIT-ADVANCE",
            self.sink.ready.eq(1),  # request consumed (latched).
            ring_inc(sq_tail),
            NextValue(self.inflight, self.inflight + 1),
            NextValue(self.submitted, self.submitted + 1),
            NextState("SUBMIT-DOORBELL"),
        )

        fsm.act("SUBMIT-DOORBELL",
            self.mmio_start.eq(1),
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.sq_db_adr),
            # sq_tail already advanced; ring with the new tail value.
            self.mmio_wdata.eq(sq_tail),
            NextState("SUBMIT-DOORBELL-WAIT"),
        )
        fsm.act("SUBMIT-DOORBELL-WAIT",
            If(self.mmio_done,
                NextState("IDLE"),
            )
        )

        # Reap: read the 4 CQE dwords from host memory.
        fsm.act("REAP-READ",
            NextValue(cqe_idx, 0),
            NextState("REAP-READ-LOOP"),
        )
        fsm.act("REAP-READ-LOOP",
            self.mem.stb.eq(1),
            self.mem.we.eq(0),
            self.mem.adr.eq((cq_slot_adr >> 2) + cqe_idx),
            If(self.mem.ack,
                Case(cqe_idx, {
                    0: NextValue(cqe0, self.mem.dat_r),
                    1: NextValue(cqe1, self.mem.dat_r),
                    2: NextValue(cqe2, self.mem.dat_r),
                    3: NextValue(cqe3, self.mem.dat_r),
                }),
                If(cqe_idx == (CQE_DWORDS - 1),
                    NextState("REAP-CHECK"),
                ).Else(
                    NextValue(cqe_idx, cqe_idx + 1),
                )
            )
        )

        # Check the phase bit. If matched -> a completion is available.
        fsm.act("REAP-CHECK",
            If(cqe_phase == cq_phase,
                NextState("REAP-EMIT"),
            ).Else(
                # Not ready yet: go back to arbitration (try submitting more meanwhile).
                NextState("IDLE"),
            )
        )

        # Emit the completion descriptor.
        fsm.act("REAP-EMIT",
            self.source.valid.eq(1),
            self.source.cid.eq(cqe_cid),
            self.source.status.eq(cqe_sts),
            self.source.sqhd.eq(cqe_sqhd),
            If(self.source.ready,
                NextValue(self.inflight, self.inflight - 1),
                NextValue(self.completed, self.completed + 1),
                # Advance CQ head; toggle phase on wrap.
                If(cq_head == (qsize - 1),
                    NextValue(cq_head, 0),
                    NextValue(cq_phase, ~cq_phase),
                ).Else(
                    NextValue(cq_head, cq_head + 1),
                ),
                NextState("REAP-DOORBELL"),
            )
        )

        fsm.act("REAP-DOORBELL",
            self.mmio_start.eq(1),
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.cq_db_adr),
            self.mmio_wdata.eq(cq_head),  # cq_head already advanced above.
            NextState("REAP-DOORBELL-WAIT"),
        )
        fsm.act("REAP-DOORBELL-WAIT",
            If(self.mmio_done,
                NextState("IDLE"),
            )
        )

        if with_csr:
            self.add_csr()

    # MMIO wiring helper -------------------------------------------------------------------------
    def connect_mmio(self, mmio):
        """Return comb statements binding this engine to a LiteNVMePCIeMmioAccessor.

        NOTE: the accessor is single-outstanding and shared; if firmware also drives it,
        add arbitration. For the steady-state engine path it should own the accessor.
        """
        return [
            mmio.start.eq(self.mmio_start),
            mmio.we.eq(self.mmio_we),
            mmio.adr.eq(self.mmio_adr),
            mmio.wdata.eq(self.mmio_wdata),
            mmio.wsel.eq(0xf),
            mmio.len.eq(1),
            self.mmio_done.eq(mmio.done),
            self.mmio_err.eq(mmio.err),
        ]

    # CSRs ---------------------------------------------------------------------------------------
    def add_csr(self):
        self._enable = CSRStorage(description="Enable the hardware I/O engine.")
        self._sq_base_lo = CSRStorage(32, description="IO SQ base (host byte addr) low.")
        self._sq_base_hi = CSRStorage(32, description="IO SQ base (host byte addr) high.")
        self._cq_base_lo = CSRStorage(32, description="IO CQ base (host byte addr) low.")
        self._cq_base_hi = CSRStorage(32, description="IO CQ base (host byte addr) high.")
        self._sq_db_lo   = CSRStorage(32, description="SQ doorbell (BAR0 byte addr) low.")
        self._sq_db_hi   = CSRStorage(32, description="SQ doorbell (BAR0 byte addr) high.")
        self._cq_db_lo   = CSRStorage(32, description="CQ doorbell (BAR0 byte addr) low.")
        self._cq_db_hi   = CSRStorage(32, description="CQ doorbell (BAR0 byte addr) high.")
        self._status = CSRStatus(fields=[
            CSRField("busy",     size=1,  offset=0),
            CSRField("inflight", size=8,  offset=8),
        ])
        self._submitted = CSRStatus(32, description="Total commands submitted.")
        self._completed = CSRStatus(32, description="Total completions reaped.")

        self.comb += [
            self.enable.eq(self._enable.storage),
            self.sq_base.eq(Cat(self._sq_base_lo.storage, self._sq_base_hi.storage)),
            self.cq_base.eq(Cat(self._cq_base_lo.storage, self._cq_base_hi.storage)),
            self.sq_db_adr.eq(Cat(self._sq_db_lo.storage, self._sq_db_hi.storage)),
            self.cq_db_adr.eq(Cat(self._cq_db_lo.storage, self._cq_db_hi.storage)),
            self._status.fields.busy.eq(self.busy),
            self._status.fields.inflight.eq(self.inflight),
            self._submitted.status.eq(self.submitted),
            self._completed.status.eq(self.completed),
        ]
