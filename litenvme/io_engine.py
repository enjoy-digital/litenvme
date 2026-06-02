#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe hardware I/O command engine (RTL, single I/O queue, QD outstanding).

Authored from the validated firmware flow in bench/firmware/main.c (nvme_io_submit) and the
MMIO accessor in litenvme/mem.py.

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
from litex.soc.interconnect import axi
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
    def __init__(self, qid=1, qsize=64, qd=None, mem_adr_width=32, with_csr=True,
                 lba_shift=9, page_shift=12):
        if qd is None:
            qd = qsize - 1
        assert qd <= qsize - 1, "qd must be <= qsize-1 (one slot reserved to disambiguate full/empty)"
        self.qid        = qid
        self.qsize      = qsize
        self.qd         = qd
        self.lba_shift  = lba_shift    # log2(LBA size in bytes), default 512 B.
        self.page_shift = page_shift   # log2(MPS, NVMe memory page size), default 4 KiB.

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
        # PRP-list region (host byte address): one page-aligned list page per SQ slot,
        # used when a transfer spans >2 pages. Each list page holds up to (page/8)
        # 64-bit page-address entries. Only consulted for such transfers.
        self.prp_list_base = Signal(64)

        # Status.
        self.inflight    = Signal(max=qsize + 1)
        self.submitted   = Signal(32)
        self.completed   = Signal(32)
        self.busy        = Signal()

        # # #

        # Ring state.
        self.sq_tail = sq_tail = Signal(max=qsize)
        self.cq_head = cq_head = Signal(max=qsize)
        cq_phase = Signal(reset=1)

        # Doorbell coalescing: the last SQ-tail / CQ-head values actually rung to the device.
        # Instead of one doorbell MMIO round-trip per command (the previous behaviour, which
        # serialised the FSM at QD~=1 on reads), the engine submits a burst of SQEs advancing
        # sq_tail, then rings the SQ doorbell ONCE (when it can't submit more), and reaps a
        # batch of CQEs advancing cq_head, then rings the CQ doorbell ONCE. A doorbell is
        # "pending" whenever the ring pointer has moved past the last value rung. The qd <=
        # qsize-1 window guarantees sq_tail never laps sq_db_rung, so (sq_tail != sq_db_rung)
        # is an unambiguous "submits pending" test across the ring wrap.
        sq_db_rung = Signal(max=qsize)
        cq_db_rung = Signal(max=qsize)

        # Latched request being submitted.
        req_op   = Signal()
        req_nsid = Signal(32)
        req_lba  = Signal(64)
        req_nlb  = Signal(16)
        req_buf  = Signal(64)
        req_cid  = Signal(16)

        # PRP fields (computed at LATCH-REQ, then latched). Assumes the data buffer is
        # physically contiguous and page-aligned starting at `buf`:
        #   nbytes  = nlb << lba_shift                  (nlb is a 1-based block count)
        #   npages  = ceil(nbytes / page)               (page-aligned => exact)
        #   PRP1    = buf                               (always)
        #   PRP2    = 0                 if npages == 1  (single page)
        #             buf + page        if npages == 2  (two pages)
        #             prp_list_slot     if npages >= 3  (page list)
        # For the list case, list[i] = buf + (i+1)*page for i in 0..npages-2, written as
        # 64-bit entries into the slot's list page before the SQE.
        req_prp2      = Signal(64)
        req_need_list = Signal()
        req_nlist     = Signal(16)   # number of 64-bit list entries (= npages - 1).
        req_list_slot = Signal(64)   # byte address of this command's PRP list page.

        page_bytes = 1 << page_shift

        # Combinational PRP computation from the incoming sink fields + current sq_tail.
        s_nbytes  = Signal(32)
        s_npages  = Signal(32)
        s_list_slot = Signal(64)
        self.comb += [
            s_nbytes.eq(self.sink.nlb << lba_shift),
            s_npages.eq((s_nbytes + (page_bytes - 1)) >> page_shift),
            s_list_slot.eq(self.prp_list_base + (sq_tail << page_shift)),
        ]
        s_prp2      = Signal(64)
        s_need_list = Signal()
        self.comb += [
            s_need_list.eq(s_npages >= 3),
            If(s_npages <= 1,
                s_prp2.eq(0),
            ).Elif(s_npages == 2,
                s_prp2.eq(self.sink.buf + page_bytes),
            ).Else(
                s_prp2.eq(s_list_slot),
            ),
        ]

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
            8:  sqe_dw.eq(req_prp2[0:32]),
            9:  sqe_dw.eq(req_prp2[32:64]),
            10: sqe_dw.eq(req_lba[0:32]),
            11: sqe_dw.eq(req_lba[32:64]),
            12: sqe_dw.eq((req_nlb - 1)[0:16]),
            "default": sqe_dw.eq(0),
        })

        # PRP-list entry build: dword `list_dw` covers entry (list_dw>>1), low/high half.
        list_dw   = Signal(16)
        list_entry = Signal(16)
        list_half  = Signal()
        ent_addr   = Signal(64)
        list_dw_val = Signal(32)
        self.comb += [
            list_entry.eq(list_dw >> 1),
            list_half.eq(list_dw[0]),
            ent_addr.eq(req_buf + ((list_entry + 1) << page_shift)),
            list_dw_val.eq(Mux(list_half, ent_addr[32:64], ent_addr[0:32])),
        ]

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

        # Per-slot (== per-CID) busy bits. The aggregate `inflight < qd` window does NOT
        # guarantee that the SPECIFIC CID about to be reused (CID = sq_tail, which wraps at
        # qsize) has actually been retired by the device -- completions can lag/reorder. If
        # the engine re-submits CID N while the device still holds CID N outstanding, the
        # device rejects it with SC=0x03 "Command ID Conflict" (observed on HW as ~1
        # error/1000 around the ring wrap). slot_busy[cid] is set when a CID is submitted and
        # cleared when its CQE is reaped; submit is gated on the target CID being free.
        slot_busy = Array(Signal() for _ in range(qsize))

        # Can we submit a new command? Room in the in-flight window, a request waiting, AND
        # the CID we're about to use (sq_tail) is not still outstanding at the device.
        self.can_submit = can_submit = Signal()
        self.comb += can_submit.eq(self.enable & self.sink.valid & (self.inflight < qd)
                                   & ~slot_busy[sq_tail])

        # Should we try to reap? Something outstanding.
        self.can_reap = can_reap = Signal()
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

        # Arbitration (coalesced doorbells):
        #   1. keep filling the in-flight window (defer the SQ doorbell to coalesce);
        #   2. once we can't submit more, flush the accumulated SQ doorbell ONCE;
        #   3. reap any ready completions (the reap path loops, draining a whole batch);
        #   4. flush the accumulated CQ doorbell ONCE.
        fsm.act("IDLE",
            If(can_submit,
                NextState("LATCH-REQ"),
            ).Elif(sq_tail != sq_db_rung,
                NextState("SUBMIT-DOORBELL"),
            ).Elif(can_reap,
                NextState("REAP-READ"),
            ).Elif(cq_head != cq_db_rung,
                NextState("REAP-DOORBELL"),
            )
        )

        # Latch the request and allocate a CID (== SQ slot index). Also latch the PRP
        # fields computed combinationally from the incoming request.
        fsm.act("LATCH-REQ",
            NextValue(req_op,   self.sink.op),
            NextValue(req_nsid, self.sink.nsid),
            NextValue(req_lba,  self.sink.lba),
            NextValue(req_nlb,  self.sink.nlb),
            NextValue(req_buf,  self.sink.buf),
            NextValue(req_cid,  sq_tail),
            NextValue(req_prp2,      s_prp2),
            NextValue(req_need_list, s_need_list),
            NextValue(req_nlist,     s_npages - 1),
            NextValue(req_list_slot, s_list_slot),
            NextValue(dw_idx,   0),
            NextValue(list_dw,  0),
            If(s_need_list,
                NextState("PRP-LIST-WRITE"),
            ).Else(
                NextState("SUBMIT-WRITE"),
            )
        )

        # Build the PRP list in host memory (only for transfers spanning >2 pages):
        # write `req_nlist` 64-bit entries (2 dwords each) to req_list_slot.
        fsm.act("PRP-LIST-WRITE",
            self.mem.stb.eq(1),
            self.mem.we.eq(1),
            self.mem.adr.eq((req_list_slot >> 2) + list_dw),
            self.mem.dat_w.eq(list_dw_val),
            If(self.mem.ack,
                If(list_dw == ((req_nlist << 1) - 1),
                    NextState("SUBMIT-WRITE"),
                ).Else(
                    NextValue(list_dw, list_dw + 1),
                )
            )
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

        # Advance SQ tail, consume the request, then return to arbitration WITHOUT ringing
        # the doorbell (coalesced): the SQ doorbell is rung once in IDLE when the window is
        # full or the request stream pauses (sq_tail != sq_db_rung).
        fsm.act("SUBMIT-ADVANCE",
            self.sink.ready.eq(1),  # request consumed (latched).
            ring_inc(sq_tail),
            NextValue(slot_busy[sq_tail], 1),  # mark this CID (= sq_tail) outstanding.
            NextValue(self.inflight, self.inflight + 1),
            NextValue(self.submitted, self.submitted + 1),
            NextState("IDLE"),
        )

        # NOTE: the MMIO accessor (litenvme/mem.py) samples we/adr/wdata COMBINATIONALLY in
        # its SEND state and holds there until the request is accepted (req_sink.ready),
        # latching nothing. So the engine MUST hold these inputs stable from the start pulse
        # through the WAIT state until mmio_done -- otherwise the accessor forms the write
        # TLP with address/data 0 and the doorbell never reaches BAR0. (This was the bug
        # that made the SSD never fetch: a valid SQE was in place but the doorbell TLP was
        # malformed.) mmio_start is a one-cycle pulse (the accessor edge-detects it); the
        # address/data are held across both states below.
        fsm.act("SUBMIT-DOORBELL",
            self.mmio_start.eq(1),
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.sq_db_adr),
            # Ring with the current tail (covers every SQE written since the last ring).
            self.mmio_wdata.eq(sq_tail),
            NextValue(sq_db_rung, sq_tail),
            NextState("SUBMIT-DOORBELL-WAIT"),
        )
        fsm.act("SUBMIT-DOORBELL-WAIT",
            # Hold the request payload stable while the accessor sends it.
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.sq_db_adr),
            self.mmio_wdata.eq(sq_tail),
            If(self.mmio_done,
                NextState("IDLE"),
            )
        )

        # Reap: read the CQE status dword (dw3 = phase|status|cid) FIRST and check the phase
        # bit before reading anything else. The device writes the CQE payload before flipping
        # the phase bit, and dw3 is a single atomic 32-bit read; so observing the new phase
        # guarantees the whole entry is committed. Reading dw3-first (instead of dw0..dw3 in
        # order) avoids a torn read at the ring wrap, where the engine could otherwise sample
        # dw0..dw2 from the PREVIOUS entry, then catch the just-flipped phase in dw3 and emit
        # a completion with a stale cid/sqhd. (Observed on HW as exactly one bad CQE at
        # completion index == qsize; sim's atomic CQE model never exposed it.)
        fsm.act("REAP-READ",
            self.mem.stb.eq(1),
            self.mem.we.eq(0),
            self.mem.adr.eq((cq_slot_adr >> 2) + 3),  # dw3: phase/status/cid (atomic).
            If(self.mem.ack,
                NextValue(cqe3, self.mem.dat_r),
                NextState("REAP-CHECK"),
            )
        )

        # Check the phase bit. If matched -> the entry is complete; fetch sqhd then emit.
        fsm.act("REAP-CHECK",
            If(cqe_phase == cq_phase,
                NextState("REAP-READ-SQHD"),
            ).Elif(cq_head != cq_db_rung,
                # No more ready entries: flush the accumulated CQ doorbell once for this batch.
                NextState("REAP-DOORBELL"),
            ).Else(
                # Nothing ready and nothing to ack: back to arbitration.
                NextState("IDLE"),
            )
        )

        # Phase matched: read dw2 (sqhd). Safe now -- the device wrote the payload before the
        # phase, so dw2 is the current entry's.
        fsm.act("REAP-READ-SQHD",
            self.mem.stb.eq(1),
            self.mem.we.eq(0),
            self.mem.adr.eq((cq_slot_adr >> 2) + 2),  # dw2: sqhd/sqid.
            If(self.mem.ack,
                NextValue(cqe2, self.mem.dat_r),
                NextState("REAP-EMIT"),
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
                NextValue(slot_busy[cqe_cid], 0),  # this CID is now retired -> reusable.
                # Advance CQ head; toggle phase on wrap.
                If(cq_head == (qsize - 1),
                    NextValue(cq_head, 0),
                    NextValue(cq_phase, ~cq_phase),
                ).Else(
                    NextValue(cq_head, cq_head + 1),
                ),
                # Loop to drain the next ready completion (coalesced): the CQ doorbell is rung
                # once per batch from REAP-CHECK when no more entries have flipped phase.
                NextState("REAP-READ"),
            )
        )

        fsm.act("REAP-DOORBELL",
            self.mmio_start.eq(1),
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.cq_db_adr),
            self.mmio_wdata.eq(cq_head),  # ring with the current head (covers the whole batch).
            NextValue(cq_db_rung, cq_head),
            NextState("REAP-DOORBELL-WAIT"),
        )
        fsm.act("REAP-DOORBELL-WAIT",
            # Hold the request payload stable while the accessor sends it (see SUBMIT note).
            self.mmio_we.eq(1),
            self.mmio_adr.eq(self.cq_db_adr),
            self.mmio_wdata.eq(cq_head),
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
        self._prp_list_lo = CSRStorage(32, description="PRP-list region (host byte addr) low.")
        self._prp_list_hi = CSRStorage(32, description="PRP-list region (host byte addr) high.")
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
            self.prp_list_base.eq(Cat(self._prp_list_lo.storage, self._prp_list_hi.storage)),
            self._status.fields.busy.eq(self.busy),
            self._status.fields.inflight.eq(self.inflight),
            self._submitted.status.eq(self.submitted),
            self._completed.status.eq(self.completed),
        ]


# Mem-port -> AXI bridge ---------------------------------------------------------------------------

class LiteNVMeMemPortToAXI(LiteXModule):
    """Adapt a dword `LiteNVMeMemPort` to an AXI master (single-beat, dword-lane).

    The engine accesses host memory at dword granularity, but the hostmem backend is
    beat-wide (data_width) with byte strobes. This bridge maps each dword access to a
    single-beat AXI transaction on the enclosing beat:

      word_adr = dword_adr >> beat_dwords_shift     # beat (AXI word) index
      lane     = dword_adr & (beat_dwords - 1)       # dword lane within the beat
      byte_adr = word_adr << beat_bytes_shift        # AXI byte address

    - Write: AW + W with data placed in `lane` and strb = 0xf on that lane's bytes
      (the backend uses byte strobes, so only the addressed dword is modified — no RMW).
    - Read : AR (len=0); on R, extract the `lane` dword into `mem.dat_r`.

    `mem.ack` is asserted for exactly one cycle when the AXI response arrives, with
    `mem.dat_r` valid that cycle (matching the engine's mem-port contract). One IDLE
    bubble separates consecutive transfers.
    """
    def __init__(self, mem, data_width=128, address_width=32, base=0x0000_0000):
        # `base` is the host-memory window base. The engine addresses host memory with full
        # host byte addresses (e.g. sq_base = 0x1000_4000), but the shared backend RAM is
        # flat and zero-based, so we de-base to a window-relative address before driving
        # AXI -- exactly as the PCIe-DMA port does (`first_word = (req.adr - base) >> ...`).
        # If base is wrong, the engine's SQE/CQE accesses miss the bytes the SSD/DMA touch
        # and no command ever completes. base must be beat-aligned (it is: 0x1000_0000).
        self.mem = mem
        self.axi = axi.AXIInterface(
            data_width    = data_width,
            address_width = address_width,
            id_width      = 1,
            mode          = "rw",
        )

        beat_bytes        = data_width // 8
        beat_dwords       = data_width // 32
        beat_bytes_shift  = log2_int(beat_bytes)
        beat_dwords_shift = log2_int(beat_dwords)
        base_dw           = base >> 2  # base as a dword index (base is beat-aligned).

        # # #

        dw_adr   = Signal(address_width)  # window-relative dword address.
        word_adr = Signal(address_width)
        lane     = Signal(max=max(2, beat_dwords))
        byte_adr = Signal(address_width)
        self.comb += [
            dw_adr.eq(mem.adr - base_dw),
            word_adr.eq(dw_adr >> beat_dwords_shift),
            lane.eq(dw_adr[:beat_dwords_shift] if beat_dwords_shift else 0),
            byte_adr.eq(word_adr << beat_bytes_shift),
        ]

        # Write data/strobe: place the dword in its lane, strobe only that lane.
        wdata = Signal(data_width)
        wstrb = Signal(beat_bytes)
        if beat_dwords == 1:
            self.comb += [wdata.eq(mem.dat_w), wstrb.eq(2**beat_bytes - 1)]
        else:
            self.comb += Case(lane, {
                i: [
                    wdata[32*i:32*(i+1)].eq(mem.dat_w),
                    wstrb[4*i:4*(i+1)].eq(0xf),
                ] for i in range(beat_dwords)
            })

        # Read data: extract the addressed lane.
        rdata_lane = Signal(32)
        if beat_dwords == 1:
            self.comb += rdata_lane.eq(self.axi.r.data[0:32])
        else:
            self.comb += Case(lane, {
                i: rdata_lane.eq(self.axi.r.data[32*i:32*(i+1)]) for i in range(beat_dwords)
            })

        aw_done = Signal()
        w_done  = Signal()

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(mem.stb,
                If(mem.we,
                    NextValue(aw_done, 0),
                    NextValue(w_done,  0),
                    NextState("WRITE"),
                ).Else(
                    NextState("READ-AR"),
                )
            )
        )
        fsm.act("WRITE",
            self.axi.aw.valid.eq(~aw_done),
            self.axi.aw.addr.eq(byte_adr),
            self.axi.aw.len.eq(0),
            self.axi.aw.size.eq(axi.AXSIZE[beat_bytes]),
            self.axi.aw.burst.eq(axi.BURST_INCR),
            self.axi.aw.id.eq(0),

            self.axi.w.valid.eq(~w_done),
            self.axi.w.data.eq(wdata),
            self.axi.w.strb.eq(wstrb),
            self.axi.w.last.eq(1),

            If(self.axi.aw.valid & self.axi.aw.ready, NextValue(aw_done, 1)),
            If(self.axi.w.valid  & self.axi.w.ready,  NextValue(w_done,  1)),
            If((aw_done | (self.axi.aw.valid & self.axi.aw.ready)) &
               (w_done  | (self.axi.w.valid  & self.axi.w.ready)),
                NextState("WRITE-B"),
            )
        )
        fsm.act("WRITE-B",
            self.axi.b.ready.eq(1),
            If(self.axi.b.valid,
                mem.ack.eq(1),
                NextState("IDLE"),
            )
        )
        fsm.act("READ-AR",
            self.axi.ar.valid.eq(1),
            self.axi.ar.addr.eq(byte_adr),
            self.axi.ar.len.eq(0),
            self.axi.ar.size.eq(axi.AXSIZE[beat_bytes]),
            self.axi.ar.burst.eq(axi.BURST_INCR),
            self.axi.ar.id.eq(0),
            If(self.axi.ar.ready,
                NextState("READ-R"),
            )
        )
        fsm.act("READ-R",
            self.axi.r.ready.eq(1),
            If(self.axi.r.valid,
                mem.dat_r.eq(rdata_lane),
                mem.ack.eq(1),
                NextState("IDLE"),
            )
        )



# Engine + AXI bridge wrapper ----------------------------------------------------------------------

class LiteNVMeIOEngineAXI(LiteXModule):
    """LiteNVMeIOEngine + LiteNVMeMemPortToAXI, exposing a single AXI master (`.axi`).

    Drop-in for wiring the engine into the hostmem responder's AXI arbiter alongside the
    PCIe DMA and CSR-debug masters. The engine's request/completion streams, MMIO
    doorbell port, config and status signals are re-exported.
    """
    def __init__(self, qid=1, qsize=64, qd=None, data_width=128, with_csr=True,
                 hostmem_base=0x0000_0000):
        # hostmem_base MUST equal the responder's window base so the bridge de-bases the
        # engine's full host addresses to the flat backend (see LiteNVMeMemPortToAXI).
        self.submodules.engine = engine = LiteNVMeIOEngine(
            qid=qid, qsize=qsize, qd=qd, mem_adr_width=32, with_csr=with_csr)
        self.submodules.bridge = bridge = LiteNVMeMemPortToAXI(
            engine.mem, data_width=data_width, address_width=32, base=hostmem_base)

        # Re-export.
        self.axi        = bridge.axi
        self.sink       = engine.sink
        self.source     = engine.source
        self.mmio_start = engine.mmio_start
        self.mmio_we    = engine.mmio_we
        self.mmio_adr   = engine.mmio_adr
        self.mmio_wdata = engine.mmio_wdata
        self.mmio_done  = engine.mmio_done
        self.mmio_err   = engine.mmio_err
        self.enable     = engine.enable
        self.sq_base    = engine.sq_base
        self.cq_base    = engine.cq_base
        self.sq_db_adr  = engine.sq_db_adr
        self.cq_db_adr  = engine.cq_db_adr
        self.prp_list_base = engine.prp_list_base
        self.inflight   = engine.inflight
        self.submitted  = engine.submitted
        self.completed  = engine.completed
        self.busy       = engine.busy

    def connect_mmio(self, mmio):
        return self.engine.connect_mmio(mmio)
