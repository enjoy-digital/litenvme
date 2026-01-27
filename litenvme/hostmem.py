#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from litex.soc.interconnect import axi

# Helpers ------------------------------------------------------------------------------------------

def _shift_for_pow2(x):
    s = 0
    v = x
    while v > 1:
        v >>= 1
        s += 1
    return s

def _get_dat(ep):
    if hasattr(ep, "dat"):
        return ep.dat
    if hasattr(ep, "data"):
        return ep.data
    raise ValueError("Endpoint has no dat/data field.")

# Backend Interface --------------------------------------------------------------------------------

class LiteNVMeHostMemBackend(LiteXModule):
    """Beat-wide Host Memory backend interface.

    Addressing is in "beat words" (not bytes). A beat is `data_width` wide.

    Provides:
      - Write: (w_adr, w_data, w_en).
      - Read:  (r_adr, r_en) with synchronous return (r_data).
    """
    def __init__(self, data_width):
        self.data_width = data_width

        # Write (1 beat).
        self.w_adr   = Signal(32)
        self.w_data  = Signal(data_width)
        self.w_en    = Signal()

        # Read issue (1 beat) + synchronous data return.
        self.r_adr   = Signal(32)
        self.r_en    = Signal()
        self.r_data  = Signal(data_width)

# SRAM Backend -------------------------------------------------------------------------------------

class LiteNVMeHostMemBackendSRAM(LiteNVMeHostMemBackend):
    """SRAM Host Memory backend.

    Simple dual-port RAM:
      - Separate write/read ports.
      - Read is synchronous (1-cycle latency), READ_FIRST mode.
    """
    def __init__(self, size, data_width):
        super().__init__(data_width=data_width)

        beat_bytes  = data_width // 8
        assert (size % beat_bytes) == 0
        depth_words = size // beat_bytes

        mem = Memory(data_width, depth_words)
        rp  = mem.get_port(has_re=True, mode=READ_FIRST)
        wp  = mem.get_port(write_capable=True)
        self.specials += mem, rp, wp

        # # #

        self.comb += [
            # Write port.
            wp.adr.eq(self.w_adr),
            wp.dat_w.eq(self.w_data),
            wp.we.eq(self.w_en),

            # Read port.
            rp.adr.eq(self.r_adr),
            rp.re.eq(self.r_en),

            # Registered output from Memory port.
            self.r_data.eq(rp.dat_r),
        ]

# AXI SRAM Backend ---------------------------------------------------------------------------------

class LiteNVMeHostMemAXIRAM(LiteXModule):
    """AXI-MMAP Host Memory backend (SRAM).

    - Beat-wide accesses.
    - Single-beat AXI transactions (len=0).
    - Read latency is 1 cycle, with a small response FIFO to sustain 1 beat/clk.
    """
    def __init__(self, size, data_width, address_width=32, id_width=1, rd_fifo_depth=64):
        self.axi = axi.AXIInterface(
            data_width    = data_width,
            address_width = address_width,
            id_width      = id_width,
            mode          = "rw",
        )

        beat_bytes  = data_width // 8
        assert (size % beat_bytes) == 0
        depth_words = size // beat_bytes
        beat_bytes_shift = _shift_for_pow2(beat_bytes)

        mem = Memory(data_width, depth_words)
        rp  = mem.get_port(has_re=True, mode=READ_FIRST)
        wp  = mem.get_port(write_capable=True, we_granularity=8)
        # Debug read-only port (word address).
        self.dbg_adr  = Signal(32)
        self.dbg_en   = Signal()
        self.dbg_data = Signal(data_width)
        dbg_rp = mem.get_port(has_re=True, mode=READ_FIRST)
        self.specials += mem, rp, wp, dbg_rp

        # Read response FIFO (data/ids).
        self.submodules.rd_resp_fifo = rd_resp_fifo = stream.SyncFIFO(
            [("data", data_width), ("id", id_width)],
            depth    = rd_fifo_depth,
            buffered = False,
        )

        # Write-side latches.
        aw_pending = Signal()
        w_pending  = Signal()
        b_pending  = Signal()
        aw_addr    = Signal(address_width)
        aw_id      = Signal(id_width)
        w_data     = Signal(data_width)
        w_strb     = Signal(beat_bytes)
        b_id       = Signal(id_width)

        # # #

        # Defaults.
        self.comb += [
            # AXI write address/data handshakes.
            self.axi.aw.ready.eq(~aw_pending & ~b_pending),
            self.axi.w.ready.eq(~w_pending  & ~b_pending),

            # AXI write response.
            self.axi.b.valid.eq(b_pending),
            self.axi.b.resp.eq(axi.RESP_OKAY),
            self.axi.b.id.eq(b_id),

            # AXI read response.
            self.axi.r.valid.eq(rd_resp_fifo.source.valid),
            self.axi.r.data.eq(rd_resp_fifo.source.data),
            self.axi.r.id.eq(rd_resp_fifo.source.id),
            self.axi.r.last.eq(rd_resp_fifo.source.last),
            self.axi.r.resp.eq(axi.RESP_OKAY),
            rd_resp_fifo.source.ready.eq(self.axi.r.ready),
        ]

        # Read address acceptance (single-beat only).
        ar_ok = Signal()
        self.comb += [
            ar_ok.eq(self.axi.ar.len == 0),
            self.axi.ar.ready.eq(rd_resp_fifo.sink.ready & ar_ok),
        ]

        # Read issue to memory (1-cycle latency).
        issue_d = Signal()
        id_d    = Signal(id_width)

        ar_fire = Signal()
        self.comb += ar_fire.eq(self.axi.ar.valid & self.axi.ar.ready)
        self.comb += [
            rp.adr.eq(self.axi.ar.addr[beat_bytes_shift:]),
            rp.re.eq(ar_fire),
        ]

        self.sync += [
            issue_d.eq(ar_fire),
            If(ar_fire,
                id_d.eq(self.axi.ar.id),
            ),
        ]

        self.comb += [
            rd_resp_fifo.sink.valid.eq(issue_d),
            rd_resp_fifo.sink.data.eq(rp.dat_r),
            rd_resp_fifo.sink.id.eq(id_d),
            rd_resp_fifo.sink.last.eq(1),
        ]

        # Write commit to memory.
        write_fire = Signal()
        self.comb += write_fire.eq(aw_pending & w_pending & ~b_pending)

        self.comb += [
            wp.adr.eq(aw_addr[beat_bytes_shift:]),
            wp.dat_w.eq(w_data),
        ]
        for i in range(beat_bytes):
            self.comb += wp.we[i].eq(write_fire & w_strb[i])

        # Debug read port (word address).
        self.comb += [
            dbg_rp.adr.eq(self.dbg_adr),
            dbg_rp.re.eq(self.dbg_en),
            self.dbg_data.eq(dbg_rp.dat_r),
        ]

        self.sync += [
            # Latch AW.
            If(self.axi.aw.valid & self.axi.aw.ready,
                aw_pending.eq(1),
                aw_addr.eq(self.axi.aw.addr),
                aw_id.eq(self.axi.aw.id),
            ),
            # Latch W.
            If(self.axi.w.valid & self.axi.w.ready,
                w_pending.eq(1),
                w_data.eq(self.axi.w.data),
                w_strb.eq(self.axi.w.strb),
            ),
            # Commit write when both AW/W are available.
            If(write_fire,
                aw_pending.eq(0),
                w_pending.eq(0),
                b_pending.eq(1),
                b_id.eq(aw_id),
            ),
            # Complete write response.
            If(self.axi.b.valid & self.axi.b.ready,
                b_pending.eq(0),
            ),
        ]

# CSR Debug Frontend -------------------------------------------------------------------------------

class LiteNVMeHostMemCSR(LiteXModule):
    """CSR debug access to HostMem.

    Exposes a DWORD view on the beat-wide backend:
      - _csr_adr is a DWORD index inside the HostMem window.
      - Readback always tracks the selected DWORD.
      - Writes are done with a RMW on the enclosing beat.

    Uses an internal AXI-MMAP master port (single-beat transactions).
    """
    def __init__(self, data_width, beat_dwords, beat_dwords_shift, with_counters=True):
        # csr_adr:
        # - DWORD index (debug view).
        self._csr_adr = CSRStorage(32, description="HostMem dword address (index).")

        # csr_wdata:
        # - DWORD write payload (debug).
        self._csr_wdata = CSRStorage(32, description="HostMem write data (DWORD).")

        # csr_we:
        # - Pulse to perform the write (RMW on beat-wide backend).
        self._csr_we = CSRStorage(1, description="HostMem write strobe (pulse).")

        # csr_rdata:
        # - DWORD readback (debug).
        self._csr_rdata = CSRStatus(32, description="HostMem read data (DWORD).")

        if with_counters:
            self._dma_wr_count = CSRStatus(32, description="Count of DMA beats stored.")
            self._dma_rd_count = CSRStatus(32, description="Count of DMA beats served.")

        # AXI-MMAP master.
        self.axi = axi.AXIInterface(
            data_width    = data_width,
            address_width = 32,
            id_width      = 1,
            mode          = "rw",
        )

        # Debug read port (word address).
        self.dbg_r_adr  = Signal(32)
        self.dbg_r_en   = Signal()
        self.dbg_r_data = Signal(data_width)

        # # #

        beat_bytes       = data_width // 8
        beat_bytes_shift = _shift_for_pow2(beat_bytes)

        csr_dw_adr   = self._csr_adr.storage
        csr_word_adr = Signal(32)
        csr_lane     = Signal(max=beat_dwords)

        wr_pending      = Signal()
        wr_pending_adr  = Signal(32)
        wr_pending_data = Signal(32)
        wr_word_adr     = Signal(32)
        wr_lane         = Signal(max=beat_dwords)

        self.comb += [
            csr_word_adr.eq(csr_dw_adr >> beat_dwords_shift),
            csr_lane.eq(csr_dw_adr[:beat_dwords_shift]),
            wr_word_adr.eq(wr_pending_adr >> beat_dwords_shift),
            wr_lane.eq(wr_pending_adr[:beat_dwords_shift]),
        ]

        # Registered view of last read beat.
        csr_word      = Signal(data_width)
        last_read_adr = Signal(32)

        # DWORD lane extract.
        csr_lane_val = Signal(32)
        self.comb += Case(csr_lane, {
            i: csr_lane_val.eq(csr_word[32*i:32*(i+1)]) for i in range(beat_dwords)
        })
        self.comb += self._csr_rdata.status.eq(csr_lane_val)

        # RMW write (beat-wide update of selected lane).
        self.fsm = fsm = FSM(reset_state="IDLE")
        updated          = Signal(data_width)
        aw_sent          = Signal()
        w_sent           = Signal()
        csr_addr_latched = Signal(32)

        self.comb += updated.eq(csr_word)
        self.comb += Case(wr_lane, {
            i: updated[32*i:32*(i+1)].eq(wr_pending_data) for i in range(beat_dwords)
        })

        write_data = Signal(data_width)
        write_strb = Signal(beat_bytes)
        self.comb += Case(wr_lane, {
            i: [
                write_data[32*i:32*(i+1)].eq(wr_pending_data),
                write_strb[4*i:4*(i+1)].eq(0xf),
            ] for i in range(beat_dwords)
        })

        # Debug read port (continuous).
        self.comb += [
            self.dbg_r_en.eq(1),
            self.dbg_r_adr.eq(csr_word_adr),
        ]

        fsm.act("IDLE",
            If(wr_pending,
                NextValue(csr_addr_latched, wr_word_adr),
                NextState("WRITE-REQ"),
            )
        )
        fsm.act("WRITE-REQ",
            self.axi.aw.valid.eq(~aw_sent),
            self.axi.aw.addr.eq(csr_addr_latched << beat_bytes_shift),
            self.axi.aw.len.eq(0),
            self.axi.aw.size.eq(axi.AXSIZE[beat_bytes]),
            self.axi.aw.burst.eq(axi.BURST_INCR),
            self.axi.aw.id.eq(0),

            self.axi.w.valid.eq(~w_sent),
            self.axi.w.data.eq(write_data),
            self.axi.w.strb.eq(write_strb),
            self.axi.w.last.eq(1),

            If(aw_sent & w_sent,
                NextState("WRITE-RESP"),
            )
        )
        fsm.act("WRITE-RESP",
            self.axi.b.ready.eq(1),
            If(self.axi.b.valid,
                NextValue(last_read_adr, csr_addr_latched),
                NextValue(aw_sent, 0),
                NextValue(w_sent, 0),
                NextState("IDLE"),
            )
        )

        # Track AW/W handshakes.
        self.sync += [
            If(fsm.ongoing("WRITE-REQ") & self.axi.aw.valid & self.axi.aw.ready,
                aw_sent.eq(1),
            ),
            If(fsm.ongoing("WRITE-REQ") & self.axi.w.valid & self.axi.w.ready,
                w_sent.eq(1),
            ),
            If(fsm.ongoing("WRITE-REQ") & aw_sent & w_sent,
                csr_word.eq(Mux(last_read_adr == csr_addr_latched, updated, write_data)),
                last_read_adr.eq(csr_addr_latched),
            ).Elif(fsm.ongoing("WRITE-RESP") & self.axi.b.valid & self.axi.b.ready,
                csr_word.eq(Mux(last_read_adr == csr_addr_latched, updated, write_data)),
                last_read_adr.eq(csr_addr_latched),
            ).Else(
                csr_word.eq(self.dbg_r_data),
            ),
            If(~(fsm.ongoing("WRITE-REQ") | fsm.ongoing("WRITE-RESP")),
                aw_sent.eq(0),
                w_sent.eq(0),
            ),
        ]

        # Latch pending writes (avoid missing pulses while busy).
        self.sync += [
            If(self._csr_we.storage & ~wr_pending,
                wr_pending.eq(1),
                wr_pending_adr.eq(csr_dw_adr),
                wr_pending_data.eq(self._csr_wdata.storage),
            ),
            If(fsm.ongoing("WRITE-RESP") & self.axi.b.valid & self.axi.b.ready,
                wr_pending.eq(0),
            ),
        ]

# DMA / PCIe-side Frontend -------------------------------------------------------------------------

class LiteNVMeHostMemDMA(LiteXModule):
    """PCIe-side HostMem responder.

    Accepts MemRd/MemWr requests on LitePCIe user port and maps them to the
    beat-wide backend.

    Notes:
      - Only full-beat writes are accepted when BE is present.
      - Only one MemRd can be in-flight (completions are serialized).
      - Read path uses a small FIFO to tolerate completion backpressure.
    """
    def __init__(self, port, data_width, base, size, beat_bytes_shift, beat_dwords, beat_dwords_shift):
        self.port = port

        # AXI-MMAP master.
        self.axi = axi.AXIInterface(
            data_width    = data_width,
            address_width = 32,
            id_width      = 1,
            mode          = "rw",
        )

        # Counters.
        self.wr_count = Signal(32)
        self.rd_count = Signal(32)

        # # #

        req = port.sink
        cmp = port.source

        req_dat = _get_dat(req)
        cmp_dat = _get_dat(cmp)

        has_be    = hasattr(req, "be")
        has_tag   = hasattr(req, "tag")
        has_len   = hasattr(req, "len")
        has_reqid = hasattr(req, "req_id")
        has_cmpid = hasattr(cmp, "cmp_id")
        has_end   = hasattr(cmp, "end")

        # Address check.
        in_window = Signal()
        self.comb += in_window.eq((req.adr >= base) & (req.adr < (base + size)))

        # Only full-beat writes are supported when BE exists.
        full_be_ok = Signal(reset=1)
        if has_be:
            self.comb += full_be_ok.eq(req.be == (2**(data_width//8) - 1))

        # Latched completion metadata (single outstanding read).
        lat_tag    = Signal(8)
        lat_len_dw = Signal(10)
        lat_req_id = Signal(16)

        cur_word    = Signal(32)
        beats_total = Signal(16)
        beats_left  = Signal(16)
        beats_sent  = Signal(16)

        beat_bytes = data_width // 8
        axi_size   = axi.AXSIZE[beat_bytes]

        # Read FIFO: (dat,last) to drive last/end reliably under backpressure.
        self.submodules.rd_fifo = rd_fifo = stream.SyncFIFO(
            [("dat", data_width)],
            depth    = 64,
            buffered = False,
        )

        # Read "last" FIFO to align with AXI responses.
        self.submodules.rd_last_fifo = rd_last_fifo = stream.SyncFIFO(
            [("flag", 1)],
            depth    = 64,
            buffered = False,
        )

        # Write FIFO.
        self.submodules.wr_fifo = wr_fifo = stream.SyncFIFO(
            [("addr", 32), ("data", data_width), ("strb", beat_bytes)],
            depth    = 64,
            buffered = False,
        )

        # Completions driven from FIFO.
        self.comb += [
            cmp.valid.eq(rd_fifo.source.valid),
            cmp_dat.eq(rd_fifo.source.dat),
            rd_fifo.source.ready.eq(cmp.ready),
        ]

        # Completion metadata.
        if hasattr(cmp, "len"):
            self.comb += cmp.len.eq(lat_len_dw)
        if has_tag:
            self.comb += cmp.tag.eq(lat_tag)
        if has_reqid and hasattr(cmp, "req_id"):
            self.comb += cmp.req_id.eq(lat_req_id)
        if has_cmpid:
            self.comb += cmp.cmp_id.eq(0)
        if hasattr(cmp, "err"):
            self.comb += cmp.err.eq(0)

        pop = Signal()
        self.comb += pop.eq(cmp.valid & cmp.ready)

        if hasattr(cmp, "first"):
            self.comb += cmp.first.eq(beats_sent == 0)
        if hasattr(cmp, "last"):
            self.comb += cmp.last.eq(rd_fifo.source.last)
        if has_end:
            self.comb += cmp.end.eq(rd_fifo.source.last)

        def beats_from_len_dw(len_dw):
            # ceil(len_dw / beat_dwords)
            return (len_dw + (beat_dwords - 1)) >> beat_dwords_shift

        first_word      = Signal(32)
        total_beats_now = Signal(16)
        self.comb += [
            first_word.eq((req.adr - base) >> beat_bytes_shift),
            total_beats_now.eq(beats_from_len_dw(req.len) if has_len else 1),
        ]

        # AXI write engine.
        wr_active = Signal()
        aw_sent   = Signal()
        w_sent    = Signal()
        wr_addr   = Signal(32)
        wr_data   = Signal(data_width)
        wr_strb_l = Signal(beat_bytes)

        take_wr = Signal()
        self.comb += take_wr.eq(~wr_active & wr_fifo.source.valid)
        self.comb += wr_fifo.source.ready.eq(~wr_active)

        self.comb += [
            self.axi.aw.valid.eq(wr_active & ~aw_sent),
            self.axi.aw.addr.eq(wr_addr),
            self.axi.aw.len.eq(0),
            self.axi.aw.size.eq(axi_size),
            self.axi.aw.burst.eq(axi.BURST_INCR),
            self.axi.aw.id.eq(0),

            self.axi.w.valid.eq(wr_active & ~w_sent),
            self.axi.w.data.eq(wr_data),
            self.axi.w.strb.eq(wr_strb_l),
            self.axi.w.last.eq(1),

            self.axi.b.ready.eq(wr_active & aw_sent & w_sent),
        ]

        self.sync += [
            If(take_wr,
                wr_active.eq(1),
                wr_addr.eq(wr_fifo.source.addr),
                wr_data.eq(wr_fifo.source.data),
                wr_strb_l.eq(wr_fifo.source.strb),
                aw_sent.eq(0),
                w_sent.eq(0),
            ),
            If(wr_active & self.axi.aw.valid & self.axi.aw.ready,
                aw_sent.eq(1),
            ),
            If(wr_active & self.axi.w.valid & self.axi.w.ready,
                w_sent.eq(1),
            ),
            If(wr_active & aw_sent & w_sent & self.axi.b.valid & self.axi.b.ready,
                wr_active.eq(0),
                aw_sent.eq(0),
                w_sent.eq(0),
            ),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

        # Write push helpers.
        wr_word_adr = Signal(32)
        self.comb += wr_word_adr.eq(Mux(fsm.ongoing("WR_STREAM"), cur_word, first_word))
        wr_strb = Signal(beat_bytes)
        if has_be:
            self.comb += wr_strb.eq(req.be)
        else:
            self.comb += wr_strb.eq((2**beat_bytes) - 1)

        def latch_req_header():
            stmts = [NextValue(cur_word, (req.adr - base) >> beat_bytes_shift)]
            if has_tag:
                stmts += [NextValue(lat_tag, req.tag)]
            else:
                stmts += [NextValue(lat_tag, 0)]
            if has_len:
                stmts += [
                    NextValue(lat_len_dw, req.len),
                    NextValue(beats_total, beats_from_len_dw(req.len)),
                    NextValue(beats_left,  beats_from_len_dw(req.len)),
                ]
            else:
                stmts += [
                    NextValue(lat_len_dw, 1),
                    NextValue(beats_total, 1),
                    NextValue(beats_left,  1),
                ]
            if has_reqid:
                stmts += [NextValue(lat_req_id, req.req_id)]
            else:
                stmts += [NextValue(lat_req_id, 0)]
            stmts += [NextValue(beats_sent, 0)]
            return stmts

        fsm.act("IDLE",
            req.ready.eq(Mux(req.we, wr_fifo.sink.ready, 1)),
            If(req.valid & req.ready,
                If(~in_window | ~full_be_ok,
                    NextState("DROP"),
                ).Else(
                    *latch_req_header(),
                    If(req.we,
                        # MemWr: first beat is also data beat.
                        wr_fifo.sink.valid.eq(1),
                        wr_fifo.sink.addr.eq(first_word << beat_bytes_shift),
                        wr_fifo.sink.data.eq(req_dat),
                        wr_fifo.sink.strb.eq(wr_strb),
                        NextValue(self.wr_count, self.wr_count + 1),

                        If(total_beats_now > 1,
                            NextValue(cur_word,  first_word + 1),
                            NextValue(beats_left, total_beats_now - 1),
                            NextState("WR_STREAM"),
                        ).Else(
                            NextValue(beats_left, 0),
                            NextState("IDLE"),
                        )
                    ).Else(
                        NextValue(cur_word,  first_word),
                        NextValue(beats_left, total_beats_now),
                        NextState("RD_STREAM"),
                    )
                )
            )
        )

        fsm.act("DROP",
            req.ready.eq(1),
            If(req.valid & req.ready & getattr(req, "last", 1),
                NextState("IDLE"),
            )
        )

        fsm.act("WR_STREAM",
            req.ready.eq((beats_left != 0) & wr_fifo.sink.ready),
            If(req.valid & req.ready,
                wr_fifo.sink.valid.eq(1),
                wr_fifo.sink.addr.eq(wr_word_adr << beat_bytes_shift),
                wr_fifo.sink.data.eq(req_dat),
                wr_fifo.sink.strb.eq(wr_strb),

                NextValue(self.wr_count, self.wr_count + 1),
                NextValue(cur_word,     cur_word + 1),
                NextValue(beats_left,   beats_left - 1),

                If(beats_left == 1,
                    NextState("IDLE"),
                )
            )
        )

        # AXI read pipeline.
        rd_outstanding = Signal(16)
        rd_issue = Signal()
        rd_issue_last = Signal()
        self.comb += rd_issue.eq(fsm.ongoing("RD_STREAM") & (beats_left != 0) & rd_last_fifo.sink.ready)
        self.comb += rd_issue_last.eq(beats_left == 1)

        self.comb += [
            self.axi.ar.valid.eq(rd_issue),
            self.axi.ar.addr.eq(cur_word << beat_bytes_shift),
            self.axi.ar.len.eq(0),
            self.axi.ar.size.eq(axi_size),
            self.axi.ar.burst.eq(axi.BURST_INCR),
            self.axi.ar.id.eq(0),
        ]

        ar_fire = Signal()
        self.comb += ar_fire.eq(self.axi.ar.valid & self.axi.ar.ready)

        self.comb += [
            rd_last_fifo.sink.valid.eq(ar_fire),
            rd_last_fifo.sink.flag.eq(rd_issue_last),
        ]

        # AXI read responses to completion FIFO.
        r_fire = Signal()
        self.comb += [
            self.axi.r.ready.eq(rd_fifo.sink.ready & rd_last_fifo.source.valid),
            rd_fifo.sink.valid.eq(self.axi.r.valid & rd_last_fifo.source.valid),
            rd_fifo.sink.dat.eq(self.axi.r.data),
            rd_fifo.sink.last.eq(rd_last_fifo.source.flag),
            r_fire.eq(self.axi.r.valid & self.axi.r.ready),
            rd_last_fifo.source.ready.eq(r_fire),
        ]

        fsm.act("RD_STREAM",
            If((beats_total != 0) &
               (beats_sent  == beats_total) &
               (beats_left  == 0) &
               (rd_outstanding == 0) &
               (~rd_fifo.source.valid),
                NextState("IDLE"),
            )
        )

        self.sync += [
            If(ar_fire,
                cur_word.eq(cur_word + 1),
                beats_left.eq(beats_left - 1),
                self.rd_count.eq(self.rd_count + 1),
            ),
            If(ar_fire | r_fire,
                rd_outstanding.eq(rd_outstanding + ar_fire - r_fire),
            ),
            If(pop,
                beats_sent.eq(beats_sent + 1),
            ),
        ]

# Top-Level Responder ------------------------------------------------------------------------------

class LiteNVMeHostMemResponder(LiteXModule):
    """Host Memory responder.

    Wraps:
      - Backend (AXI SRAM).
      - DMA frontend (PCIe MemRd/MemWr handling).
      - Optional CSR debug frontend.
    """
    def __init__(self, port, base=0x10000000, size=0x20000, data_width=128, with_csr=True):
        self.port = port
        self.base = base
        self.size = size

        assert data_width in [64, 128, 256]
        beat_bytes  = data_width // 8
        beat_dwords = data_width // 32

        beat_bytes_shift  = _shift_for_pow2(beat_bytes)
        beat_dwords_shift = _shift_for_pow2(beat_dwords)

        # # #

        self.backend = backend = LiteNVMeHostMemAXIRAM(
            size       = size,
            data_width = data_width,
        )

        self.dma = dma = LiteNVMeHostMemDMA(
            port              = port,
            data_width        = data_width,
            base              = base,
            size              = size,
            beat_bytes_shift  = beat_bytes_shift,
            beat_dwords       = beat_dwords,
            beat_dwords_shift = beat_dwords_shift,
        )

        csr = None
        if with_csr:
            self.csr = csr = LiteNVMeHostMemCSR(
                data_width        = data_width,
                beat_dwords       = beat_dwords,
                beat_dwords_shift = beat_dwords_shift,
                with_counters     = True,
            )
            self.comb += [
                csr._dma_wr_count.status.eq(dma.wr_count),
                csr._dma_rd_count.status.eq(dma.rd_count),
            ]

        # CSR debug read port hookup.
        if csr is not None:
            self.comb += [
                backend.dbg_en.eq(csr.dbg_r_en),
                backend.dbg_adr.eq(csr.dbg_r_adr),
                csr.dbg_r_data.eq(backend.dbg_data),
            ]

        # AXI arbitration (DMA + optional CSR).
        if csr is not None:
            self.submodules.axi_arbiter = axi.AXIArbiter([dma.axi, csr.axi], backend.axi)
        else:
            self.comb += dma.axi.connect(backend.axi)
