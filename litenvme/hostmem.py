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

# CSR Debug Frontend -------------------------------------------------------------------------------

class LiteNVMeHostMemCSR(LiteXModule):
    """CSR debug access to HostMem.

    Exposes a DWORD view on the beat-wide backend:
      - _csr_adr is a DWORD index inside the HostMem window.
      - Readback always tracks the selected DWORD.
      - Writes are done with a RMW on the enclosing beat.

    Backend requests are exported as bk_* and must be arbitrated at top-level.
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

        # Backend request-side signals (arbitrated in top-level).
        self.bk_w_adr  = Signal(32)
        self.bk_w_data = Signal(data_width)
        self.bk_w_en   = Signal()

        self.bk_r_adr  = Signal(32)
        self.bk_r_en   = Signal()
        self.bk_r_data = Signal(data_width)  # driven by top-level from backend.r_data

        # # #

        csr_dw_adr   = self._csr_adr.storage
        csr_word_adr = Signal(32)
        csr_lane     = Signal(max=beat_dwords)

        self.comb += [
            csr_word_adr.eq(csr_dw_adr >> beat_dwords_shift),
            csr_lane.eq(csr_dw_adr[:beat_dwords_shift]),
        ]

        # Default backend requests.
        self.comb += [
            self.bk_w_en.eq(0),
            self.bk_w_adr.eq(csr_word_adr),
            self.bk_w_data.eq(0),

            self.bk_r_en.eq(1),
            self.bk_r_adr.eq(csr_word_adr),
        ]

        # Registered view of backend read data.
        csr_word = Signal(data_width)
        self.sync += csr_word.eq(self.bk_r_data)

        # DWORD lane extract.
        csr_lane_val = Signal(32)
        self.comb += Case(csr_lane, {
            i: csr_lane_val.eq(csr_word[32*i:32*(i+1)]) for i in range(beat_dwords)
        })
        self.comb += self._csr_rdata.status.eq(csr_lane_val)

        # RMW write (beat-wide update of selected lane).
        self.fsm = fsm = FSM(reset_state="IDLE")
        shadow   = Signal(data_width)
        updated  = Signal(data_width)

        self.comb += updated.eq(shadow)
        self.comb += Case(csr_lane, {
            i: updated[32*i:32*(i+1)].eq(self._csr_wdata.storage) for i in range(beat_dwords)
        })

        fsm.act("IDLE",
            If(self._csr_we.storage,
                NextState("READ"),
            )
        )
        fsm.act("READ",
            NextValue(shadow, csr_word),
            NextState("WRITE"),
        )
        fsm.act("WRITE",
            self.bk_w_en.eq(1),
            self.bk_w_data.eq(updated),
            NextState("IDLE"),
        )

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

        # Backend request-side signals (arbitrated in top-level).
        self.bk_w_adr  = Signal(32)
        self.bk_w_data = Signal(data_width)
        self.bk_w_en   = Signal()

        self.bk_r_adr  = Signal(32)
        self.bk_r_en   = Signal()
        self.bk_r_data = Signal(data_width)  # driven by top-level from backend.r_data

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

        # Read FIFO: (dat,last) to drive last/end reliably under backpressure.
        self.submodules.rd_fifo = rd_fifo = stream.SyncFIFO(
            [("dat", data_width)],
            depth    = 64,
            buffered = False,
        )

        # Defaults.
        self.comb += [
            req.ready.eq(0),

            self.bk_w_en.eq(0),
            self.bk_w_adr.eq(0),
            self.bk_w_data.eq(0),

            self.bk_r_en.eq(0),
            self.bk_r_adr.eq(0),

            rd_fifo.sink.valid.eq(0),
            rd_fifo.sink.dat.eq(0),
            rd_fifo.sink.last.eq(0),
        ]

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

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

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
            req.ready.eq(1),
            If(req.valid & req.ready,
                If(~in_window | ~full_be_ok,
                    NextState("DROP"),
                ).Else(
                    *latch_req_header(),
                    If(req.we,
                        # MemWr: first beat is also data beat.
                        self.bk_w_adr.eq(first_word),
                        self.bk_w_data.eq(req_dat),
                        self.bk_w_en.eq(1),
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
            req.ready.eq(beats_left != 0),
            If(req.valid & req.ready,
                self.bk_w_adr.eq(cur_word),
                self.bk_w_data.eq(req_dat),
                self.bk_w_en.eq(1),

                NextValue(self.wr_count, self.wr_count + 1),
                NextValue(cur_word,     cur_word + 1),
                NextValue(beats_left,   beats_left - 1),

                If(beats_left == 1,
                    NextState("IDLE"),
                )
            )
        )

        # Read pipeline: issue read, enqueue next cycle.
        issue   = Signal()
        issue_d = Signal()
        last_d  = Signal()

        self.comb += issue.eq(fsm.ongoing("RD_STREAM") & (beats_left != 0) & rd_fifo.sink.ready)

        self.comb += [
            self.bk_r_adr.eq(cur_word),
            self.bk_r_en.eq(issue),
        ]

        self.comb += [
            rd_fifo.sink.valid.eq(issue_d),
            rd_fifo.sink.dat.eq(self.bk_r_data),
            rd_fifo.sink.last.eq(last_d),
        ]

        fsm.act("RD_STREAM",
            If((beats_total != 0) &
               (beats_sent  == beats_total) &
               (beats_left  == 0) &
               (~issue_d) &
               (~rd_fifo.source.valid),
                NextState("IDLE"),
            )
        )

        self.sync += [
            issue_d.eq(issue),
            last_d.eq(issue & (beats_left == 1)),

            If(issue,
                cur_word.eq(cur_word + 1),
                beats_left.eq(beats_left - 1),
                self.rd_count.eq(self.rd_count + 1),
            ),
            If(pop,
                beats_sent.eq(beats_sent + 1),
            ),
        ]

# Top-Level Responder ------------------------------------------------------------------------------

class LiteNVMeHostMemResponder(LiteXModule):
    """Host Memory responder.

    Wraps:
      - Backend (SRAM today).
      - DMA frontend (PCIe MemRd/MemWr handling).
      - Optional CSR debug frontend.

    Arbitration:
      - DMA has priority on backend access.
      - CSR debug reads are best-effort and stall during DMA activity.
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

        self.backend = backend = LiteNVMeHostMemBackendSRAM(
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

        # Backend arbitration (DMA priority).

        self.comb += [
            backend.w_en.eq(0),
            backend.w_adr.eq(0),
            backend.w_data.eq(0),

            backend.r_en.eq(0),
            backend.r_adr.eq(0),
        ]

        # Read data fanout.
        self.comb += dma.bk_r_data.eq(backend.r_data)
        if csr is not None:
            self.comb += csr.bk_r_data.eq(backend.r_data)

        # Write mux.
        self.comb += If(dma.bk_w_en,
            backend.w_en.eq(1),
            backend.w_adr.eq(dma.bk_w_adr),
            backend.w_data.eq(dma.bk_w_data),
        ).Elif((csr is not None) & csr.bk_w_en,
            backend.w_en.eq(1),
            backend.w_adr.eq(csr.bk_w_adr),
            backend.w_data.eq(csr.bk_w_data),
        )

        # Read mux.
        self.comb += If(dma.bk_r_en,
            backend.r_en.eq(1),
            backend.r_adr.eq(dma.bk_r_adr),
        ).Elif(csr is not None,
            backend.r_en.eq(csr.bk_r_en),
            backend.r_adr.eq(csr.bk_r_adr),
        )
