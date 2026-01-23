# gateware/hostmem.py
#
# BRAM-backed host memory target for NVMe (RootPort mode).
#
# - Sits on a LitePCIe crossbar SLAVE port (receives Requests from NVMe).
# - Implements a small memory window (base/size).
# - MemWr: writes into BRAM (1 beat/cycle).
# - MemRd: returns Completion w/ Data from BRAM (1 beat/cycle after 1-cycle BRAM latency),
#          with a small FIFO to handle backpressure cleanly.
#
# Bring-up oriented: simple, conservative, no dynamic slicing / NextValue(None).
#

from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream


# Helpers ------------------------------------------------------------------------------------------

def _shift_for_pow2(x):
    # x must be {1,2,4,8,16,32,...}
    # returns constant integer shift.
    s = 0
    v = x
    while v > 1:
        v >>= 1
        s += 1
    return s


class LiteNVMeHostMemResponder(LiteXModule):
    def __init__(self, port, base=0x10000000, size=0x20000, data_width=128, with_csr=True):
        self.port = port
        self.base = base
        self.size = size

        assert data_width in [64, 128, 256]
        beat_bytes  = data_width // 8
        beat_dwords = data_width // 32

        beat_bytes_shift  = _shift_for_pow2(beat_bytes)
        beat_dwords_shift = _shift_for_pow2(beat_dwords)

        # -----------------------------------------------------------------------------------------
        # BRAM: beat-wide words.
        # -----------------------------------------------------------------------------------------
        assert (size % beat_bytes) == 0
        depth_words = size // beat_bytes
        mem = Memory(data_width, depth_words)

        dma_rp = mem.get_port(has_re=True,  mode=READ_FIRST)
        dma_wp = mem.get_port(write_capable=True)
        csr_rp = mem.get_port(has_re=True,  mode=READ_FIRST)
        csr_wp = mem.get_port(write_capable=True)
        self.specials += mem, dma_rp, dma_wp, csr_rp, csr_wp

        # -----------------------------------------------------------------------------------------
        # CSR access (debug): 32-bit dword view over beat-wide BRAM via RMW.
        # -----------------------------------------------------------------------------------------
        dma_wr_count = Signal(32)
        dma_rd_count = Signal(32)

        if with_csr:
            self._csr_adr      = CSRStorage(32, description="HostMem dword address (index).")
            self._csr_wdata    = CSRStorage(32, description="HostMem write data.")
            self._csr_we       = CSRStorage(1,  description="HostMem write strobe (pulse).")
            self._csr_rdata    = CSRStatus(32,  description="HostMem read data.")
            self._dma_wr_count = CSRStatus(32, description="Count of DMA beats stored.")
            self._dma_rd_count = CSRStatus(32, description="Count of DMA beats served.")

            csr_dw_adr   = self._csr_adr.storage
            csr_word_adr = Signal(32)
            csr_lane     = Signal(max=beat_dwords)

            self.comb += [
                csr_word_adr.eq(csr_dw_adr >> beat_dwords_shift),
                csr_lane.eq(csr_dw_adr[:beat_dwords_shift]),
            ]

            csr_word = Signal(data_width)
            self.sync += csr_word.eq(csr_rp.dat_r)

            csr_lane_val = Signal(32)
            self.comb += Case(csr_lane, {
                i: csr_lane_val.eq(csr_word[32*i:32*(i+1)]) for i in range(beat_dwords)
            })

            self.comb += [
                csr_rp.adr.eq(csr_word_adr),
                csr_rp.re.eq(1),
                self._csr_rdata.status.eq(csr_lane_val),

                self._dma_wr_count.status.eq(dma_wr_count),
                self._dma_rd_count.status.eq(dma_rd_count),
            ]

            csr_fsm = FSM(reset_state="CSR_IDLE")
            self.submodules += csr_fsm
            csr_shadow = Signal(data_width)

            csr_fsm.act("CSR_IDLE",
                If(self._csr_we.storage,
                    NextState("CSR_RMW_READ")
                )
            )
            csr_fsm.act("CSR_RMW_READ",
                NextValue(csr_shadow, csr_word),
                NextState("CSR_RMW_WRITE")
            )

            csr_updated = Signal(data_width)
            self.comb += csr_updated.eq(csr_shadow)
            self.comb += Case(csr_lane, {
                i: csr_updated[32*i:32*(i+1)].eq(self._csr_wdata.storage) for i in range(beat_dwords)
            })

            csr_fsm.act("CSR_RMW_WRITE",
                csr_wp.adr.eq(csr_word_adr),
                csr_wp.dat_w.eq(csr_updated),
                csr_wp.we.eq(1),
                NextState("CSR_IDLE")
            )
        else:
            self.comb += [csr_rp.re.eq(0), csr_wp.we.eq(0)]

        # -----------------------------------------------------------------------------------------
        # Port field helpers.
        # -----------------------------------------------------------------------------------------
        req = port.sink
        cmp = port.source

        def ep_dat(ep):
            if hasattr(ep, "dat"):
                return ep.dat
            if hasattr(ep, "data"):
                return ep.data
            raise ValueError("Endpoint has no dat/data field.")

        req_dat = ep_dat(req)
        cmp_dat = ep_dat(cmp)

        has_be    = hasattr(req, "be")
        has_tag   = hasattr(req, "tag")
        has_len   = hasattr(req, "len")
        has_reqid = hasattr(req, "req_id")
        has_cmpid = hasattr(cmp, "cmp_id")
        has_end   = hasattr(cmp, "end")

        in_window = Signal()
        self.comb += in_window.eq((req.adr >= base) & (req.adr < (base + size)))

        lat_tag    = Signal(8)
        lat_len_dw = Signal(10)
        lat_req_id = Signal(16)

        cur_word    = Signal(32)
        beats_total = Signal(16)
        beats_left  = Signal(16)
        beats_sent  = Signal(16)

        # -----------------------------------------------------------------------------------------
        # Read FIFO (data only; metadata comes from latched header).
        # -----------------------------------------------------------------------------------------
        self.submodules.rd_fifo = rd_fifo = stream.SyncFIFO([("dat", data_width)], depth=16, buffered=True)

        # 1-cycle BRAM read return tracker.
        re_d = Signal(reset=0)

        # 1-beat return skid buffer.
        ret_valid = Signal(reset=0)
        ret_data  = Signal(data_width)

        # -----------------------------------------------------------------------------------------
        # Defaults.
        # -----------------------------------------------------------------------------------------
        self.comb += [
            req.ready.eq(0),
            dma_rp.re.eq(0),
            dma_wp.we.eq(0),

            rd_fifo.sink.valid.eq(0),
            rd_fifo.sink.dat.eq(0),
        ]

        # Completions always driven from FIFO.
        self.comb += [
            cmp.valid.eq(rd_fifo.source.valid),
            cmp_dat.eq(rd_fifo.source.dat),
            rd_fifo.source.ready.eq(cmp.ready),
        ]

        # Completion metadata from latched header.
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
            self.comb += cmp.last.eq((beats_total != 0) & (beats_sent == (beats_total - 1)))
        if has_end:
            self.comb += cmp.end.eq((beats_total != 0) & (beats_sent == (beats_total - 1)))

        full_be_ok = Signal(reset=1)
        if has_be:
            self.comb += full_be_ok.eq(req.be == (2**beat_bytes - 1))

        # -----------------------------------------------------------------------------------------
        # FSM.
        # -----------------------------------------------------------------------------------------
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
                    NextValue(beats_total, (req.len + (beat_dwords - 1)) >> beat_dwords_shift),
                    NextValue(beats_left,  (req.len + (beat_dwords - 1)) >> beat_dwords_shift),
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

        # Combinatorial helpers for IDLE MemWr first-beat consume.
        first_word      = Signal(32)
        total_beats_now = Signal(16)
        self.comb += [
            first_word.eq((req.adr - base) >> beat_bytes_shift),
            total_beats_now.eq(((req.len + (beat_dwords - 1)) >> beat_dwords_shift) if has_len else 1),
        ]

        # IDLE: accept one request header/beat.
        fsm.act("IDLE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                If(~in_window,
                    NextState("DROP")
                ).Else(
                    *latch_req_header(),
                    If(req.we,
                        # Consume + store first beat here (header beat is also data beat).
                        dma_wp.adr.eq(first_word),
                        dma_wp.dat_w.eq(req_dat),
                        dma_wp.we.eq(1),
                        NextValue(dma_wr_count, dma_wr_count + 1),

                        If(total_beats_now > 1,
                            # Remaining beats will be consumed in WR_STREAM.
                            NextValue(cur_word,  first_word + 1),
                            NextValue(beats_left, total_beats_now - 1),
                            NextState("WR_STREAM")
                        ).Else(
                            # Single-beat write done.
                            NextValue(beats_left, 0),
                            NextState("IDLE")
                        )
                    ).Else(
                        # New MemRd: clear read pipeline and go read stream.
                        NextValue(re_d, 0),
                        NextValue(ret_valid, 0),
                        NextState("RD_STREAM")
                    )
                )
            )
        )

        # DROP: eat a request we don't handle (outside window).
        fsm.act("DROP",
            req.ready.eq(1),
            If(req.valid & req.ready & getattr(req, "last", 1),
                NextState("IDLE")
            )
        )

        # MemWr: write one beat per cycle (remaining beats only).
        # IMPORTANT: only accept beats when req.we==1. If a read shows up unexpectedly, stall.
        fsm.act("WR_STREAM",
            req.ready.eq((beats_left != 0) & req.we),
            If(req.valid & req.ready,
                dma_wp.adr.eq(cur_word),
                dma_wp.dat_w.eq(req_dat),
                dma_wp.we.eq(1),

                NextValue(dma_wr_count, dma_wr_count + 1),
                NextValue(cur_word, cur_word + 1),
                NextValue(beats_left, beats_left - 1),

                If(beats_left == 1,
                    NextState("IDLE")
                )
            )
        )

        # MemRd control.
        # We allow at most one outstanding BRAM read, and use a 1-beat skid buffer.
        fifo_has_room = Signal()
        self.comb += fifo_has_room.eq(rd_fifo.level < (rd_fifo.depth - 1))

        can_issue = Signal()
        self.comb += can_issue.eq((beats_left != 0) & fifo_has_room & (~ret_valid) & (~re_d))

        fsm.act("RD_STREAM",
            req.ready.eq(0),

            # Push buffered return into FIFO.
            rd_fifo.sink.valid.eq(ret_valid),
            rd_fifo.sink.dat.eq(ret_data),

            # Issue BRAM read (address increments per beat).
            If(can_issue,
                dma_rp.adr.eq(cur_word),
                dma_rp.re.eq(1),
                NextValue(cur_word, cur_word + 1),
                NextValue(beats_left, beats_left - 1),
                NextValue(dma_rd_count, dma_rd_count + 1),
            ),

            # Done once all beats have been popped and everything is drained.
            If((beats_total != 0) &
               (beats_sent == beats_total) &
               (beats_left == 0) &
               (~re_d) &
               (~ret_valid) &
               (~rd_fifo.source.valid),
                NextState("IDLE")
            )
        )

        # BRAM 1-cycle latency tracker.
        self.sync += re_d.eq(dma_rp.re)

        # Capture BRAM return and manage skid buffer.
        self.sync += If(re_d,
            # capture return
            ret_data.eq(dma_rp.dat_r),
            ret_valid.eq(1)
        ).Elif(ret_valid & rd_fifo.sink.ready,
            # consumed into FIFO
            ret_valid.eq(0)
        )

        # Count beats sent (downstream pops).
        self.sync += If(pop,
            beats_sent.eq(beats_sent + 1)
        )
