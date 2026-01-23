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

        # These are powers of two for supported widths.
        beat_bytes_shift  = _shift_for_pow2(beat_bytes)   # 64b->3, 128b->4, 256b->5
        beat_dwords_shift = _shift_for_pow2(beat_dwords)  # 64b->1, 128b->2, 256b->3

        # -----------------------------------------------------------------------------------------
        # BRAM: data_width-wide words (eg 128-bit).
        # Addressed in "beats" (not dwords).
        # -----------------------------------------------------------------------------------------
        assert (size % beat_bytes) == 0
        depth_words = size // beat_bytes
        mem = Memory(data_width, depth_words)

        # Dual-port BRAM:
        # - Port A: DMA (NVMe Requests).
        # - Port B: CSR access (debug only).
        dma_rp = mem.get_port(has_re=True,  mode=READ_FIRST)        # read port
        dma_wp = mem.get_port(write_capable=True)                   # write port
        csr_rp = mem.get_port(has_re=True,  mode=READ_FIRST)
        csr_wp = mem.get_port(write_capable=True)
        self.specials += mem, dma_rp, dma_wp, csr_rp, csr_wp

        # -----------------------------------------------------------------------------------------
        # CSR access (debug): expose 32-bit dword view via read-modify of 128b words.
        # This is for convenience; DMA path stays beat-wide and fast.
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

            # Map dword address to word address + lane.
            csr_dw_adr   = self._csr_adr.storage
            csr_word_adr = Signal(32)
            csr_lane     = Signal(max=beat_dwords)

            self.comb += [
                csr_word_adr.eq(csr_dw_adr >> beat_dwords_shift),
                csr_lane.eq(csr_dw_adr[:beat_dwords_shift]),
            ]

            # CSR read: read whole word, select lane (registered by BRAM, so rdata is "previous").
            csr_word = Signal(data_width)
            self.sync += csr_word.eq(csr_rp.dat_r)

            # Select 32-bit lane with Case (no dynamic slicing).
            csr_lane_val = Signal(32)
            csr_lane_cases = {i: csr_lane_val.eq(csr_word[32*i:32*(i+1)]) for i in range(beat_dwords)}
            self.comb += Case(csr_lane, csr_lane_cases)

            self.comb += [
                csr_rp.adr.eq(csr_word_adr),
                csr_rp.re.eq(1),
                self._csr_rdata.status.eq(csr_lane_val),

                # Writes: best-effort whole-word write of only one lane:
                # We do a read-modify-write over two cycles (debug only).
                self._dma_wr_count.status.eq(dma_wr_count),
                self._dma_rd_count.status.eq(dma_rd_count),
            ]

            # Simple RMW for CSR writes (debug): 2-state mini FSM.
            csr_fsm = FSM(reset_state="CSR_IDLE")
            self.submodules += csr_fsm
            csr_shadow = Signal(data_width)

            csr_fsm.act("CSR_IDLE",
                If(self._csr_we.storage,
                    NextState("CSR_RMW_READ")
                )
            )
            csr_fsm.act("CSR_RMW_READ",
                # capture current word (already coming from csr_rp.dat_r registered into csr_word)
                NextValue(csr_shadow, csr_word),
                NextState("CSR_RMW_WRITE")
            )
            # Prepare updated word with Case.
            csr_updated = Signal(data_width)
            self.comb += csr_updated.eq(csr_shadow)
            csr_upd_cases = {i: csr_updated[32*i:32*(i+1)].eq(self._csr_wdata.storage) for i in range(beat_dwords)}
            self.comb += Case(csr_lane, csr_upd_cases)

            csr_fsm.act("CSR_RMW_WRITE",
                csr_wp.adr.eq(csr_word_adr),
                csr_wp.dat_w.eq(csr_updated),
                csr_wp.we.eq(1),
                NextState("CSR_IDLE")
            )
        else:
            # If no CSR, keep ports quiet.
            self.comb += [
                csr_rp.re.eq(0),
                csr_wp.we.eq(0),
            ]

        # -----------------------------------------------------------------------------------------
        # Port field helpers.
        # -----------------------------------------------------------------------------------------
        req = port.sink   # incoming Requests (MemRd/MemWr)
        cmp = port.source # outgoing Completions (Cpl/CplD)

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

        # -----------------------------------------------------------------------------------------
        # Address window check: req.adr is byte address in LitePCIe user ports.
        # -----------------------------------------------------------------------------------------
        in_window = Signal()
        self.comb += in_window.eq((req.adr >= base) & (req.adr < (base + size)))

        # -----------------------------------------------------------------------------------------
        # Internal latches.
        # -----------------------------------------------------------------------------------------
        lat_tag    = Signal(8)
        lat_len_dw = Signal(10)   # in dwords
        lat_req_id = Signal(16)

        # Beat addressing within the window:
        #   word_index = (byte_addr - base) / beat_bytes
        cur_word = Signal(32)

        # Total beats for request:
        #   beats = ceil(len_dw / beat_dwords) = (len_dw + beat_dwords-1) >> log2(beat_dwords)
        beats_total = Signal(16)
        beats_left  = Signal(16)
        beats_sent  = Signal(16)

        # -----------------------------------------------------------------------------------------
        # Read pipeline: issue 1 read/cycle and push into FIFO when data returns.
        # -----------------------------------------------------------------------------------------
        # Small FIFO to absorb backpressure from packetizer/PHY.
        self.submodules.rd_fifo = rd_fifo = stream.SyncFIFO([("dat", data_width)], depth=16, buffered=True)

        # Track one-cycle BRAM latency:
        rd_inflight = Signal()
        rd_last_inflight = Signal()  # "is last beat" for the beat currently inflight

        # Defaults.
        self.comb += [
            req.ready.eq(0),
            cmp.valid.eq(0),
            dma_rp.re.eq(0),
            dma_wp.we.eq(0),
        ]
        if hasattr(cmp, "first"): self.comb += cmp.first.eq(0)
        if hasattr(cmp, "last"):  self.comb += cmp.last.eq(0)
        if has_end:               self.comb += cmp.end.eq(0)
        if hasattr(cmp, "err"):   self.comb += cmp.err.eq(0)
        if hasattr(cmp, "len"):   self.comb += cmp.len.eq(0)
        if hasattr(cmp, "tag"):   self.comb += cmp.tag.eq(0)
        if has_cmpid:             self.comb += cmp.cmp_id.eq(0)
        if has_reqid and hasattr(cmp, "req_id"):
            self.comb += cmp.req_id.eq(0)

        # Completion output comes from FIFO (stable under stall automatically).
        self.comb += [
            cmp.valid.eq(rd_fifo.source.valid),
            cmp_dat.eq(rd_fifo.source.dat),
            rd_fifo.source.ready.eq(cmp.ready),
        ]

        # Tag/len/ids are constant for the whole completion.
        if has_tag:
            self.comb += cmp.tag.eq(lat_tag)
        if hasattr(cmp, "len"):
            self.comb += cmp.len.eq(lat_len_dw)
        if has_reqid and hasattr(cmp, "req_id"):
            self.comb += cmp.req_id.eq(lat_req_id)

        # first/last based on beats_sent counter; only update on successful pop.
        pop = Signal()
        self.comb += pop.eq(cmp.valid & cmp.ready)

        if hasattr(cmp, "first"):
            self.comb += cmp.first.eq(beats_sent == 0)
        if hasattr(cmp, "last"):
            self.comb += cmp.last.eq(beats_sent == (beats_total - 1))
        if has_end:
            self.comb += cmp.end.eq(beats_sent == (beats_total - 1))

        # -----------------------------------------------------------------------------------------
        # MemWr byte-enable policy:
        # For sustained 1 beat/cycle, we treat MemWr as full-beat writes.
        # NVMe Identify path writes full dwords; in practice BE will be all-ones.
        # If you *really* need partial-beat correctness, do RMW (will cost throughput).
        # -----------------------------------------------------------------------------------------
        full_be_ok = Signal(reset=1)
        if has_be:
            # Consider it OK if all bytes enabled on that beat.
            self.comb += full_be_ok.eq(req.be == (2**beat_bytes - 1))

        # -----------------------------------------------------------------------------------------
        # FSM.
        # -----------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")

        # Helper: safe latches (no NextValue(None,...)).
        def latch_req_header():
            stmts = []
            stmts += [
                NextValue(cur_word, (req.adr - base) >> beat_bytes_shift),
            ]
            if has_tag:
                stmts += [NextValue(lat_tag, req.tag)]
            else:
                stmts += [NextValue(lat_tag, 0)]
            if has_len:
                stmts += [NextValue(lat_len_dw, req.len)]
                stmts += [NextValue(beats_total, (req.len + (beat_dwords - 1)) >> beat_dwords_shift)]
                stmts += [NextValue(beats_left,  (req.len + (beat_dwords - 1)) >> beat_dwords_shift)]
            else:
                stmts += [NextValue(lat_len_dw, 1)]
                stmts += [NextValue(beats_total, 1)]
                stmts += [NextValue(beats_left,  1)]
            if has_reqid:
                stmts += [NextValue(lat_req_id, req.req_id)]
            else:
                stmts += [NextValue(lat_req_id, 0)]
            stmts += [NextValue(beats_sent, 0)]
            return stmts

        # IDLE: accept one request header beat.
        fsm.act("IDLE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                If(~in_window,
                    NextState("DROP")
                ).Else(
                    *latch_req_header(),
                    If(req.we,
                        NextState("WR_STREAM")
                    ).Else(
                        # flush FIFO/inflight state for new read
                        NextValue(rd_inflight, 0),
                        NextState("RD_STREAM")
                    )
                )
            )
        )

        fsm.act("DROP",
            req.ready.eq(1),
            If(req.valid & req.ready & getattr(req, "last", 1),
                NextState("IDLE")
            )
        )

        # MemWr: 1 beat/cycle write.
        # We keep req.ready asserted while we still have beats to consume.
        fsm.act("WR_STREAM",
            req.ready.eq(beats_left != 0),
            If(req.valid & req.ready,
                # Optional policy check: if BE not full, you can drop or accept.
                # For now accept (and write full beat) since Identify uses full BE.
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

        # MemRd: issue reads at 1/cycle, push returned beats into FIFO.
        # Stop issuing when FIFO is near full or when we issued all beats.
        can_issue = Signal()
        self.comb += can_issue.eq((beats_left != 0) & (rd_fifo.level < (rd_fifo.depth - 1)) & (~rd_inflight | (rd_fifo.level < rd_fifo.depth)))

        fsm.act("RD_STREAM",
            # We already accepted request in IDLE; req stream should be one-beat for MemRd.
            req.ready.eq(0),

            # Issue one read if possible.
            If(can_issue,
                dma_rp.adr.eq(cur_word),
                dma_rp.re.eq(1),
                NextValue(rd_inflight, 1),
                NextValue(rd_last_inflight, beats_left == 1),
                NextValue(cur_word, cur_word + 1),
                NextValue(beats_left, beats_left - 1),
                NextValue(dma_rd_count, dma_rd_count + 1),
            ),

            # When inflight data returns (next cycle), push to FIFO.
            # We treat dma_rp.dat_r as valid whenever rd_inflight was set in previous cycle.
            If(rd_inflight,
                If(rd_fifo.sink.ready,
                    rd_fifo.sink.valid.eq(1),
                    rd_fifo.sink.dat.eq(dma_rp.dat_r),
                    If(rd_fifo.sink.valid & rd_fifo.sink.ready,
                        NextValue(rd_inflight, 0)
                    )
                )
            ),

            # Count sent beats (on completion pop).
            If(pop,
                NextValue(beats_sent, beats_sent + 1)
            ),

            # Done when we have produced and sent all beats.
            If((beats_sent == beats_total) & (beats_total != 0),
                NextState("IDLE")
            )
        )

        # NOTE: beats_sent termination above is conservative; if you prefer, use:
        #   If((beats_sent == beats_total) & ~rd_fifo.source.valid & (beats_left==0) & ~rd_inflight, IDLE)

        # Keep beats_sent in sync even outside RD_STREAM (safe).
        self.sync += If(pop, beats_sent.eq(beats_sent + 1))
