#
# BRAM-backed host memory target for NVMe (RootPort mode).
#
# - Sits on a LitePCIe crossbar SLAVE port (receives Requests from NVMe).
# - Implements a small memory window (base/size).
# - MemWr: writes into BRAM (1 dword per cycle, beat buffered).
# - MemRd: returns Completion w/ Data from BRAM (1 dword per cycle, packed into beats).
#
# Bring-up oriented: simple, conservative, and avoids any dynamic slicing / NextValue(None).
#

from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *


class LiteNVMeHostMemResponder(LiteXModule):
    def __init__(self, port, base=0x10000000, size=0x20000, data_width=128, with_csr=True):
        self.port = port
        self.base = base
        self.size = size

        assert data_width in [64, 128, 256]
        beat_bytes  = data_width // 8
        beat_dwords = beat_bytes // 4

        # -----------------------------------------------------------------------------------------
        # BRAM: 32-bit words.
        # -----------------------------------------------------------------------------------------
        depth_dwords = size // 4
        mem = Memory(32, depth_dwords)

        # Dual-port BRAM:
        # - Port A: DMA (NVMe Requests).
        # - Port B: CSR access.
        dma_rp = mem.get_port(has_re=True,  mode=READ_FIRST)
        dma_wp = mem.get_port(write_capable=True)
        csr_rp = mem.get_port(has_re=True,  mode=READ_FIRST)
        csr_wp = mem.get_port(write_capable=True)

        self.specials += mem, dma_rp, dma_wp, csr_rp, csr_wp

        # -----------------------------------------------------------------------------------------
        # CSR access to BRAM (debug).
        # -----------------------------------------------------------------------------------------
        dma_wr_count = Signal(32)
        dma_rd_count = Signal(32)

        if with_csr:
            self._csr_adr      = CSRStorage(32, description="HostMem dword address (index).")
            self._csr_wdata    = CSRStorage(32, description="HostMem write data.")
            self._csr_we       = CSRStorage(1,  description="HostMem write strobe (pulse).")
            self._csr_rdata    = CSRStatus(32,  description="HostMem read data.")
            self._dma_wr_count = CSRStatus(32, description="Count of DMA dwords stored.")
            self._dma_rd_count = CSRStatus(32, description="Count of DMA dwords served.")

            self.comb += [
                csr_rp.adr.eq(self._csr_adr.storage),
                csr_rp.re.eq(1),
                self._csr_rdata.status.eq(csr_rp.dat_r),

                csr_wp.adr.eq(self._csr_adr.storage),
                csr_wp.dat_w.eq(self._csr_wdata.storage),
                csr_wp.we.eq(self._csr_we.storage),

                self._dma_wr_count.status.eq(dma_wr_count),
                self._dma_rd_count.status.eq(dma_rd_count),
            ]

        # -----------------------------------------------------------------------------------------
        # Port field helpers (dat vs data, be optional, req_id/cmp_id optional).
        # -----------------------------------------------------------------------------------------
        req = port.sink   # incoming Requests from NVMe (MemRd/MemWr)
        cmp = port.source # outgoing Completions to NVMe (Cpl/CplD)

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
        has_cmpid = hasattr(cmp, "cmp_id")  # on completion source
        has_end   = hasattr(cmp, "end")

        # NOTE: In LitePCIe, request .adr is a byte address (as used by crossbar/bridges).
        in_window = Signal()
        self.comb += in_window.eq((req.adr >= base) & (req.adr < (base + size)))

        # -----------------------------------------------------------------------------------------
        # Internal latches (captured from request header).
        # -----------------------------------------------------------------------------------------
        lat_tag    = Signal(8)
        lat_len    = Signal(10)   # dwords
        lat_adr    = Signal(64)   # byte address
        lat_req_id = Signal(16)
        lat_cmp_id = Signal(16)   # completer id (if available)

        # Current BRAM dword index for DMA access.
        cur_dw = Signal(32)

        # -----------------------------------------------------------------------------------------
        # Write path: buffer one beat, then write 1 dword/cycle.
        # -----------------------------------------------------------------------------------------
        wr_buf     = Signal(data_width)
        wr_be_buf  = Signal(beat_bytes) if has_be else None
        wr_sub_idx = Signal(max=beat_dwords)
        wr_dw_left = Signal(16)   # remaining dwords in this write request

        # Constant word views of buffered beat (constant slices only).
        wr_buf_words = Array([wr_buf[32*i:32*(i+1)] for i in range(beat_dwords)])

        # -----------------------------------------------------------------------------------------
        # Read path: read 1 dword/cycle into staging words, then emit a beat completion.
        # -----------------------------------------------------------------------------------------
        rd_sub_idx  = Signal(max=beat_dwords)
        rd_dw_left  = Signal(16)  # remaining dwords to serve for this read request
        rd_words    = Array([Signal(32) for _ in range(beat_dwords)])
        rd_buf      = Signal(data_width)
        self.comb += rd_buf.eq(Cat(*rd_words))

        # Whether current completion beat is the *first* beat of this completion packet.
        rd_first = Signal(reset=1)

        # -----------------------------------------------------------------------------------------
        # Defaults.
        # -----------------------------------------------------------------------------------------
        self.comb += [
            req.ready.eq(0),
            cmp.valid.eq(0),

            dma_rp.re.eq(0),
            dma_wp.we.eq(0),
        ]
        if hasattr(cmp, "first"):
            self.comb += cmp.first.eq(0)
        if hasattr(cmp, "last"):
            self.comb += cmp.last.eq(0)
        if has_end:
            self.comb += cmp.end.eq(0)
        if hasattr(cmp, "err"):
            self.comb += cmp.err.eq(0)
        if hasattr(cmp, "len"):
            self.comb += cmp.len.eq(0)
        if hasattr(cmp, "tag"):
            self.comb += cmp.tag.eq(0)
        if hasattr(cmp, "adr"):
            self.comb += cmp.adr.eq(0)
        if has_reqid:
            # for completion source, req_id is usually present; guard anyway
            pass
        if has_cmpid:
            pass

        # -----------------------------------------------------------------------------------------
        # Helpers: BE handling (best-effort).
        # - If BE is per-byte for the beat, accept a dword if any of its 4 bytes are enabled.
        # - If BE is per-dword, use that bit.
        # - Otherwise, write.
        # -----------------------------------------------------------------------------------------
        def be_allows_dword(be_sig, sub_dw):
            if be_sig is None:
                return 1
            if len(be_sig) == beat_bytes:
                b0 = be_sig[sub_dw*4 + 0]
                b1 = be_sig[sub_dw*4 + 1]
                b2 = be_sig[sub_dw*4 + 2]
                b3 = be_sig[sub_dw*4 + 3]
                return b0 | b1 | b2 | b3
            if len(be_sig) == beat_dwords:
                return be_sig[sub_dw]
            return 1

        # -----------------------------------------------------------------------------------------
        # FSM.
        # -----------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")

        # Build safe latch statements for IDLE (avoid NextValue(None, ...)).
        idle_latches = [
            NextValue(lat_adr, req.adr),
            NextValue(cur_dw,  (req.adr - base) >> 2),
        ]
        if has_tag:
            idle_latches.append(NextValue(lat_tag, req.tag))
        else:
            idle_latches.append(NextValue(lat_tag, 0))

        if has_len:
            idle_latches.append(NextValue(lat_len, req.len))
        else:
            idle_latches.append(NextValue(lat_len, 1))

        if has_reqid:
            idle_latches.append(NextValue(lat_req_id, req.req_id))
        else:
            idle_latches.append(NextValue(lat_req_id, 0))

        # Completion cmp_id: some ports expose it, otherwise keep 0.
        if has_cmpid:
            # Many LitePCIe setups want cmp_id = port-side completer_id (Root Port endpoint id).
            # If you have a better constant for this, wire it in from outside.
            idle_latches.append(NextValue(lat_cmp_id, 0))
        else:
            idle_latches.append(NextValue(lat_cmp_id, 0))

        fsm.act("IDLE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                If(~in_window,
                    NextState("DROP")
                ).Else(
                    *idle_latches,
                    If(req.we,
                        # MemWr: payload follows on stream (including this beat).
                        NextValue(wr_dw_left, (req.len if has_len else 1)),
                        NextValue(wr_sub_idx, 0),
                        NextState("WR_CAPTURE")
                    ).Else(
                        # MemRd: serve completion data from BRAM.
                        NextValue(rd_dw_left, (req.len if has_len else 1)),
                        NextValue(rd_sub_idx, 0),
                        NextValue(rd_first, 1),
                        NextState("RD_CLEAR")
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

        # -----------------------------
        # MemWr: capture one beat (data + be if present).
        # -----------------------------
        wr_capture_updates = [NextValue(wr_buf, req_dat), NextValue(wr_sub_idx, 0)]
        if wr_be_buf is not None:
            wr_capture_updates.append(NextValue(wr_be_buf, req.be))

        fsm.act("WR_CAPTURE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                *wr_capture_updates,
                NextState("WR_STORE")
            )
        )

        # -----------------------------
        # MemWr: store buffered beat to BRAM, 1 dword per cycle.
        # -----------------------------
        wr_we_this = Signal()
        wr_word    = Signal(32)

        # Combinational select for current dword within the buffered beat.
        self.comb += [
            wr_we_this.eq(1),
            wr_word.eq(0),
        ]

        # Compute per-dword enable and word via Case (no dynamic slicing).
        wr_cases = {}
        for i in range(beat_dwords):
            be_src = wr_be_buf if (wr_be_buf is not None) else None
            wr_cases[i] = [
                wr_we_this.eq(be_allows_dword(be_src, i)),
                wr_word.eq(wr_buf_words[i]),
            ]
        self.comb += Case(wr_sub_idx, wr_cases)

        fsm.act("WR_STORE",
            # Perform the write if enabled.
            dma_wp.adr.eq(cur_dw),
            dma_wp.dat_w.eq(wr_word),
            dma_wp.we.eq(wr_we_this),

            If(wr_we_this,
                NextValue(dma_wr_count, dma_wr_count + 1)
            ),

            # Advance.
            NextValue(cur_dw, cur_dw + 1),
            NextValue(wr_dw_left, wr_dw_left - 1),

            If(wr_dw_left == 1,
                # Done with request (we have already consumed the beat(s) we needed through WR_CAPTURE).
                NextState("IDLE")
            ).Else(
                # Advance within beat; when beat completes, capture next beat.
                If(wr_sub_idx == (beat_dwords - 1),
                    NextValue(wr_sub_idx, 0),
                    NextState("WR_CAPTURE")
                ).Else(
                    NextValue(wr_sub_idx, wr_sub_idx + 1)
                )
            )
        )

        # -----------------------------
        # MemRd: clear staging words for the next beat.
        # -----------------------------
        fsm.act("RD_CLEAR",
            *[NextValue(rd_words[i], 0) for i in range(beat_dwords)],
            NextValue(rd_sub_idx, 0),
            NextState("RD_READ")
        )

        # -----------------------------
        # MemRd: issue BRAM read for current dword.
        # -----------------------------
        fsm.act("RD_READ",
            dma_rp.adr.eq(cur_dw),
            dma_rp.re.eq(1),
            NextState("RD_LATCH")
        )

        # -----------------------------
        # MemRd: latch BRAM read into rd_words[sub] using Case.
        # -----------------------------
        rd_latch_cases = {i: NextValue(rd_words[i], dma_rp.dat_r) for i in range(beat_dwords)}
        fsm.act("RD_LATCH",
            Case(rd_sub_idx, rd_latch_cases),

            NextValue(dma_rd_count, dma_rd_count + 1),
            NextValue(cur_dw, cur_dw + 1),
            NextValue(rd_dw_left, rd_dw_left - 1),

            # Decide whether to emit this beat now.
            If((rd_dw_left == 1) | (rd_sub_idx == (beat_dwords - 1)),
                NextState("CPLD")
            ).Else(
                NextValue(rd_sub_idx, rd_sub_idx + 1),
                NextState("RD_READ")
            )
        )

        # -----------------------------
        # Completion with Data: emit one beat.
        # We emit a *single completion packet* across one or more beats:
        # - first asserted on first beat only
        # - last asserted on final beat only
        # - len is total dwords for the request (lat_len)
        # - adr is lower address within the request (LitePCIe packetizer can ignore/override)
        # -----------------------------
        fsm.act("CPLD",
            cmp.valid.eq(1),
            cmp_dat.eq(rd_buf),

            # first/last framing across beats.
            If(hasattr(cmp, "first"), cmp.first.eq(rd_first)),
            If(hasattr(cmp, "last"),  cmp.last.eq(rd_dw_left == 0)),  # after decrement in RD_LATCH
            If(has_end,               cmp.end.eq(1)),                 # conservative: single completion segment

            # propagate tag/len/id fields where present.
            If(has_tag,   cmp.tag.eq(lat_tag)),
            If(has_len,   cmp.len.eq(lat_len)),
            If(has_reqid and hasattr(cmp, "req_id"), cmp.req_id.eq(lat_req_id)),
            If(has_cmpid, cmp.cmp_id.eq(lat_cmp_id)),

            If(cmp.valid & cmp.ready,
                NextValue(rd_first, 0),
                If(rd_dw_left == 0,
                    NextState("IDLE")
                ).Else(
                    NextState("RD_CLEAR")
                )
            )
        )
