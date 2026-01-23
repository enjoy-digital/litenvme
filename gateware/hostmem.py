#
# BRAM-backed host memory target for NVMe (RootPort mode).
#
# - Sits on a LitePCIe crossbar SLAVE port (i.e. receives Requests from NVMe).
# - Implements a small memory window (base/size).
# - MemWr: writes into BRAM (1 dword per cycle).
# - MemRd: returns Completion w/ Data from BRAM (1 dword per cycle, packed into beats).
#
# Intentionally simple / bring-up oriented.
#

from migen import *
from litex.gen import *

from litex.soc.interconnect.csr import *

# LiteNVMeHostMemResponder -------------------------------------------------------------------------

class LiteNVMeHostMemResponder(LiteXModule):
    def __init__(self, port, base=0x10000000, size=0x20000, data_width=128, with_csr=True):
        self.port = port
        self.base = base
        self.size = size

        assert data_width in [64, 128, 256]
        beat_bytes  = data_width // 8
        beat_dwords = beat_bytes // 4

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
            self._csr_adr   = CSRStorage(32, description="HostMem dword address (index).")
            self._csr_wdata = CSRStorage(32, description="HostMem write data.")
            self._csr_we    = CSRStorage(1,  description="HostMem write strobe (pulse).")
            self._csr_rdata = CSRStatus(32,  description="HostMem read data.")
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
        # Port field helpers (dat vs data, be optional).
        # -----------------------------------------------------------------------------------------
        req = port.sink
        cmp = port.source

        def get_dat(ep):
            if hasattr(ep, "dat"):  return ep.dat
            if hasattr(ep, "data"): return ep.data
            raise ValueError("Endpoint has no dat/data field.")

        req_dat = get_dat(req)
        cmp_dat = get_dat(cmp)

        req_be = None
        if hasattr(req, "be"):
            req_be = req.be

        # -----------------------------------------------------------------------------------------
        # Decode window.
        # -----------------------------------------------------------------------------------------
        in_window = Signal()
        self.comb += in_window.eq((req.adr >= base) & (req.adr < (base + size)))

        # -----------------------------------------------------------------------------------------
        # Internal latches.
        # -----------------------------------------------------------------------------------------
        lat_tag = Signal(8)
        lat_len = Signal(10)   # dwords (LitePCIe usually uses dwords for len)
        lat_adr = Signal(64)   # byte address

        # Current BRAM dword index.
        cur_dw  = Signal(32)

        # Write path latches.
        wr_buf     = Signal(data_width)  # one beat captured from req_dat
        wr_sub_idx = Signal(max=beat_dwords)
        wr_dw_left = Signal(16)

        # Read path latches.
        rd_sub_idx = Signal(max=beat_dwords)
        rd_dw_left = Signal(16)
        rd_buf_w   = Array(Signal(32) for _ in range(beat_dwords))
        rd_buf     = Signal(data_width)

        # Assemble completion data from words.
        self.comb += rd_buf.eq(Cat(*rd_buf_w))

        # Defaults.
        self.comb += [
            req.ready.eq(0),
            cmp.valid.eq(0),
            dma_rp.re.eq(0),
            dma_wp.we.eq(0),
        ]
        if hasattr(cmp, "first"): self.comb += cmp.first.eq(0)
        if hasattr(cmp, "last"):  self.comb += cmp.last.eq(0)
        if hasattr(cmp, "end"):   self.comb += cmp.end.eq(0)
        if hasattr(cmp, "err"):   self.comb += cmp.err.eq(0)
        if hasattr(cmp, "len"):   self.comb += cmp.len.eq(0)
        if hasattr(cmp, "tag"):   self.comb += cmp.tag.eq(0)

        # -----------------------------------------------------------------------------------------
        # Helpers: BE handling (best-effort).
        # We only do "write the whole dword if any byte enabled in that dword".
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

        # Pre-split write beat into constant slices.
        wr_words = Array(req_dat[i*32:(i+1)*32] for i in range(beat_dwords))
        wr_be_dw = []
        for i in range(beat_dwords):
            wr_be_dw.append(be_allows_dword(req_be, i))

        # -----------------------------------------------------------------------------------------
        # FSM.
        # -----------------------------------------------------------------------------------------
        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        fsm.act("IDLE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                If(~in_window,
                    NextState("DROP")
                ).Else(
                    NextValue(lat_adr, req.adr),
                    NextValue(cur_dw,  (req.adr - base) >> 2),

                    If(hasattr(req, "tag"), NextValue(lat_tag, req.tag)).Else(NextValue(lat_tag, 0)),
                    If(hasattr(req, "len"), NextValue(lat_len, req.len)).Else(NextValue(lat_len, 1)),

                    If(req.we,
                        # MemWr.
                        NextValue(wr_dw_left,  Mux(hasattr(req, "len"), req.len, 1)),
                        NextValue(wr_sub_idx,  0),
                        NextState("WR_CAPTURE")
                    ).Else(
                        # MemRd.
                        NextValue(rd_dw_left,  Mux(hasattr(req, "len"), req.len, 1)),
                        NextValue(rd_sub_idx,  0),
                        NextState("RD_CLEARBUF")
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
        # MemWr: capture one beat.
        # -----------------------------
        fsm.act("WR_CAPTURE",
            req.ready.eq(1),
            If(req.valid & req.ready,
                NextValue(wr_buf, req_dat),
                NextValue(wr_sub_idx, 0),
                NextState("WR_STORE")
            )
        )

        # -----------------------------
        # MemWr: store captured beat to BRAM, 1 dword per cycle.
        # -----------------------------
        fsm.act("WR_STORE",
            # Select the current dword from wr_buf using a Case (no dynamic slices).
            Case(wr_sub_idx, {
                i: [
                    dma_wp.adr.eq(cur_dw),
                    dma_wp.dat_w.eq(wr_buf[i*32:(i+1)*32]),
                    dma_wp.we.eq(wr_be_dw[i]),
                ]
                for i in range(beat_dwords)
            }),

            If(dma_wp.we,
                NextValue(dma_wr_count, dma_wr_count + 1)
            ),

            NextValue(cur_dw,     cur_dw + 1),
            NextValue(wr_dw_left, wr_dw_left - 1),

            If(wr_dw_left == 1,
                # Done with this request. We rely on req.last having been consumed in WR_CAPTURE.
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
        # MemRd: clear rd_buf_w before filling.
        # -----------------------------
        fsm.act("RD_CLEARBUF",
            # clear all words for safety (constant ops)
            *[NextValue(rd_buf_w[i], 0) for i in range(beat_dwords)],
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
        # MemRd: latch BRAM read into rd_buf_w[sub] using Case.
        # -----------------------------
        fsm.act("RD_LATCH",
            Case(rd_sub_idx, {
                i: NextValue(rd_buf_w[i], dma_rp.dat_r)
                for i in range(beat_dwords)
            }),

            NextValue(dma_rd_count, dma_rd_count + 1),
            NextValue(cur_dw,     cur_dw + 1),
            NextValue(rd_dw_left, rd_dw_left - 1),

            If(rd_dw_left == 1,
                # last dword -> send partial beat
                NextState("CPLD")
            ).Else(
                If(rd_sub_idx == (beat_dwords - 1),
                    NextState("CPLD")
                ).Else(
                    NextValue(rd_sub_idx, rd_sub_idx + 1),
                    NextState("RD_READ")
                )
            )
        )

        # -----------------------------
        # Completion with Data: send one beat.
        # -----------------------------
        fsm.act("CPLD",
            cmp.valid.eq(1),
            cmp_dat.eq(rd_buf),

            If(hasattr(cmp, "first"), cmp.first.eq(1)),
            If(hasattr(cmp, "last"),  cmp.last.eq(1)),
            If(hasattr(cmp, "end"),   cmp.end.eq(1)),
            If(hasattr(cmp, "tag"),   cmp.tag.eq(lat_tag)),
            If(hasattr(cmp, "len"),   cmp.len.eq(lat_len)),
            If(hasattr(cmp, "err"),   cmp.err.eq(0)),

            If(cmp.valid & cmp.ready,
                # Prepare next beat if remaining.
                *[NextValue(rd_buf_w[i], 0) for i in range(beat_dwords)],
                NextValue(rd_sub_idx, 0),
                If(rd_dw_left == 0,
                    NextState("IDLE")
                ).Else(
                    NextState("RD_READ")
                )
            )
        )
