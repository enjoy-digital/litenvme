#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe block streamer — a LiteSATA-style sector read/write front-end over the I/O engine.

The I/O engine (litenvme/io_engine.py) is addressed: a command carries a host-memory buffer
address (`buf`, the PRP1) and the SSD DMAs data to/from that buffer in the host-memory
window. To give integrators a turnkey "write/read data to the SSD" interface that needs no
knowledge of PRPs or the host-memory layout, this module stages data through a reserved
region of the window:

  - write : the user streams payload into `wr_sink`; the streamer fills the staging region,
            then issues an engine WRITE command pointing at it,
  - read  : the streamer issues an engine READ command into the staging region; once the SSD
            has filled it, the streamer drains it out of `rd_source`.

Interface (control as plain signals — a CSR wrapper is added with `with_csr=True`):

    start, write, sector(64), count(32 sectors), nsid(32)   # command
    done (registered, 1 at idle/complete), error (sticky), busy
    wr_sink   : stream Endpoint [data]   — write payload in
    rd_source : stream Endpoint [data]   — read payload out
    source/sink : the engine-facing request/completion streams (like LiteNVMeRequestGen),
                  connected to engine.sink / engine.source by the integrator.

Data staging reuses the host-memory backend via one extra AXI master (`dma.axi`), added to
the responder's `extra_axi_masters` — no new ports on the responder. v1 runs ONE engine
command per transfer (assert `count*512 <= staging_size`); the engine itself builds the
PRP list internally for transfers spanning >2 pages, so the streamer only needs a
page-aligned, contiguous staging buffer and a once-set `prp_list_base` on the engine.
Multi-chunk (transfers larger than the staging region) is a follow-up — the FSM is written
so it slots in as a loop over chunks.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect import axi
from litex.soc.interconnect.csr import *

from litenvme.io_engine import request_layout, completion_layout

# Host-memory stream DMA ---------------------------------------------------------------------------

class LiteNVMeHostMemStreamDMA(LiteXModule):
    """Beat-wide AXI master that streams a data stream into / out of the flat hostmem backend.

    One command at a time. Drive `we` (1=fill from `sink`, 0=drain to `source`), `adr`
    (window-relative byte address, beat-aligned), `beats`, `last_en` (assert `source.last`
    on the final drained beat), then pulse `start`; wait for `busy` to drop. Single-beat AXI
    transactions (len=0), mirroring LiteNVMeMemPortToAXI.
    """
    def __init__(self, data_width=256, address_width=32):
        self.axi    = axi.AXIInterface(data_width=data_width, address_width=address_width,
                                       id_width=1, mode="rw")
        self.sink   = stream.Endpoint([("data", data_width)])  # fill data in  (host write).
        self.source = stream.Endpoint([("data", data_width)])  # drain data out (host read).

        # Command.
        self.start   = Signal()
        self.we      = Signal()
        self.adr     = Signal(address_width)
        self.beats   = Signal(32)
        self.last_en = Signal()
        self.busy    = Signal()

        # # #

        beat_bytes       = data_width // 8
        beat_bytes_shift = log2_int(beat_bytes)
        axsize           = axi.AXSIZE[beat_bytes]

        cur_adr    = Signal(address_width)
        beats_left = Signal(32)
        last_r     = Signal()
        aw_done    = Signal()
        w_done     = Signal()

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

        fsm.act("IDLE",
            If(self.start,
                NextValue(cur_adr,    self.adr),
                NextValue(beats_left, self.beats),
                NextValue(last_r,     self.last_en),
                NextValue(aw_done, 0),
                NextValue(w_done,  0),
                If(self.we,
                    NextState("FILL"),
                ).Else(
                    NextState("DRAIN-AR"),
                )
            )
        )

        # Fill: one beat per AW+W from `sink`, then B.
        fsm.act("FILL",
            self.axi.aw.valid.eq(~aw_done),
            self.axi.aw.addr.eq(cur_adr),
            self.axi.aw.len.eq(0),
            self.axi.aw.size.eq(axsize),
            self.axi.aw.burst.eq(axi.BURST_INCR),
            self.axi.aw.id.eq(0),

            self.axi.w.valid.eq(~w_done & self.sink.valid),
            self.axi.w.data.eq(self.sink.data),
            self.axi.w.strb.eq(2**beat_bytes - 1),
            self.axi.w.last.eq(1),
            self.sink.ready.eq(self.axi.w.valid & self.axi.w.ready),

            If(self.axi.aw.valid & self.axi.aw.ready, NextValue(aw_done, 1)),
            If(self.axi.w.valid  & self.axi.w.ready,  NextValue(w_done,  1)),
            If((aw_done | (self.axi.aw.valid & self.axi.aw.ready)) &
               (w_done  | (self.axi.w.valid  & self.axi.w.ready)),
                NextState("FILL-B"),
            )
        )
        fsm.act("FILL-B",
            self.axi.b.ready.eq(1),
            If(self.axi.b.valid,
                NextValue(aw_done, 0),
                NextValue(w_done,  0),
                NextValue(cur_adr,    cur_adr + beat_bytes),
                NextValue(beats_left, beats_left - 1),
                If(beats_left == 1,
                    NextState("IDLE"),
                ).Else(
                    NextState("FILL"),
                )
            )
        )

        # Drain: one beat per AR then R to `source`.
        fsm.act("DRAIN-AR",
            self.axi.ar.valid.eq(1),
            self.axi.ar.addr.eq(cur_adr),
            self.axi.ar.len.eq(0),
            self.axi.ar.size.eq(axsize),
            self.axi.ar.burst.eq(axi.BURST_INCR),
            self.axi.ar.id.eq(0),
            If(self.axi.ar.ready,
                NextState("DRAIN-R"),
            )
        )
        fsm.act("DRAIN-R",
            self.source.valid.eq(self.axi.r.valid),
            self.source.data.eq(self.axi.r.data),
            self.source.last.eq(last_r & (beats_left == 1)),
            self.axi.r.ready.eq(self.source.ready),
            If(self.axi.r.valid & self.source.ready,
                NextValue(cur_adr,    cur_adr + beat_bytes),
                NextValue(beats_left, beats_left - 1),
                If(beats_left == 1,
                    NextState("IDLE"),
                ).Else(
                    NextState("DRAIN-AR"),
                )
            )
        )

# LiteNVMe Block Streamer --------------------------------------------------------------------------

class LiteNVMeBlockStreamer(LiteXModule):
    """Sector read/write streaming front-end over LiteNVMeIOEngine (see module docstring)."""
    def __init__(self, data_width=256, hostmem_base=0x1000_0000, staging_base=0x2_0000,
                 staging_size=0x4_0000, lba_bytes=512, with_csr=True):
        # Engine-facing streams (named like LiteNVMeRequestGen for a symmetric front-end mux).
        self.source = stream.Endpoint(request_layout())     # -> engine.sink
        self.sink   = stream.Endpoint(completion_layout())  # <- engine.source

        # User data streams.
        self.wr_sink   = stream.Endpoint([("data", data_width)])
        self.rd_source = stream.Endpoint([("data", data_width)])

        # Control / status.
        self.start  = Signal()
        self.write  = Signal()
        self.sector = Signal(64)
        self.count  = Signal(32)
        self.nsid   = Signal(32, reset=1)
        self.done   = Signal()
        self.error  = Signal()
        self.busy   = Signal()

        self.staging_base = staging_base
        self.staging_size = staging_size

        # # #

        beat_bytes       = data_width // 8
        beat_bytes_shift = log2_int(beat_bytes)
        lba_shift        = log2_int(lba_bytes)
        # sectors -> beats: count * lba_bytes / beat_bytes (lba_bytes >= beat_bytes).
        sec_to_beats_sh  = lba_shift - beat_bytes_shift

        # Staging DMA (joins the hostmem AXI arbiter via .axi).
        self.dma = dma = LiteNVMeHostMemStreamDMA(data_width=data_width, address_width=32)
        self.comb += [
            self.wr_sink.connect(dma.sink),      # user write payload -> staging.
            dma.source.connect(self.rd_source),  # staging -> user read payload.
        ]

        # Latched command.
        w_write  = Signal()
        w_sector = Signal(64)
        w_count  = Signal(32)
        w_nsid   = Signal(32)
        nbeats   = Signal(32)
        buf_adr  = Signal(64)

        # Registered done (request_gen idiom: avoids the start-pulse vs CSR-read race).
        done_r = Signal(reset=1)
        self.comb += self.done.eq(done_r)

        # Completion status decode.
        cpl_sc  = Signal(8)
        cpl_sct = Signal(3)
        self.comb += [
            cpl_sc.eq(self.sink.status[1:9]),
            cpl_sct.eq(self.sink.status[9:12]),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

        fsm.act("IDLE",
            If(self.start,
                NextValue(w_write,  self.write),
                NextValue(w_sector, self.sector),
                NextValue(w_count,  self.count),
                NextValue(w_nsid,   self.nsid),
                NextValue(nbeats,   self.count << sec_to_beats_sh),
                NextValue(buf_adr,  hostmem_base + staging_base),
                NextValue(self.error, 0),
                NextValue(done_r,     0),
                If(self.write,
                    NextState("WR-FILL"),
                ).Else(
                    NextState("RD-CMD"),
                )
            )
        )

        # Write path: fill staging from wr_sink -> issue engine WRITE -> wait completion.
        fsm.act("WR-FILL",
            dma.we.eq(1),
            dma.adr.eq(staging_base),
            dma.beats.eq(nbeats),
            dma.start.eq(1),
            NextState("WR-FILL-WAIT"),
        )
        fsm.act("WR-FILL-WAIT",
            If(~dma.busy, NextState("WR-CMD")),
        )
        fsm.act("WR-CMD",
            self.source.valid.eq(1),
            self.source.op.eq(1),  # write.
            self.source.nsid.eq(w_nsid),
            self.source.lba.eq(w_sector),
            self.source.nlb.eq(w_count),
            self.source.buf.eq(buf_adr),
            If(self.source.ready, NextState("WR-WAIT")),
        )
        fsm.act("WR-WAIT",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                If((cpl_sc != 0) | (cpl_sct != 0), NextValue(self.error, 1)),
                NextState("DONE"),
            )
        )

        # Read path: issue engine READ -> wait completion -> drain staging to rd_source.
        fsm.act("RD-CMD",
            self.source.valid.eq(1),
            self.source.op.eq(0),  # read.
            self.source.nsid.eq(w_nsid),
            self.source.lba.eq(w_sector),
            self.source.nlb.eq(w_count),
            self.source.buf.eq(buf_adr),
            If(self.source.ready, NextState("RD-WAIT")),
        )
        fsm.act("RD-WAIT",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                If((cpl_sc != 0) | (cpl_sct != 0), NextValue(self.error, 1)),
                NextState("RD-DRAIN"),
            )
        )
        fsm.act("RD-DRAIN",
            dma.we.eq(0),
            dma.adr.eq(staging_base),
            dma.beats.eq(nbeats),
            dma.last_en.eq(1),
            dma.start.eq(1),
            NextState("RD-DRAIN-WAIT"),
        )
        fsm.act("RD-DRAIN-WAIT",
            If(~dma.busy, NextState("DONE")),
        )

        fsm.act("DONE",
            NextValue(done_r, 1),
            NextState("IDLE"),
        )

        if with_csr:
            self.add_csr()

    # CSRs ---------------------------------------------------------------------------------------
    def add_csr(self):
        self._sector_lo = CSRStorage(32, description="Start LBA (sector) low.")
        self._sector_hi = CSRStorage(32, description="Start LBA (sector) high.")
        self._count     = CSRStorage(32, reset=1, description="Number of 512B sectors to transfer.")
        self._nsid      = CSRStorage(32, reset=1, description="Namespace ID.")
        self._ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start the transfer."),
            CSRField("write", size=1, offset=1,             description="1=write (host->SSD), 0=read."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("done",  size=1, offset=0, description="Transfer complete / idle."),
            CSRField("busy",  size=1, offset=1, description="Transfer in progress."),
            CSRField("error", size=1, offset=2, description="A completion returned an error status."),
        ])
        self._cycles = CSRStatus(32, description="Cycles from start to done (for host MB/s).")

        cycles  = Signal(32)
        running = Signal()
        self.sync += [
            If(self.start,
                running.eq(1),
                cycles.eq(0),
            ).Elif(running,
                cycles.eq(cycles + 1),
                If(self.done, running.eq(0)),
            ),
        ]

        self.comb += [
            self.sector.eq(Cat(self._sector_lo.storage, self._sector_hi.storage)),
            self.count.eq(self._count.storage),
            self.nsid.eq(self._nsid.storage),
            self.start.eq(self._ctrl.fields.start),
            self.write.eq(self._ctrl.fields.write),

            self._status.fields.done.eq(self.done),
            self._status.fields.busy.eq(self.busy),
            self._status.fields.error.eq(self.error),
            self._cycles.status.eq(cycles),
        ]
