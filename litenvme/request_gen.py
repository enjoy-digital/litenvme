#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe request generator — drive the I/O engine at full rate for benchmarking.

`LiteNVMeIOEngine` accepts requests on a stream `sink` and emits completions on a stream
`source`, but nothing drives those streams on hardware (only the simulation tests do). To
*measure* the engine's true throughput we must feed requests as fast as the engine can
take them, with **no CPU in the steady-state loop** — a per-command CPU push would just
re-create the firmware submission bottleneck we already measured.

`LiteNVMeRequestGen` is a small CSR-configured hardware generator that:
  - emits `count` sequential requests into `source` (-> engine.sink): for command k,
        op   = op
        nsid = nsid
        lba  = base_lba + k * lba_step
        nlb  = nlb
        buf  = buf_base + (k % qmod) * buf_stride     # distinct per-slot PRP1 buffers
  - consumes completions from `sink` (<- engine.source): counts them, OR-accumulates any
    error status (SC/SCT != 0), and frees the implicit window (the engine's own `inflight`
    bounds outstanding commands, so the generator just needs back-pressure from
    engine.sink.ready — it never overruns the engine).
  - measures `cycles` from `start` until the last completion arrives, so firmware can
    compute MB/s = payload * clk / cycles with the CPU completely out of the hot loop.

`qmod` mirrors the engine's `qsize`: the engine assigns `cid = sq_tail` and reuses SQ slot
/ buffer i after `qsize` submissions, so keying the buffer by `k % qmod` (with qmod=qsize)
gives each in-flight command a distinct buffer, exactly as the firmware QD bench does.

Interfaces:
- `source` : stream Endpoint(request_layout())    -> connect to engine.sink
- `sink`   : stream Endpoint(completion_layout())  <- connect from engine.source
CSR config: op, nsid, base_lba, nlb, lba_step, count, buf_base, buf_stride, qmod, ctrl.start
CSR status: done, completed, errors, cycles, cqe_status (last)
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litenvme.io_engine import request_layout, completion_layout, NVME_OP_READ, NVME_OP_WRITE

# LiteNVMe Request Generator -----------------------------------------------------------------------

class LiteNVMeRequestGen(LiteXModule):
    """CSR-driven sequential request generator + completion counter for the I/O engine."""
    def __init__(self, with_csr=True):
        # Streams to/from the engine.
        self.source = stream.Endpoint(request_layout())     # -> engine.sink
        self.sink   = stream.Endpoint(completion_layout())  # <- engine.source

        # Config (set by CSR or directly).
        self.op         = Signal()       # 0 = read, 1 = write.
        self.nsid       = Signal(32)
        self.base_lba   = Signal(64)
        self.nlb        = Signal(16, reset=1)
        self.lba_step   = Signal(32)
        self.count      = Signal(32, reset=1)
        self.buf_base   = Signal(64)
        self.buf_stride = Signal(32, reset=0x1000)
        self.qmod       = Signal(16, reset=64)
        self.start      = Signal()       # pulse to begin.

        # Status.
        self.done       = Signal()
        self.completed  = Signal(32)
        self.errors     = Signal(32)
        self.cycles     = Signal(32)
        self.last_status = Signal(16)
        # First-error capture (for debugging errors>0): the status, cid and completion index
        # of the FIRST CQE seen with a non-success status. Lets firmware tell a real device
        # error (a valid NVMe SC/SCT) from a harness/reap artifact, and which command erred.
        self.first_err_status = Signal(16)
        self.first_err_cid    = Signal(16)
        self.first_err_idx    = Signal(32)

        # # #

        submitted = Signal(32)
        running   = Signal()
        lba       = Signal(64)
        buf_key   = Signal(16)   # k % qmod.

        # Registered done flag: 1 at reset/idle, cleared the cycle a run starts, set when
        # it finishes. Robust against the 1-cycle start-pulse vs CSR-read-latency hazard
        # (a plain ongoing(IDLE) could read stale-1 in the start cycle).
        done_r = Signal(reset=1)
        self.comb += self.done.eq(done_r)

        # Combinational request payload for the current command.
        self.comb += [
            self.source.op.eq(self.op),
            self.source.nsid.eq(self.nsid),
            self.source.lba.eq(lba),
            self.source.nlb.eq(self.nlb),
            self.source.buf.eq(self.buf_base + buf_key * self.buf_stride),
        ]

        # Completion intake: always ready while running; tally completions + errors.
        cpl_fire = Signal()
        self.comb += [
            self.sink.ready.eq(running),
            cpl_fire.eq(self.sink.valid & self.sink.ready),
        ]
        cpl_sc  = Signal(8)
        cpl_sct = Signal(3)
        self.comb += [
            cpl_sc.eq(self.sink.status[1:9]),
            cpl_sct.eq(self.sink.status[9:12]),
        ]

        # Submit handshake.
        sub_fire = Signal()
        self.comb += sub_fire.eq(self.source.valid & self.source.ready)

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.start,
                NextValue(submitted,      0),
                NextValue(self.completed, 0),
                NextValue(self.errors,    0),
                NextValue(self.first_err_status, 0),
                NextValue(self.first_err_cid,    0),
                NextValue(self.first_err_idx,    0),
                NextValue(self.cycles,    0),
                NextValue(lba,            self.base_lba),
                NextValue(buf_key,        0),
                NextValue(running,        1),
                NextValue(done_r,         0),
                NextState("RUN"),
            )
        )

        fsm.act("RUN",
            # Count cycles for the whole measured window.
            NextValue(self.cycles, self.cycles + 1),

            # Offer a request while there are still some to submit.
            self.source.valid.eq(submitted < self.count),

            # On submit handshake, advance lba / buffer key / submitted.
            If(sub_fire,
                NextValue(submitted, submitted + 1),
                NextValue(lba, lba + self.lba_step),
                If(buf_key == (self.qmod - 1),
                    NextValue(buf_key, 0),
                ).Else(
                    NextValue(buf_key, buf_key + 1),
                ),
            ),

            # On completion, tally + accumulate errors.
            If(cpl_fire,
                NextValue(self.completed, self.completed + 1),
                NextValue(self.last_status, self.sink.status),
                If((cpl_sc != 0) | (cpl_sct != 0),
                    NextValue(self.errors, self.errors + 1),
                    # Latch the first error's details only (errors still 0 this cycle).
                    If(self.errors == 0,
                        NextValue(self.first_err_status, self.sink.status),
                        NextValue(self.first_err_cid,    self.sink.cid),
                        NextValue(self.first_err_idx,    self.completed),
                    ),
                ),
            ),

            # Done when every submitted command has completed.
            If((self.completed + cpl_fire) == self.count,
                NextValue(running, 0),
                NextValue(done_r,  1),
                NextState("IDLE"),
            )
        )

        if with_csr:
            self.add_csr()

    def add_csr(self):
        self._op         = CSRStorage(1,  description="0=read, 1=write.")
        self._nsid       = CSRStorage(32, description="Namespace ID.")
        self._base_lba_lo = CSRStorage(32, description="Base LBA low.")
        self._base_lba_hi = CSRStorage(32, description="Base LBA high.")
        self._nlb        = CSRStorage(16, reset=1,      description="Blocks per command (1-based).")
        self._lba_step   = CSRStorage(32, description="LBA increment between commands.")
        self._count      = CSRStorage(32, reset=1,      description="Number of commands to issue.")
        self._buf_base_lo = CSRStorage(32, description="PRP1 buffer base (host byte addr) low.")
        self._buf_base_hi = CSRStorage(32, description="PRP1 buffer base (host byte addr) high.")
        self._buf_stride = CSRStorage(32, reset=0x1000, description="Per-slot buffer stride (bytes).")
        self._qmod       = CSRStorage(16, reset=64,     description="Buffer-key modulus (== engine qsize).")
        self._ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start the run."),
        ])

        self._status = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="Run complete / idle."),
        ])
        self._completed  = CSRStatus(32, description="Completions counted.")
        self._errors     = CSRStatus(32, description="Completions with error status.")
        self._cycles     = CSRStatus(32, description="Cycles from start to last completion.")
        self._last_status = CSRStatus(16, description="Last CQE status field.")
        self._first_err_status = CSRStatus(16, description="Status of the first erroring CQE.")
        self._first_err_cid    = CSRStatus(16, description="CID of the first erroring CQE.")
        self._first_err_idx    = CSRStatus(32, description="Completion index of the first error.")

        self.comb += [
            self.op.eq(self._op.storage),
            self.nsid.eq(self._nsid.storage),
            self.base_lba.eq(Cat(self._base_lba_lo.storage, self._base_lba_hi.storage)),
            self.nlb.eq(self._nlb.storage),
            self.lba_step.eq(self._lba_step.storage),
            self.count.eq(self._count.storage),
            self.buf_base.eq(Cat(self._buf_base_lo.storage, self._buf_base_hi.storage)),
            self.buf_stride.eq(self._buf_stride.storage),
            self.qmod.eq(self._qmod.storage),
            self.start.eq(self._ctrl.fields.start),

            self._status.fields.done.eq(self.done),
            self._completed.status.eq(self.completed),
            self._errors.status.eq(self.errors),
            self._cycles.status.eq(self.cycles),
            self._last_status.status.eq(self.last_status),
            self._first_err_status.status.eq(self.first_err_status),
            self._first_err_cid.status.eq(self.first_err_cid),
            self._first_err_idx.status.eq(self.first_err_idx),
        ]
