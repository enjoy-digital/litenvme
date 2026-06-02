#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litenvme.mem import _LiteNVMePCIeAccessor

# LiteNVMe PCIe CFG Accessor -----------------------------------------------------------------------

class LiteNVMePCIeCfgAccessor(_LiteNVMePCIeAccessor):
    """PCIe CFG0 Read/Write accessor (CFG-over-REQUEST).

    Issues Config Space accesses through a LitePCIe crossbar master port. Expects the master port
    REQUEST layout to include the CONFIGURATION extensions (is_cfg, bus_number, device_no, func,
    ext_reg, register_no). Single outstanding transaction; reads and writes both wait for the
    completion (CFG writes are non-posted).
    """
    def __init__(self, port, requester_id=0x0000, tag=0x42, timeout=2**20, with_csr=False):
        self.requester_id = requester_id
        self.bus      = Signal(8)
        self.device   = Signal(5)
        self.function = Signal(3)
        self.reg      = Signal(6)   # DWORD index (offset/4).
        self.ext_reg  = Signal(3)   # Extended register.
        super().__init__(port, tag=tag, timeout=timeout, with_csr=with_csr)

    def send_request(self, req_sink):
        return [
            req_sink.req_id.eq(self.requester_id),
            req_sink.adr.eq(0),  # Unused for CFG but must be driven.
            req_sink.len.eq(1),  # CFG is 1DW.

            # CFG-over-REQUEST fields.
            req_sink.is_cfg.eq(1),
            req_sink.bus_number.eq(self.bus),
            req_sink.device_no.eq(self.device),
            req_sink.func.eq(self.function),
            req_sink.ext_reg.eq(self.ext_reg),
            req_sink.register_no.eq(self.reg),
        ]

    def add_csr(self):
        # cfg_ctrl:
        # - start: Pulse to launch a new CFG transaction.
        # - we:    0=Read, 1=Write.
        self._cfg_ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, values=[
                ("``1``", "Start a new CFG transaction."),
            ]),
            CSRField("we",    size=1, offset=1, values=[
                ("``0``", "CFG Read."),
                ("``1``", "CFG Write."),
            ]),
        ])

        # cfg_bdf:
        # - bus/dev/fn select the target function.
        # - reg/ext select the DWORD register (offset = (ext<<8 | reg) * 4).
        self._cfg_bdf = CSRStorage(fields=[
            CSRField("bus", size=8, offset=0,  description="Bus number."),
            CSRField("dev", size=5, offset=8,  description="Device number."),
            CSRField("fn",  size=3, offset=13, description="Function number."),
            CSRField("reg", size=6, offset=16, description="DWORD register number (offset/4)."),
            CSRField("ext", size=3, offset=22, description="Extended register number."),
        ])

        # cfg_wdata:
        # - DWORD write payload for CFG writes.
        self._cfg_wdata = CSRStorage(32, description="CFG write data (DWORD).")

        # cfg_stat:
        # - done: Set when the transaction completed (or timed out).
        # - err:  Completion error or timeout.
        self._cfg_stat = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="Transaction done."),
            CSRField("err",  size=1, offset=1, description="Transaction error (completion error/timeout)."),
        ])

        # cfg_rdata:
        # - DWORD read payload for CFG reads (valid when done=1 and err=0).
        self._cfg_rdata = CSRStatus(32, description="CFG read data (DWORD).")

        # Connections.
        self.comb += [
           # Control.
           self.start.eq(self._cfg_ctrl.fields.start),
           self.we.eq(self._cfg_ctrl.fields.we),

           # Addressing.
           self.bus.eq(self._cfg_bdf.fields.bus),
           self.device.eq(self._cfg_bdf.fields.dev),
           self.function.eq(self._cfg_bdf.fields.fn),
           self.reg.eq(self._cfg_bdf.fields.reg),
           self.ext_reg.eq(self._cfg_bdf.fields.ext),

           # Write data.
           self.wdata.eq(self._cfg_wdata.storage),

           # Status / Readback.
           self._cfg_stat.fields.done.eq(self.done),
           self._cfg_stat.fields.err.eq(self.err),
           self._cfg_rdata.status.eq(self.rdata),
        ]

# LiteNVMe Root-Port CFG Management ----------------------------------------------------------------

class LiteNVMeRootCfgMgmt(LiteXModule):
    """Drive the USPPCIEPHY Configuration Management interface from sys-domain CSRs.

    Unlike the CFG accessor above (which issues CFG TLPs to the *downstream* endpoint), this
    reaches the Root Port's OWN config space via the hard IP's local cfg_mgmt port. It exists to
    let firmware program the root port's DevCtl.MPS (raise MaxPayloadSize), which no enumeration
    software does in this embedded design. The cfg_mgmt port lives in the "pcie" clock domain;
    this bridges it to sys-domain CSRs with a start/done handshake.

    Requires the phy built with `with_cfg_mgmt=True` (exposes phy.cfg_mgmt_*). Sys->pcie buses
    (addr/wdata/be/func/write) are quasi-static during a transaction and are covered by the
    existing sys<->pcie false-path constraint.
    """
    def __init__(self, phy):
        self._ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start a cfg_mgmt transaction."),
            CSRField("write", size=1, offset=1, description="0=Read, 1=Write."),
        ])
        self._addr  = CSRStorage(10, description="DWORD address into Root Port config space.")
        self._func  = CSRStorage(8,  description="Function number (0 for PF0).")
        self._wdata = CSRStorage(32, description="Write data (DWORD).")
        self._be    = CSRStorage(4,  reset=0b1111, description="Byte enables.")
        self._stat  = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="Last transaction complete."),
            CSRField("busy", size=1, offset=1, description="Transaction in progress."),
        ])
        self._rdata = CSRStatus(32, description="Read data (valid when done=1).")

        # # #

        # Handshake CDC.
        self.start_ps = start_ps = PulseSynchronizer("sys", "pcie")
        self.done_ps  = done_ps  = PulseSynchronizer("pcie", "sys")

        # Busy / done status (sys).
        busy = Signal()
        done = Signal()
        self.sync += [
            If(self._ctrl.fields.start, busy.eq(1), done.eq(0)),
            If(done_ps.o,               busy.eq(0), done.eq(1)),
        ]
        self.comb += [
            start_ps.i.eq(self._ctrl.fields.start),
            self._stat.fields.busy.eq(busy),
            self._stat.fields.done.eq(done),
        ]

        # Read data (stable after done) -> sys.
        rdata_pcie = Signal(32)
        self.specials += MultiReg(rdata_pcie, self._rdata.status)

        # Drive the cfg_mgmt buses directly from sys-domain storage (quasi-static, false-pathed).
        cfg_read  = Signal()
        cfg_write = Signal()
        self.comb += [
            phy.cfg_mgmt_addr.eq(self._addr.storage),
            phy.cfg_mgmt_function_number.eq(self._func.storage),
            phy.cfg_mgmt_write_data.eq(self._wdata.storage),
            phy.cfg_mgmt_byte_enable.eq(self._be.storage),
            phy.cfg_mgmt_read.eq(cfg_read),
            phy.cfg_mgmt_write.eq(cfg_write),
        ]

        # Transaction FSM (pcie domain): assert read/write, hold until done, latch read data.
        write_lat = Signal()
        self.fsm = fsm = ClockDomainsRenamer("pcie")(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            If(start_ps.o,
                NextValue(write_lat, self._ctrl.fields.write),
                NextState("ISSUE"),
            )
        )
        fsm.act("ISSUE",
            cfg_read.eq(~write_lat),
            cfg_write.eq(write_lat),
            If(phy.cfg_mgmt_read_write_done,
                NextValue(rdata_pcie, phy.cfg_mgmt_read_data),
                NextState("DONE"),
            )
        )
        fsm.act("DONE",
            done_ps.i.eq(1),
            NextState("IDLE"),
        )
