#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Developed with LLM assistance.
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe top-level core — drop-in NVMe block access for a LiteX/LitePCIe SoC.

This packages the validated building blocks into one integratable core:

    LiteNVMe(pcie_endpoint, ...)
      ├── cfg       : LiteNVMePCIeCfgAccessor   (CSR)  — PCIe config space (BAR0, MEM/BME)
      ├── mmio      : LiteNVMePCIeMmioAccessor   (CSR)  — BAR0 MMIO for controller setup
      ├── mmio_db   : LiteNVMePCIeMmioAccessor   (—)    — dedicated doorbell path for the engine
      ├── hostmem   : LiteNVMeHostMemResponder          — host-memory window (SSD DMA target)
      └── io_engine : LiteNVMeIOEngineAXI        (CSR)  — hardware I/O command engine (QD)
                         .axi joins the hostmem AXI backend via the responder's arbiter
                         .connect_mmio(mmio_db) drives SQ/CQ doorbells

Interfaces (the requested "standardized interfaces"):

- Config / control: CSR (auto AXI-Lite in LiteX). The cfg/mmio accessors and the engine
  config (sq_base/cq_base/doorbell addresses, enable) and status (busy/inflight/
  submitted/completed) are CSR-exposed.
- Request / completion: stream Endpoints on the engine (`request` sink:
  {op,nsid,lba,nlb,buf}; `completion` source: {cid,status,sqhd}). User logic (or a CSR
  shim, or a DMA front-end) drives requests and consumes completions.
- Bulk data: addressed host memory (the hostmem window). NVMe is the DMA master, so the
  data path is *addressed* (PRP buffers), not a free-running stream — see
  doc/THROUGHPUT_DESIGN.md for the AXI-MM vs AXI-Stream rationale. The hostmem window is
  the canonical data interface; an optional AXI-Stream adapter can be layered on top for
  sequential streaming use-cases.

Bring-up model: a host/firmware does one-time admin setup (enable controller, create the
IO SQ/CQ) via the cfg/mmio CSRs, programs the engine's queue/doorbell addresses, then
sets `io_engine.enable`. Steady-state I/O is then hardware-driven by the engine with up
to `qd` commands outstanding — no per-command CPU cost.

Validated in simulation end to end (test/test_io_engine_integration.py exercises the
engine + bridge + arbiter + real backend). SoC wiring example: bench/alibaba_xcku3p.py.
"""

from migen import *

from litex.gen import *

from litenvme.cfg       import LiteNVMePCIeCfgAccessor
from litenvme.mem       import LiteNVMePCIeMmioAccessor
from litenvme.hostmem   import LiteNVMeHostMemResponder
from litenvme.io_engine import LiteNVMeIOEngineAXI

# LiteNVMe Core ------------------------------------------------------------------------------------

class LiteNVMe(LiteXModule):
    """Drop-in NVMe core for a LiteX/LitePCIe RootPort SoC.

    Parameters
    ----------
    pcie_endpoint : LitePCIeRootPort
        The PCIe RootPort endpoint; its crossbar provides the master/slave ports.
    hostmem_base, hostmem_size : int
        Host-memory window the SSD DMAs to/from (queues + PRP data buffers).
    data_width : int
        PCIe / backend data width (64/128/256). Defaults to the PHY's data width.
    qid, qsize, qd : int
        IO queue id, ring size, and max commands outstanding for the engine.
    requester_id : int
        PCIe requester id for the accessors.

    Notable attributes
    ------------------
    request    : stream Endpoint sink   — submit {op,nsid,lba,nlb,buf} descriptors.
    completion : stream Endpoint source — receive {cid,status,sqhd} descriptors.
    cfg, mmio, mmio_db, hostmem, io_engine : the sub-blocks (for CSR access / wiring).
    """
    def __init__(self, pcie_endpoint, hostmem_base=0x1000_0000, hostmem_size=0x8_0000,
                 data_width=None, qid=1, qsize=64, qd=32, requester_id=0x0000,
                 hostmem_backend=None, with_request_gen=False):
        # hostmem_backend: optional pre-built host-memory backend exposing a matching
        # `.axi` slave. Default (None) instantiates on-FPGA BRAM. Pass a LiteDRAM AXI
        # port wrapper here for a DDR-backed host-memory window with large buffers
        # (sustained high-QD streaming). See doc/THROUGHPUT_DESIGN.md section 5.
        #
        # with_request_gen: instantiate a hardware request generator (LiteNVMeRequestGen)
        # bound to the engine's request/completion streams, so the core can self-drive at
        # full rate for benchmarking (no CPU in the steady-state loop). The user request
        # stream is then owned by the generator (see self.request_gen); leave False to
        # drive `self.request`/`self.completion` from your own logic.
        self.pcie_endpoint = pcie_endpoint
        endpoint = pcie_endpoint

        if data_width is None:
            data_width = endpoint.phy.data_width

        # Master ports off the PCIe crossbar.
        cfg_port     = endpoint.crossbar.get_master_port()
        mmio_port    = endpoint.crossbar.get_master_port()
        mmio_db_port = endpoint.crossbar.get_master_port()

        # PCIe config-space accessor (BAR0 discovery, Command.MEM/BME) — CSR-driven setup.
        self.cfg = LiteNVMePCIeCfgAccessor(
            port         = cfg_port,
            requester_id = requester_id,
            tag          = 0x42,
            with_csr     = True,
        )

        # BAR0 MMIO accessor for controller setup (CAP/CC/CSTS/AQA/queue create) — CSR.
        self.mmio = LiteNVMePCIeMmioAccessor(
            port     = mmio_port,
            tag      = 0x44,
            with_csr = True,
        )

        # Dedicated MMIO accessor the engine uses for SQ/CQ doorbells (no CSR; engine-owned).
        self.mmio_db = LiteNVMePCIeMmioAccessor(
            port     = mmio_db_port,
            tag      = 0x45,
            with_csr = False,
        )

        # Hardware I/O command engine (engine + dword->AXI bridge).
        self.io_engine = io_engine = LiteNVMeIOEngineAXI(
            qid=qid, qsize=qsize, qd=qd, data_width=data_width, with_csr=True,
            hostmem_base=hostmem_base)

        # Host-memory responder; the engine joins the AXI backend as an extra master.
        def hostmem_decoder(a):
            return (a >= hostmem_base) & (a < (hostmem_base + hostmem_size))
        self.hostmem_port = endpoint.crossbar.get_slave_port(address_decoder=hostmem_decoder)
        self.hostmem = LiteNVMeHostMemResponder(
            port              = self.hostmem_port,
            base              = hostmem_base,
            size              = hostmem_size,
            data_width        = data_width,
            with_csr          = True,
            extra_axi_masters = [io_engine.axi],
            backend           = hostmem_backend,
        )

        # Drive the engine's doorbells through the dedicated MMIO accessor.
        self.comb += io_engine.connect_mmio(self.mmio_db)

        # Optional hardware request generator (benchmark self-driver).
        if with_request_gen:
            from litenvme.request_gen import LiteNVMeRequestGen
            self.request_gen = request_gen = LiteNVMeRequestGen(with_csr=True)
            self.comb += [
                request_gen.source.connect(io_engine.sink),
                io_engine.source.connect(request_gen.sink),
            ]
            # Streams are owned by the generator in this mode.
            self.request    = None
            self.completion = None
        else:
            # Re-export the request/completion streams as the core's data-command interface.
            self.request    = io_engine.sink
            self.completion = io_engine.source

    def add_csrs(self, soc, name="nvme"):
        """Register the core's sub-blocks as CSR modules on `soc` with a common prefix.

        Convenience for SoC integration. Call after instantiating the core, e.g.:
            self.nvme = LiteNVMe(self.pcie_endpoint, ...)
            self.nvme.add_csrs(self)
        """
        soc.add_module(name=f"{name}_cfg",    module=self.cfg)
        soc.add_module(name=f"{name}_mmio",   module=self.mmio)
        soc.add_module(name=f"{name}_engine", module=self.io_engine.engine)
        if hasattr(self.hostmem, "csr"):
            soc.add_module(name=f"{name}_hostmem", module=self.hostmem.csr)
