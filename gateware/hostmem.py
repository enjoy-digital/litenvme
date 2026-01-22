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
        # Port field helpers (dat vs
