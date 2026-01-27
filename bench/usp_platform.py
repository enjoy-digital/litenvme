from litex.build.generic_platform import *
from litex.build.xilinx           import XilinxUSPPlatform
from litex.build.openfpgaloader   import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk200", 0,
        Subsignal("p", Pins("T24")),
        Subsignal("n", Pins("U24")),
        IOStandard("DIFF_SSTL12")
    ),

    # Serial.
    ("serial", 0,
        Subsignal("rx", Pins("C12"), IOStandard("LVCMOS33")), # GPIO-0.
        Subsignal("tx", Pins("E12"), IOStandard("LVCMOS33")), # GPIO-1.
    ),

    # LEDs.
    ("user_led", 0, Pins("H12"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("J12"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("H13"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("G14"), IOStandard("LVCMOS33")),
    ("user_led", 4, Pins("J13"), IOStandard("LVCMOS33")),
    ("user_led", 5, Pins("H14"), IOStandard("LVCMOS33")),
    ("user_led", 6, Pins("J14"), IOStandard("LVCMOS33")),
    ("user_led", 7, Pins("J15"), IOStandard("LVCMOS33")),

    # GPIOs.
    ("gpio", 0, Pins("C12"), IOStandard("LVCMOS33")),
    ("gpio", 1, Pins("E12"), IOStandard("LVCMOS33")),
    ("gpio", 2, Pins("C13"), IOStandard("LVCMOS33")),
    ("gpio", 3, Pins("C14"), IOStandard("LVCMOS33")),
    ("gpio", 4, Pins("B12"), IOStandard("LVCMOS33")),
    ("gpio", 5, Pins("E13"), IOStandard("LVCMOS33")),
    ("gpio", 6, Pins("D13"), IOStandard("LVCMOS33")),
    ("gpio", 7, Pins("D14"), IOStandard("LVCMOS33")),

    # PCIe.
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("AC13"), IOStandard("LVCMOS33")),
        Subsignal("wake",  Pins("AC14"), IOStandard("LVCMOS33")),
        Subsignal("pedet", Pins("AD14"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("AB7")),
        Subsignal("clk_n", Pins("AB6")),
        Subsignal("rx_p",  Pins("AF2 AE4 AD2 AB2")),
        Subsignal("rx_n",  Pins("AF1 AE3 AD1 AB1")),
        Subsignal("tx_p",  Pins("AF7 AE9 AD7 AC5")),
        Subsignal("tx_n",  Pins("AF6 AE8 AD6 AC4")),
    ),

    # PCIe.
    ("pcie_clk", 0,
        Subsignal("p", Pins("AB7")),
        Subsignal("n", Pins("AB6")),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxUSPPlatform):
    default_clk_name   = "clk200"
    default_clk_period = 1e9/200e6

    def __init__(self, toolchain="vivado"):
        XilinxUSPPlatform.__init__(self, "xcau10p-ffvb676-2-i", _io, toolchain=toolchain)

    def create_programmer(self):
        return OpenFPGALoader(fpga_part="xcau10p-ffvb676", cable="digilent_hs2")

    def do_finalize(self, fragment):
        XilinxUSPPlatform.do_finalize(self, fragment)

        # Timings Constraints.
        self.add_period_constraint(self.lookup_request("clk200",        loose=True), 1e9/200e6)
        self.add_period_constraint(self.lookup_request("pcie_x4:clk_p", loose=True), 1e9/100e6)

        # Bitstreams Parameters.
        self.add_platform_command("set_property BITSTREAM.GENERAL.COMPRESS TRUE     [current_design]")
        self.add_platform_command("set_property BITSTREAM.CONFIG.CONFIGRATE 51.0    [current_design]")
        self.add_platform_command("set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4     [current_design]")
        self.add_platform_command("set_property BITSTREAM.CONFIG.UNUSEDPIN PULLNONE [current_design]")
