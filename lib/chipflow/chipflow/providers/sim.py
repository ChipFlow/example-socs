from amaranth import *
from amaranth.build import *
from amaranth_boards.ulx3s import *
from amaranth_boards.ulx3s import *

from amaranth_orchard.memory.spimemio import QSPIPins
from amaranth_orchard.base.gpio import GPIOPins
from amaranth_orchard.io.uart import UARTPins
from amaranth_orchard.memory.hyperram import HyperRAMPins

class QSPIFlash():
    def add(self, m, platform):
        flash = QSPIPins()
        m.submodules.flash = platform.add_model("spiflash_model", flash, edge_det=['clk_o', 'csn_o'])
        return flash


class LEDGPIO():
    def add(self, m, platform):
        leds = GPIOPins(width=8)
        # TODO - something in simulation?
        return leds

class UART():
    def add(self, m, platform):
        uart = UARTPins()
        m.submodules.uart_model = platform.add_model("uart_model", uart, edge_det=[])

        return uart

class HyperRAM():
    def add(self, m, platform):
        # Dual HyperRAM PMOD, starting at GPIO 0+/-
        hram = HyperRAMPins(cs_count=4)
        m.submodules.hram = platform.add_model("hyperram_model", hram, edge_det=['clk_o', ])
        return hram

class JTAG:
    def add(self, m, platform, cpu):
        m.d.comb += [
            cpu.jtag_tck.eq(0),
            cpu.jtag_tdi.eq(0),
            cpu.jtag_tms.eq(0),
        ]

class Init:
    def add(self, m, platform):
        m.domains.sync = ClockDomain()
        m.d.comb += ClockSignal().eq(platform.clk)
        m.d.comb += ResetSignal().eq(platform.rst)
