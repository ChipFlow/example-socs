# SPDX-License-Identifier: BSD-2-Clause
from amaranth import *
from amaranth.build import *
from amaranth_boards.ulx3s import *
from amaranth_boards.ulx3s import *

from amaranth_orchard.memory.spimemio import QSPIPins
from amaranth_orchard.base.gpio import GPIOPins
from amaranth_orchard.io.uart import UARTPins
from amaranth_orchard.memory.hyperram import HyperRAMPins
from chipflow.providers.base import BaseProvider


class QSPIFlash(BaseProvider):
    def add(self, m):
        flash = QSPIPins()
        m.submodules.flash = self.platform.add_model("spiflash_model", flash, edge_det=['clk_o', 'csn_o'])
        return flash


class LEDGPIO(BaseProvider):
    def add(self, m):
        leds = GPIOPins(width=8)
        # TODO - something in simulation?
        return leds


class ButtonGPIO(BaseProvider):
    def add(self, m):
        buttons = GPIOPins(width=2)
        m.d.comb += buttons.i.eq(self.platform.buttons)
        return buttons


class UART(BaseProvider):
    def add(self, m):
        uart = UARTPins()
        m.submodules.uart_model = self.platform.add_model("uart_model", uart, edge_det=[])

        return uart


class HyperRAM(BaseProvider):
    def add(self, m):
        # Dual HyperRAM PMOD, starting at GPIO 0+/-
        hram = HyperRAMPins(cs_count=4)
        m.submodules.hram = self.platform.add_model("hyperram_model", hram, edge_det=['clk_o', ])
        return hram


class JTAG(BaseProvider):
    def add(self, m, cpu):
        m.d.comb += [
            cpu.jtag_tck.eq(0),
            cpu.jtag_tdi.eq(0),
            cpu.jtag_tms.eq(0),
        ]


class Init(BaseProvider):
    def add(self, m):
        m.domains.sync = ClockDomain()
        m.d.comb += ClockSignal().eq(self.platform.clk)
        m.d.comb += ResetSignal().eq(self.platform.rst)
