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
        platform.connect_io(m, flash, "flash")
        return flash

class LEDGPIO():
    def add(self, m, platform):
        leds = GPIOPins(width=8)
        platform.connect_io(m, leds, "gpio")
        return leds

class UART():
    def add(self, m, platform):
        uart = UARTPins()
        
        platform.connect_io(m, uart, "uart")
        
        return uart

class HyperRAM():
    def add(self, m, platform):
        # Dual HyperRAM PMOD, starting at GPIO 0+/-
        hram = HyperRAMPins(cs_count=4)

        platform.connect_io(m, hram, "ram")
    
        return hram

class JTAG:
    def add(self, m, platform, cpu):
        jtag_io = Record([
            ('tck_i', 1),
            ('tms_i', 1),
            ('tdi_i', 1),
            ('tdo_o', 1),
        ])
        platform.connect_io(m, jtag_io, "jtag")

        m.d.comb += [
            cpu.jtag_tck.eq(jtag_io.tck_i),
            cpu.jtag_tdi.eq(jtag_io.tdi_i),
            cpu.jtag_tms.eq(jtag_io.tms_i),
            jtag_io.tdo_o.eq(cpu.jtag_tdo),
        ]

        return jtag_io

class Init:
    def add(self, m, platform):
        sys_io = Record([
            ('clk_i', 1),
            ('rstn_i', 1),
        ])
        platform.connect_io(m, sys_io, "sys")
        m.domains.sync = ClockDomain()
        m.d.comb += ClockSignal().eq(sys_io.clk_i)

        rst = Signal()
        m.d.comb += rst.eq(~sys_io.rstn_i)
        rst_sync0 = Signal(reset_less=True)
        rst_sync1 = Signal(reset_less=True)
        m.d.sync += [
            rst_sync0.eq(rst),
            rst_sync1.eq(rst_sync0),
        ]
        m.d.comb += [
            ResetSignal().eq(rst_sync1),
        ]

