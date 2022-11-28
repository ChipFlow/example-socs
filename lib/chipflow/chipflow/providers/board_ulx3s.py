# SPDX-License-Identifier: BSD-2-Clause
from amaranth import *
from amaranth.build import *
from amaranth_boards.ulx3s import *
from amaranth_boards.ulx3s import *
from amaranth.lib.cdc import ResetSynchronizer
from amaranth_orchard.memory.spimemio import QSPIPins
from amaranth_orchard.base.gpio import GPIOPins
from amaranth_orchard.io.uart import UARTPins
from amaranth_orchard.memory.hyperram import HyperRAMPins
from chipflow.providers.base import BaseProvider


class QSPIFlash(BaseProvider):
    def add(self, m):
        flash = QSPIPins()

        plat_flash = self.platform.request("spi_flash", dir=dict(cs='-', copi='-', cipo='-', wp='-', hold='-'))
        # Flash clock requires a special primitive to access in ECP5
        m.submodules.usrmclk = Instance(
            "USRMCLK",
            i_USRMCLKI=flash.clk_o,
            i_USRMCLKTS=ResetSignal(),  # tristate in reset for programmer accesss
            a_keep=1,
        )
        # IO pins and buffers
        m.submodules += Instance(
            "OBZ",
            o_O=plat_flash.cs.io,
            i_I=flash.csn_o,
            i_T=ResetSignal(),
        )
        # Pins in order
        data_pins = ["copi", "cipo", "wp", "hold"]

        for i in range(4):
            m.submodules += Instance(
                "BB",
                io_B=getattr(plat_flash, data_pins[i]).io,
                i_I=flash.d_o[i],
                i_T=~flash.d_oe[i],
                o_O=flash.d_i[i]
            )
        return flash


class LEDGPIO(BaseProvider):
    def add(self, m):
        leds = GPIOPins(width=8)

        for i in range(8):
            led = self.platform.request("led", i)
            m.d.comb += led.o.eq(leds.o[i])

        return leds


class ButtonGPIO(BaseProvider):
    def add(self, m):
        buttons = GPIOPins(width=2)

        for i in range(2):
            btn = self.platform.request("button_fire", i)
            m.d.comb += buttons.i[i].eq(btn.i)

        return buttons


class UART(BaseProvider):
    def add(self, m):
        uart = UARTPins()

        plat_uart = self.platform.request("uart")
        m.d.comb += [
            plat_uart.tx.o.eq(uart.tx_o),
            uart.rx_i.eq(plat_uart.rx.i),
        ]

        return uart


class HyperRAM(BaseProvider):
    def add(self, m):
        # Dual HyperRAM PMOD, starting at GPIO 0+/-
        hram = HyperRAMPins(cs_count=4)

        self.platform.add_resources([
            Resource(
                "hyperram",
                0,
                Subsignal("csn",    Pins("9- 9+ 10- 10+", conn=("gpio", 0), dir='o')),
                Subsignal("rstn",   Pins("8+", conn=("gpio", 0), dir='o')),
                Subsignal("clk",    Pins("8-", conn=("gpio", 0), dir='o')),
                Subsignal("rwds",   Pins("7+", conn=("gpio", 0), dir='io')),

                Subsignal("dq",     Pins("3- 2- 1- 0- 0+ 1+ 2+ 3+", conn=("gpio", 0), dir='io')),

                Attrs(IO_TYPE="LVCMOS33"),
            )
        ])

        plat_hram = self.platform.request("hyperram", 0)
        m.d.comb += [
            plat_hram.clk.o.eq(hram.clk_o),
            plat_hram.csn.o.eq(hram.csn_o),
            plat_hram.rstn.o.eq(hram.rstn_o),

            plat_hram.rwds.o.eq(hram.rwds_o),
            plat_hram.rwds.oe.eq(hram.rwds_oe),
            hram.rwds_i.eq(plat_hram.rwds.i),

            plat_hram.dq.o.eq(hram.dq_o),
            plat_hram.dq.oe.eq(hram.dq_oe),
            hram.dq_i.eq(plat_hram.dq.i),
        ]

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
        clk25 = self.platform.request("clk25")
        m.domains.sync = ClockDomain()
        m.d.comb += ClockSignal().eq(clk25.i)
        reset_in = self.platform.request("button_pwr", 0)
        m.submodules += ResetSynchronizer(reset_in)
