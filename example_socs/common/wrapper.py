from amaranth import *
from amaranth.build import *
from amaranth.lib.cdc import ResetSynchronizer
from amaranth_boards.ulx3s import *
from amaranth_boards.ulx3s import *

from amaranth_orchard.memory.spimemio import QSPIPins
from amaranth_orchard.base.gpio import GPIOPins
from amaranth_orchard.io.uart import UARTPins
from amaranth_orchard.memory.hyperram import HyperRAMPins

class SoCWrapper(Elaboratable):
    """
    This wrapper provides glue to simplify use of the ULX3S platform, and integrate between
    the Amaranth platform and the format of pins that the IP cores expect.
    """

    def is_sim(self, platform):
        return hasattr(platform, "is_sim") and platform.is_sim

    def get_flash(self, m, platform):
        flash = QSPIPins()
        if self.is_sim(platform):
            m.submodules.flash = platform.add_model("spiflash_model", flash, edge_det=['clk_o', 'csn_o'])
        else:
            plat_flash = platform.request("spi_flash", dir=dict(cs='-', copi='-', cipo='-', wp='-', hold='-'))
            # Flash clock requires a special primitive to access in ECP5
            m.submodules.usrmclk = Instance("USRMCLK",
                i_USRMCLKI=flash.clk_o,
                i_USRMCLKTS=ResetSignal(), # tristate in reset for programmer accesss
                a_keep=1,
            )
            # IO pins and buffers
            m.submodules += Instance("OBZ",
                o_O=plat_flash.cs.io,
                i_I=flash.csn_o,
                i_T=ResetSignal(),
            )
            # Pins in order
            data_pins = ["copi", "cipo", "wp", "hold"]

            for i in range(4):
                m.submodules += Instance("BB",
                    io_B=getattr(plat_flash, data_pins[i]).io,
                    i_I=flash.d_o[i],
                    i_T=~flash.d_oe[i],
                    o_O=flash.d_i[i]
                )
        return flash

    def get_led_gpio(self, m, platform):
        leds = GPIOPins(width=8)
        if self.is_sim(platform):
            # TODO
            pass
        else:
            for i in range(8):
                led = platform.request("led", i)
                m.d.comb += led.o.eq(leds.o[i])
        return leds

    def get_uart(self, m, platform):
        uart = UARTPins()
        if self.is_sim(platform):
            m.submodules.uart_model = platform.add_model("uart_model", uart, edge_det=[])
        else:
            plat_uart = platform.request("uart")
            m.d.comb += [
                plat_uart.tx.o.eq(uart.tx_o),
                uart.rx_i.eq(plat_uart.rx.i),
            ]
        return uart

    def get_hram(self, m, platform):
        # Dual HyperRAM PMOD, starting at GPIO 0+/-
        hram = HyperRAMPins(cs_count=4)
        if self.is_sim(platform):
            m.submodules.hram = platform.add_model("hyperram_model", hram, edge_det=['clk_o', ])
        else:
            platform.add_resources([
                Resource("hyperram", 0,
                    Subsignal("csn",    Pins("9- 9+ 10- 10+", conn=("gpio", 0), dir='o')),
                    Subsignal("rstn",   Pins("8+", conn=("gpio", 0), dir='o')),
                    Subsignal("clk",    Pins("8-", conn=("gpio", 0), dir='o')),
                    Subsignal("rwds",   Pins("7+", conn=("gpio", 0), dir='io')),

                    Subsignal("dq",     Pins("3- 2- 1- 0- 0+ 1+ 2+ 3+", conn=("gpio", 0), dir='io')),

                    Attrs(IO_TYPE="LVCMOS33"),
                )
            ])

            plat_hram = platform.request("hyperram", 0)
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

    def elaborate(self, platform):
        m = Module()
        if self.is_sim(platform):
            m.domains.sync = ClockDomain()
            m.d.comb += ClockSignal().eq(platform.clk)
            m.d.comb += ResetSignal().eq(platform.rst)
        else:
            clk25 = platform.request("clk25")
            m.domains.sync = ClockDomain()
            m.d.comb += ClockSignal().eq(clk25.i)
            reset_in = platform.request("button_pwr", 0)
            m.submodules += ResetSynchronizer(reset_in)
        return m
