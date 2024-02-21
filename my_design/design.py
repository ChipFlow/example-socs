# SPDX-License-Identifier: BSD-2-Clause
from chipflow_lib.platforms.sim import SimPlatform
from chipflow_lib.software.soft_gen import SoftwareGenerator

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import connect

from amaranth_soc import csr, wishbone
from amaranth_soc.csr.wishbone import WishboneCSRBridge

from amaranth_vexriscv.vexriscv import VexRiscv

from amaranth_orchard.base.gpio import GPIOPeripheral
from amaranth_orchard.memory.spimemio import SPIMemIO
from amaranth_orchard.io.uart import UARTPeripheral
from amaranth_orchard.memory.sram import SRAMPeripheral
from amaranth_orchard.base.platform_timer import PlatformTimer
from amaranth_orchard.base.soc_id import SoCID

from .cv32e40p_wrapper import CV32E40P, DebugModule

__all__ = ["MySoC"]


class MySoC(wiring.Component):
    def __init__(self):
        super().__init__({})

        # Memory regions:
        self.mem_spiflash_base = 0x00000000
        self.mem_sram_base     = 0x10000000

        # Debug region
        self.debug_base        = 0xa0000000

        # CSR regions:
        self.csr_base          = 0xb0000000
        self.csr_spiflash_base = 0xb0000000
        self.csr_led_gpio_base = 0xb1000000
        self.csr_uart_base     = 0xb2000000
        self.csr_timer_base    = 0xb3000000
        self.csr_soc_id_base   = 0xb4000000
        #self.csr_btn_gpio_base = 0xb5000000

        self.sram_size  = 0x2000 # 8KiB
        self.bios_start = 0x100000 # 1MiB into spiflash to make room for a bitstream

    def elaborate(self, platform):
        m = Module()

        m.submodules.clock_reset_provider = platform.providers.ClockResetProvider()

        wb_arbiter  = wishbone.Arbiter(addr_width=30, data_width=32, granularity=8)
        wb_decoder  = wishbone.Decoder(addr_width=30, data_width=32, granularity=8)
        csr_decoder = csr.Decoder(addr_width=28, data_width=8)

        m.submodules.wb_arbiter  = wb_arbiter
        m.submodules.wb_decoder  = wb_decoder
        m.submodules.csr_decoder = csr_decoder

        connect(m, wb_arbiter.bus, wb_decoder.bus)

        # CPU

        cpu = CV32E40P(config="default", reset_vector=self.bios_start, dm_haltaddress=self.debug_base+0x800)
        wb_arbiter.add(cpu.ibus)
        wb_arbiter.add(cpu.dbus)

        m.submodules.cpu = cpu

        # Debug
        debug = DebugModule()
        wb_arbiter.add(debug.initiator)
        wb_decoder.add(debug.target, addr=self.debug_base)
        m.d.comb += cpu.debug_req.eq(debug.debug_req)

        m.submodules.debug = debug
        # SPI flash

        spiflash_provider = platform.providers.QSPIFlashProvider()
        spiflash = SPIMemIO(name="spiflash", flash=spiflash_provider.pins)
        wb_decoder .add(spiflash.data_bus, addr=self.mem_spiflash_base)
        csr_decoder.add(spiflash.ctrl_bus, addr=self.csr_spiflash_base - self.csr_base)

        m.submodules.spiflash_provider = spiflash_provider
        m.submodules.spiflash = spiflash

        # SRAM

        sram = SRAMPeripheral(name="sram", size=self.sram_size)
        wb_decoder.add(sram.bus, addr=self.mem_sram_base)

        m.submodules.sram = sram

        # LED GPIOs

        led_gpio_provider = platform.providers.LEDGPIOProvider()
        led_gpio = GPIOPeripheral(name="led_gpio", pins=led_gpio_provider.pins)
        csr_decoder.add(led_gpio.bus, addr=self.csr_led_gpio_base - self.csr_base)

        m.submodules.led_gpio_provider = led_gpio_provider
        m.submodules.led_gpio = led_gpio

        # UART

        uart_provider = platform.providers.UARTProvider()
        uart = UARTPeripheral(name="uart", init_divisor=int(25e6//115200), pins=uart_provider.pins)
        csr_decoder.add(uart.bus, addr=self.csr_uart_base - self.csr_base)

        m.submodules.uart_provider = uart_provider
        m.submodules.uart = uart

        # Timer

        timer = PlatformTimer(name="timer")
        csr_decoder.add(timer.bus, addr=self.csr_timer_base - self.csr_base)

        m.submodules.timer = timer
        m.d.comb += cpu.timer_irq.eq(timer.irq)

        # SoC ID

        soc_id = SoCID(name="soc_id", type_id=0xCA7F100F)
        csr_decoder.add(soc_id.bus, addr=self.csr_soc_id_base - self.csr_base)

        m.submodules.soc_id = soc_id

        # Button GPIOs

        #btn_gpio_provider = platform.providers.ButtonGPIOProvider()
        #btn_gpio = GPIOPeripheral(name="btn_gpio", pins=btn_gpio_provider.pins)
        #csr_decoder.add(btn_gpio.bus, addr=self.csr_btn_gpio_base - self.csr_base)

        #m.submodules.btn_gpio_provider = btn_gpio_provider
        #m.submodules.btn_gpio = btn_gpio

        # Wishbone-CSR bridge

        wb_to_csr = WishboneCSRBridge(csr_decoder.bus, data_width=32, name="csr")
        wb_decoder.add(wb_to_csr.wb_bus, addr=self.csr_base, sparse=False)

        m.submodules.wb_to_csr = wb_to_csr

        # Debug support

        m.submodules.jtag_provider = platform.providers.JTAGProvider(debug)

        if isinstance(platform, SimPlatform):
            m.submodules.wb_mon = platform.add_monitor("wb_mon", wb_decoder.bus)

        # Software

        sw = SoftwareGenerator(rom_start=self.bios_start, rom_size=0x00100000,
                               # place BIOS data in SRAM
                               ram_start=self.mem_sram_base, ram_size=self.sram_size)

        sw.add_periph("spiflash",   "SPIFLASH", self.csr_spiflash_base)
        sw.add_periph("gpio",       "LED_GPIO", self.csr_led_gpio_base)
        sw.add_periph("uart",       "UART",     self.csr_uart_base)
        sw.add_periph("plat_timer", "TIMER",    self.csr_timer_base)
        sw.add_periph("soc_id",     "SOC_ID",   self.csr_soc_id_base)
        #sw.add_periph("gpio",       "BTN_GPIO", self.csr_btn_gpio_base)

        sw.generate("build/software/generated")

        return m
