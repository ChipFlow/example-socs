# SPDX-License-Identifier: BSD-2-Clause
from chipflow_lib.platforms.sim import SimPlatform
from chipflow_lib.software.soft_gen import SoftwareGenerator

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import connect

from amaranth_soc import csr, wishbone, gpio
from amaranth_soc.csr.wishbone import WishboneCSRBridge

from amaranth_vexriscv.vexriscv import VexRiscv

from amaranth_orchard.memory.spimemio import SPIMemIO
from amaranth_orchard.io.uart import UARTPeripheral
from amaranth_orchard.memory.sram import SRAMPeripheral
from amaranth_orchard.base.platform_timer import PlatformTimer
from amaranth_orchard.base.soc_id import SoCID


__all__ = ["MySoC"]


class MySoC(wiring.Component):
    def __init__(self):
        super().__init__({})

        # Memory regions:
        self.mem_spiflash_base = 0x00000000
        self.mem_sram_base     = 0x10000000

        # CSR regions:
        self.csr_base          = 0xb0000000
        self.csr_spiflash_base = 0xb0000000
        self.csr_led_gpio_base = 0xb1000000
        self.csr_uart_base     = 0xb2000000
        self.csr_timer_base    = 0xb3000000
        self.csr_soc_id_base   = 0xb4000000
        #self.csr_btn_gpio_base = 0xb5000000

        self.sram_size  = 0x400 # 1KiB
        self.bios_start = 0x100000 # 1MiB into spiflash to make room for a bitstream

    def elaborate(self, platform):
        m = Module()

        m.domains += ClockDomain("sync")
        m.submodules.clock_reset_provider = platform.providers.ClockResetProvider()

        wb_arbiter  = wishbone.Arbiter(addr_width=30, data_width=32, granularity=8)
        wb_decoder  = wishbone.Decoder(addr_width=30, data_width=32, granularity=8)
        csr_decoder = csr.Decoder(addr_width=28, data_width=8)

        m.submodules.wb_arbiter  = wb_arbiter
        m.submodules.wb_decoder  = wb_decoder
        m.submodules.csr_decoder = csr_decoder

        connect(m, wb_arbiter.bus, wb_decoder.bus)

        # CPU

        cpu = VexRiscv(config="LiteDebug", reset_vector=self.bios_start)
        wb_arbiter.add(cpu.ibus)
        wb_arbiter.add(cpu.dbus)

        m.submodules.cpu = cpu

        # SPI flash

        spiflash_provider = platform.providers.QSPIFlashProvider()
        spiflash = SPIMemIO(flash=spiflash_provider.pins)
        wb_decoder .add(spiflash.data_bus, name="spiflash", addr=self.mem_spiflash_base)
        csr_decoder.add(spiflash.ctrl_bus, name="spiflash", addr=self.csr_spiflash_base - self.csr_base)

        m.submodules.spiflash_provider = spiflash_provider
        m.submodules.spiflash = spiflash

        # SRAM

        sram = SRAMPeripheral(size=self.sram_size)
        wb_decoder.add(sram.bus, name="sram", addr=self.mem_sram_base)

        m.submodules.sram = sram

        # LED GPIOs

        led_gpio_provider = platform.providers.LEDGPIOProvider()
        led_gpio = gpio.Peripheral(pin_count=8, addr_width=4, data_width=8)

        m.submodules.led_gpio_provider = led_gpio_provider
        m.submodules.led_gpio = led_gpio

        for n in range(8):
            connect(m, led_gpio.pins[n], led_gpio_provider.pins[n])

        csr_decoder.add(led_gpio.bus, name="led_gpio", addr=self.csr_led_gpio_base - self.csr_base)

        # UART

        uart_provider = platform.providers.UARTProvider()
        uart = UARTPeripheral(init_divisor=int(25e6//115200), pins=uart_provider.pins)
        csr_decoder.add(uart.bus, name="uart", addr=self.csr_uart_base - self.csr_base)

        m.submodules.uart_provider = uart_provider
        m.submodules.uart = uart

        # Timer

        timer = PlatformTimer()
        csr_decoder.add(timer.bus, name="timer", addr=self.csr_timer_base - self.csr_base)

        m.submodules.timer = timer
        m.d.comb += cpu.timer_irq.eq(timer.irq)

        # SoC ID

        soc_id = SoCID(type_id=0xCA7F100F)
        csr_decoder.add(soc_id.bus, name="soc_id", addr=self.csr_soc_id_base - self.csr_base)

        m.submodules.soc_id = soc_id

        # Button GPIOs

        #btn_gpio_provider = platform.providers.ButtonGPIOProvider()
        #btn_gpio = gpio.Peripheral(pin_count=2, addr_width=4, data_width=8)

        #m.submodules.btn_gpio_provider = btn_gpio_provider
        #m.submodules.btn_gpio = btn_gpio

        #for n in range(2):
        #    connect(m, btn_gpio.pins[n], btn_gpio_provider.pins[n])

        #csr_decoder.add(btn_gpio.bus, name="btn_gpio", addr=self.csr_btn_gpio_base - self.csr_base)

        # Wishbone-CSR bridge

        wb_to_csr = WishboneCSRBridge(csr_decoder.bus, data_width=32)
        wb_decoder.add(wb_to_csr.wb_bus, name="csr", addr=self.csr_base, sparse=False)

        m.submodules.wb_to_csr = wb_to_csr

        # Debug support

        m.submodules.jtag_provider = platform.providers.JTAGProvider(cpu)

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
