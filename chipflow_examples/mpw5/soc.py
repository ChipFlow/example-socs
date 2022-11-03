from ..common.wrapper import SoCWrapper
from ..common.software.soft_gen import SoftwareGenerator

from amaranth_soc import wishbone

from amaranth_vexriscv.vexriscv import VexRiscv

from amaranth_orchard.base.gpio import GPIOPeripheral
from amaranth_orchard.memory.spimemio import SPIMemIO
from amaranth_orchard.io.uart import UARTPeripheral
from amaranth_orchard.memory.hyperram import HyperRAM
from amaranth_orchard.base.platform_timer import PlatformTimer
from amaranth_orchard.base.soc_id import SoCID

class Mpw5SoC(SoCWrapper):
    def __init__(self, *, large_cfg=False):
        super().__init__()

        # Memory regions
        self.spi_base = 0x00000000
        self.hyperram_base = 0x10000000

        # CSR regions
        self.spi_ctrl_base = 0xb0000000
        self.led_gpio_base = 0xb1000000
        self.uart_base = 0xb2000000
        self.timer_base = 0xb3000000
        self.soc_id_base = 0xb4000000
        self.hram_ctrl_base = 0xb5000000
        self.large_cfg = large_cfg

    def elaborate(self, platform):
        m = super().elaborate(platform)

        self._arbiter = wishbone.Arbiter(addr_width=30, data_width=32, granularity=8)
        self._decoder = wishbone.Decoder(addr_width=30, data_width=32, granularity=8)

        self.cpu = VexRiscv(config="LinuxMPW5L" if self.large_cfg else "LinuxMPW5", reset_vector=0x00100000)
        self._arbiter.add(self.cpu.ibus)
        self._arbiter.add(self.cpu.dbus)

        self.rom = SPIMemIO(flash=super().get_flash(m, platform))
        self._decoder.add(self.rom.data_bus, addr=self.spi_base)
        self._decoder.add(self.rom.ctrl_bus, addr=self.spi_ctrl_base)

        self.hyperram = HyperRAM(pins=super().get_hram(m, platform))
        self._decoder.add(self.hyperram.data_bus, addr=self.hyperram_base)
        self._decoder.add(self.hyperram.ctrl_bus, addr=self.hram_ctrl_base)

        self.gpio = GPIOPeripheral(pins=super().get_led_gpio(m, platform))
        self._decoder.add(self.gpio.bus, addr=self.led_gpio_base)

        self.uart = UARTPeripheral(
            init_divisor=(25000000//115200),
            pins=super().get_uart(m, platform))
        self._decoder.add(self.uart.bus, addr=self.uart_base)

        self.timer = PlatformTimer(width=48)
        self._decoder.add(self.timer.bus, addr=self.timer_base)

        soc_type = 0xBADCA77E if self.is_sim(platform) else 0xCA7F100F
        self.soc_id = SoCID(type_id=soc_type)
        self._decoder.add(self.soc_id.bus, addr=self.soc_id_base)

        m.submodules.arbiter  = self._arbiter
        m.submodules.cpu      = self.cpu
        m.submodules.decoder  = self._decoder
        m.submodules.rom      = self.rom
        m.submodules.hyperram = self.hyperram
        m.submodules.gpio     = self.gpio
        m.submodules.uart     = self.uart
        m.submodules.timer    = self.timer
        m.submodules.soc_id   = self.soc_id

        m.d.comb += [
            self._arbiter.bus.connect(self._decoder.bus),
            self.cpu.software_irq.eq(0),
            self.cpu.timer_irq.eq(self.timer.timer_irq),
        ]

        if self.is_sky130(platform):
            jtag_pins = super().get_jtag(m, platform)
            m.d.comb += [
                self.cpu.jtag_tck.eq(jtag_pins.tck_i),
                self.cpu.jtag_tdi.eq(jtag_pins.tdi_i),
                self.cpu.jtag_tms.eq(jtag_pins.tms_i),
                jtag_pins.tdo_o.eq(self.cpu.jtag_tdo),
            ]
        else:
            m.d.comb += [
                self.cpu.jtag_tck.eq(0),
                self.cpu.jtag_tdi.eq(0),
                self.cpu.jtag_tms.eq(0),
            ]

        if self.is_sim(platform):
            m.submodules.bus_mon = platform.add_monitor("wb_mon", self._decoder.bus)

        sw = SoftwareGenerator(
            rom_start=self.spi_base + 0x00100000, rom_size=0x00100000, # place BIOS at 1MB so room for bitstream
            ram_start=self.hyperram_base + 0x01F00000, ram_size=0x00100000, # place BIOS data at top of RAM above kernel (it stays resident)
        )

        sw.add_periph("spiflash", "FLASH_CTRL", self.spi_ctrl_base)
        sw.add_periph("gpio", "LED_GPIO", self.led_gpio_base)
        sw.add_periph("uart", "UART0", self.uart_base)
        sw.add_periph("plat_timer", "TIMER0", self.timer_base)
        sw.add_periph("soc_id", "SOC_ID", self.soc_id_base)

        sw.generate("chipflow_examples/mpw5/software/generated")

        return m

