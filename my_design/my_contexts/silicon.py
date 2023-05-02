# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.sky130_platform import Sky130Platform
from chipflow_lib.contexts.silicon import SiliconContext
from ..design import MySoC

pins = {
    "sys_clk":   0,
    "sys_rstn":  1,

    'uart_tx':   2,
    'uart_rx':   3,

    "flash_clk": 4,
    "flash_csn": 5,
    "flash_d0":  6,
    "flash_d1":  7,
    "flash_d2":  8,
    "flash_d3":  9,

    "gpio_0":   10,
    "gpio_1":   11,
    "gpio_2":   12,
    "gpio_3":   13,
    "gpio_4":   14,
    "gpio_5":   15,
    "gpio_6":   16,
    "gpio_7":   17,

    "btn_0":    18,
    "btn_1":    19,

    "jtag_tck": 33,
    "jtag_tms": 34,
    "jtag_tdi": 35,
    "jtag_tdo": 36,
}


class MySiliconContext(SiliconContext):
    def __init__(self, config):
        our_core_size = (285*10.0, 335*10.0)
        platform = Sky130Platform(pin_map=pins, core_size=our_core_size)

        super().__init__(config, platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design)
