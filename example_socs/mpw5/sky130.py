import sys, argparse
from .soc import Mpw5SoC
from ..common.sky130_platform import Sky130Platform

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

    "ram_rstn": 10,
    "ram_csn0": 11,
    "ram_csn1": 12,
    "ram_csn2": 13,
    "ram_csn3": 14,
    "ram_clk":  15,
    "ram_rwds": 16,
    "ram_dq0":  17,
    "ram_dq1":  18,
    "ram_dq2":  19,
    "ram_dq3":  20,
    "ram_dq4":  21,
    "ram_dq5":  22,
    "ram_dq6":  23,
    "ram_dq7":  24,

    "gpio_0":   25,
    "gpio_1":   26,
    "gpio_2":   27,
    "gpio_3":   28,
    "gpio_4":   29,
    "gpio_5":   30,
    "gpio_6":   31,
    "gpio_7":   32,

    "jtag_tck": 33,
    "jtag_tms": 34,
    "jtag_tdi": 35,
    "jtag_tdo": 36,
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--synth', action='store_true')
    parser.add_argument('--pnr', action='store_true')
    parser.add_argument('--large', action='store_true')

    args = parser.parse_args(sys.argv[1:])

    platform = Sky130Platform(pin_map=pins, core_size=(285*10.0, 335*10.0) if args.large else (240*10.0, 240*10.0))
    platform.build(Mpw5SoC(large_cfg=args.large), synth=args.synth, pnr=args.pnr)
