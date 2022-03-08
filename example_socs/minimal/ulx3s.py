from amaranth import *
from amaranth_boards.ulx3s import *

from .soc import Mpw5SoC

if __name__ == "__main__":
    platform = ULX3S_85F_Platform()
    # TODO: build and program BIOS, too
    platform.build(Mpw5SoC(), do_program=True)
