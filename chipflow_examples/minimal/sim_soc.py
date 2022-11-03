from .soc import Mpw5SoC
from ..common.sim_platform import SimPlatform

if __name__ == '__main__':
    platform = SimPlatform()
    platform.build(Mpw5SoC())
