import sys, argparse

class Main():
    def _simulate(self, args):
        # TODO: module should be an argument
        from my_design.design import Mpw5SoC
        from .sim_platform import SimPlatform

        platform = SimPlatform()
        platform.build(Mpw5SoC(large_cfg=True))

    def _ulx3s(self, args):
        from amaranth_boards.ulx3s import ULX3S_85F_Platform
        from my_design.design import Mpw5SoC

        platform = ULX3S_85F_Platform()
        platform.build(Mpw5SoC(), do_program=False)

    def _gen_rtlil(self, args):
        from my_design.sky130 import Platform

        platform = Platform()
        platform.build(large_cfg=args.large)

    def run(self):    
        parser = argparse.ArgumentParser()
        parser.add_argument('action', type=str, default=None)

        args = parser.parse_args(sys.argv[1:])
        getattr(self, '_' + args.action)(args)
        # print(args)

if __name__ == '__main__':
    Main().run()
