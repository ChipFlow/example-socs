import sys, argparse, os, tomli, importlib

class ChipFlowError(Exception):
    pass

class Main():
    def run_simulate(self, args):
        from .sim_platform import SimPlatform

        platform = SimPlatform()
        platform.build(self._load_design())

    def run_ulx3s(self, args):
        # TODO: refactor this/make this load from config
        from amaranth_boards.ulx3s import ULX3S_85F_Platform

        platform = ULX3S_85F_Platform()
        platform.build(self._load_design(), do_program=False)

    def run_gen_rtlil(self, args):
        # TODO: refactor this/make this load from config
        from my_design.sky130 import Platform

        platform = Platform()
        platform.build()

    def run(self):    
        parser = argparse.ArgumentParser()
        parser.add_argument('action', type=str, default=None)

        args = parser.parse_args(sys.argv[1:])

        self._parse_config(args)
    
        getattr(self, 'run_' + args.action)(args)

    def _parse_config(self, args):
        config_dir = os.getcwd();
        config_file = f"{config_dir}/chipflow.toml"

        # TODO: better errors
        with open(config_file, mode="rb") as fp:
            self.config = tomli.load(fp)

    def _load_design(self):
        try:
            module_loc = self.config["chipflow"]["design_module"]
            class_name = self.config["chipflow"]["design_class"]

            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Could not load design module, {module_loc}.") from error

        if (not hasattr(module, class_name)):
            raise ChipFlowError(f"Design module is missing class. {module_loc}, {class_name}");

        return getattr(module, class_name)()

if __name__ == '__main__':
    Main().run()
