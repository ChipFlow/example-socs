import sys, argparse, os, tomli, importlib

class ChipFlowError(Exception):
    pass

class Main():
    def run_simulate(self, args):
        context = self._load("load_sim_context")
        context.build()

    def run_ulx3s(self, args):
        context = self._load("load_board_context")
        context.build()

    def run_gen_rtlil(self, args):
        context = self._load("load_silicon_context")
        context.build()

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

    def _load(self, loader_name):
        try:
            module_loc = self.config["chipflow"]["loader_module"]

            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Could not locate module, {module_loc}.") from error

        if (not hasattr(module, loader_name)):
            raise ChipFlowError(f"Loader module is missing loader. module={module_loc}, loader={loader_name}");

        return getattr(module, loader_name)(self.config)

if __name__ == '__main__':
    Main().run()
