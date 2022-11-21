import sys, argparse, os, tomli, importlib
from doit.cmd_base import ModuleTaskLoader
from doit.doit_cmd import DoitMain

class ChipFlowError(Exception):
    pass

class Main():
    def run_sim(self, args):
        if args.sim_action == "build-yosys":
            return self._sim_build_yosys()

        import chipflow.sim.doit_build

        design_module = self._load_design_module()
        design_dir = os.path.dirname(design_module.__file__)

        cmd = [
            "set_params", "-d", design_dir, 
            "build_sim"
        ]

        DoitMain(ModuleTaskLoader(chipflow.sim.doit_build)).run(cmd)

    def run_software(self, args):
        import chipflow.software.doit_build

        design_module = self._load_design_module()
        design_dir = os.path.dirname(design_module.__file__)

        cmd = [
            "set_params", "-d", design_dir, 
            # TODO: Sort out file linking in doit so we don't need to call these
            "gather_chipflow_library_deps",
            "gather_project_deps",
            "build_bios_bin"
        ]

        DoitMain(ModuleTaskLoader(chipflow.software.doit_build)).run(cmd)

    def run_board(self, args):
        context = self._load("load_board_context")
        context.build()

    def run_gen_rtlil(self, args):
        context = self._load("load_silicon_context")
        context.build()

    def run_path(self, args):
        print(os.path.dirname(__file__))

    def run(self):    
        parser = self._build_arg_parser()

        args = parser.parse_args(sys.argv[1:])

        self._parse_config(args)
    
        getattr(self, 'run_' + args.action)(args)

    def _sim_build_yosys(self):
        context = self._load("load_sim_context")
        context.build()

    def _build_arg_parser(self):
        parser = argparse.ArgumentParser()

        parser_action = parser.add_subparsers(dest="action")

        # Sim
        sim_parser = parser_action.add_parser("sim", help="Simulate the design.")
        sim_action = sim_parser.add_subparsers(dest="sim_action")
        sim_action.add_parser("build", help="Build the simulation binary.")
        sim_action.add_parser("build-yosys", help="Build the intermediate yosys simulation.")

        parser_action.add_parser("gen_rtlil", help="Generate RTLIL")
        parser_action.add_parser("board", help="Build the design for a board.")
        parser_action.add_parser("path", help="Get path of module.")
        
        # Software/BIOS
        software_parser = parser_action.add_parser("software", help="Software.")
        software_action = software_parser.add_subparsers(dest="software_action")
        software_action.add_parser("build", help="Build.")

        return parser

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

    def _load_design_module(self):
        try:
            module_loc = self.config["chipflow"]["design_module"]
            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Could not load design module, {module_loc}.") from error

        return module

if __name__ == '__main__':
    Main().run()
