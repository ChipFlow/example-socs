# SPDX-License-Identifier: BSD-2-Clause
import sys
import argparse
import os
import tomli
import importlib
from doit.cmd_base import ModuleTaskLoader
from doit.doit_cmd import DoitMain


class ChipFlowError(Exception):
    pass


class Main():
    def run_sim(self, args):
        if args.sim_action == "build-yosys":
            return self._sim_build_yosys()

        module_loc = self.config["chipflow"]["sim_module"]
        doit_build_module = self._load_module(module_loc + ".doit_build")

        cmd = ["build_sim"]
        DoitMain(ModuleTaskLoader(doit_build_module)).run(cmd)

    def run_software(self, args):
        module_loc = self.config["chipflow"]["software_module"]
        doit_build_module = self._load_module(module_loc + ".doit_build")

        cmd = ["build_software"]
        DoitMain(ModuleTaskLoader(doit_build_module)).run(cmd)

    def run_board(self, args):
        context = self._load("load_board_context")
        context.build()

    def run_silicon_rtlil(self, args):
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

        # Board
        parser_action.add_parser("board", help="Build the design for a board.")
        parser_action.add_parser("path", help="Get path of module.")

        # Software/BIOS
        software_parser = parser_action.add_parser("software", help="Software.")
        software_action = software_parser.add_subparsers(dest="software_action")
        software_action.add_parser("build", help="Build.")

        # Silicon
        parser_action.add_parser("silicon_rtlil", help="Generate RTLIL for silicon.")

        return parser

    def _parse_config(self, args):
        config_dir = os.getcwd()
        config_file = f"{config_dir}/chipflow.toml"

        # TODO: Add better validation/errors for loading chipflow.toml
        with open(config_file, mode="rb") as fp:
            self.config = tomli.load(fp)

    def _load(self, loader_name):
        try:
            module_loc = self.config["chipflow"]["loader_module"]

            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Could not locate module, {module_loc}.") from error

        if (not hasattr(module, loader_name)):
            raise ChipFlowError(f"Loader module is missing loader. module={module_loc}, loader={loader_name}")

        return getattr(module, loader_name)(self.config)

    def _load_design_module(self):
        return self._load_module(self.config["chipflow"]["design_module"])

    def _load_module(self, module_loc):
        try:
            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Could not load module, {module_loc}.") from error

        return module


if __name__ == '__main__':
    Main().run()
