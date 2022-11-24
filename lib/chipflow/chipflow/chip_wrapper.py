# SPDX-License-Identifier: BSD-2-Clause
import importlib

from amaranth import *
from amaranth.build import *
from amaranth.lib.cdc import ResetSynchronizer
from amaranth_boards.ulx3s import *
from amaranth_boards.ulx3s import *

from amaranth_orchard.memory.spimemio import QSPIPins
from amaranth_orchard.base.gpio import GPIOPins
from amaranth_orchard.io.uart import UARTPins
from amaranth_orchard.memory.hyperram import HyperRAMPins


class ChipFlowError(Exception):
    pass


class SoCWrapper(Elaboratable):
    """
    This wrapper provides helper methods to access ChipFlow tools.
    """

    def get_chipflow_context(self, platform):
        if (not hasattr(platform, "chipflow_context")):
            raise ChipFlowError("Platform obj didn't have `chipflow_context` attribute.")
        return platform.chipflow_context

    def load_provider(self, platform, class_name):
        """Require a ChipFlow provider for the platform's context."""
        context = self.get_chipflow_context(platform)

        try:
            module_loc = f"chipflow.providers.{context}"
            module = importlib.import_module(module_loc)
        except ModuleNotFoundError as error:
            raise ChipFlowError("Provider module couldn't be loaded for context, {module_loc}.") from error

        if (not hasattr(module, class_name)):
            raise ChipFlowError(f"Provider module missing class. {module_loc}, {class_name}")

        return getattr(module, class_name)(platform)
