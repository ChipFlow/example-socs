# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.contexts.sim import SimContext
from chipflow_lib.sim_platform import SimPlatform
from ..design import MySoC
from ..sim import doit_build


class MySimContext(SimContext):
    doit_build_module = doit_build

    def __init__(self, config):
        platform = SimPlatform()

        super().__init__(config, platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design)
        self.doit_build()
