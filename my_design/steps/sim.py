# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.steps.sim import SimStep
from chipflow_lib.platforms.sim import SimPlatform
from ..design import MySoC
from ..sim import doit_build


class MySimStep(SimStep):
    doit_build_module = doit_build

    def __init__(self, config):
        platform = SimPlatform()

        super().__init__(config, platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design)
        self.doit_build()
