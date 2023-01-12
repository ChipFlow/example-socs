# SPDX-License-Identifier: BSD-2-Clause
from ..design import MySoC
from chipflow_lib.contexts.sim import SimContext
from chipflow_lib.sim_platform import SimPlatform


class MySimContext(SimContext):
    def __init__(self):
        platform = SimPlatform()

        super().__init__(platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design)
