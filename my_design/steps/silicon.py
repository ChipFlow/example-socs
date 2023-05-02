# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.steps.silicon import SiliconStep
from ..design import MySoC


class MySiliconStep(SiliconStep):
    def build(self):
        my_design = MySoC()

        self.platform.build(my_design, name="my_design")
