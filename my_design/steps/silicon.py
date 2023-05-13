# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.steps.silicon import SiliconStep
from ..design import MySoC


class MySiliconStep(SiliconStep):
    def prepare(self):
        return self.platform.build(MySoC(), name="my_design")
