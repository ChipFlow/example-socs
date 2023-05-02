# SPDX-License-Identifier: BSD-2-Clause

from amaranth_boards.ulx3s import ULX3S_85F_Platform
from chipflow_lib.steps.board import BoardStep
from ..design import MySoC


class MyBoardStep(BoardStep):
    def __init__(self, config):

        platform = ULX3S_85F_Platform()
        platform.chipflow_context = "board_ulx3s"

        super().__init__(config, platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design, do_program=False)
