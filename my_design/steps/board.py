# SPDX-License-Identifier: BSD-2-Clause

from amaranth_boards.ulx3s import ULX3S_85F_Platform
from chipflow_lib.steps.board import BoardStep
from chipflow_lib.providers import board_ulx3s as board_ulx3s_providers
from ..design import MySoC


class MyBoardStep(BoardStep):
    def __init__(self, config):

        platform = ULX3S_85F_Platform()
        platform.providers = board_ulx3s_providers

        super().__init__(config, platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design, do_program=False)
