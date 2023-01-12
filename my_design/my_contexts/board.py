# SPDX-License-Identifier: BSD-2-Clause
from ..design import MySoC
from amaranth_boards.ulx3s import ULX3S_85F_Platform
from chipflow_lib.contexts.board import BoardContext


class MyBoardContext(BoardContext):
    def __init__(self):

        platform = ULX3S_85F_Platform()
        platform.chipflow_context = "board_ulx3s"

        super().__init__(platform)

    def build(self):
        my_design = MySoC()

        self.platform.build(my_design, do_program=False)
