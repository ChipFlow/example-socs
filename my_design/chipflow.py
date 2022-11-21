from .my_contexts.board import MyBoardContext
from .my_contexts.silicon import MySiliconContext
from .my_contexts.sim import MySimContext


def load_sim_context(chipflow_config):
    """Load our sim context for ChipFlow"""
    return MySimContext()


def load_silicon_context(chipflow_config):
    """Load our silicon context for ChipFlow"""
    return MySiliconContext()


def load_board_context(chipflow_config):
    """Load our board context for ChipFlow"""
    return MyBoardContext()
