# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest
import subprocess
from pathlib import Path


def get_bin_false():
    for path in ["/bin/false", "/usr/bin/false"]:
        if Path(path).exists():
            return path
    raise Exception("Coudln't locate /bin/false")


class TestBoard(unittest.TestCase):
    def test_build_runs(self):
        project_path = os.path.dirname(os.path.dirname(__file__))
        subprocess.check_call(
            ["make", "-C", project_path, "board-build"],
            env={**os.environ, "OPENFPGALOADER": get_bin_false()})
        assert os.path.exists(project_path + "/build/top.bit")
