# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest
import subprocess


class TestBoard(unittest.TestCase):
    def test_build_runs(self):
        project_path = os.path.dirname(os.path.dirname(__file__))
        subprocess.check_call(
            ["make", "-C", project_path, "board-build"],
            env={**os.environ, "OPENFPGALOADER": "/bin/false"})
        assert os.path.exists(project_path + "/build/top.bit")
