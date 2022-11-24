# SPDX-License-Identifier: BSD-2-Clause
import os
import unittest
import subprocess


class TestRtlil(unittest.TestCase):
    def test_build_runs(self):
        project_path = os.path.abspath(
            os.path.dirname(__file__) + "/../"
        )

        command = f"cd {project_path} && \
            make silicon-rtlil"

        subprocess.run(command, shell=True, check=True)

        assert os.path.exists(project_path + "/build/my_design.rtlil") is True
