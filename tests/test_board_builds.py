# SPDX-License-Identifier: BSD-2-Clause
import os
import unittest
import subprocess


class TestBoard(unittest.TestCase):
    def test_build_runs(self):
        project_path = os.path.abspath(
            os.path.dirname(__file__) + "/../"
        )

        # We make a fake executable openFPGALoader so Amaranth doesn't complain.
        # We don't need it for the build.
        fake_file = f"{project_path}/openFPGALoader"
        command = f"cd {project_path} && \
            export PATH={project_path}:$PATH && \
            cp README.md {fake_file} && \
            chmod +x {fake_file} && \
            make board-build"

        try:
            subprocess.run(command, shell=True, check=True)
        finally:
            if os.path.exists(fake_file):
                os.remove(fake_file)

        assert os.path.exists(project_path + "/build/top.bit") is True
