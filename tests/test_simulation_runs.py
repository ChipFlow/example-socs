# SPDX-License-Identifier: BSD-2-Clause
import os
import signal
import subprocess
import time
import unittest


class TestAPI(unittest.TestCase):
    def test_simulation_builds_and_boops_kernel(self):
        project_path = os.path.abspath(
            os.path.dirname(__file__) + "/../"
        )

        build_command = f"cd {project_path} && \
            make sim-build && \
            make software-build"

        subprocess.run(build_command, shell=True, check=True)

        run_command = f"cd {project_path} && \
            make sim-run"

        with subprocess.Popen(run_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              shell=True, encoding="utf-8", start_new_session=True) as process:
            output_lines = []
            start_time = time.time()
            max_seconds = 60
            expected_text = "Initialised!"
            extra_msg = ""

            while True:
                line = process.stdout.readline()
                if not line:
                    break
                output_lines.append(line)

                if expected_text in line:
                    # Leave the loop early, we do our assertion after to make the failure case look nicer.
                    break
                if (time.time() - start_time) > max_seconds:
                    extra_msg = f"Timeout of {max_seconds}s reached."
                    break

            assert expected_text in "".join(
                output_lines), f"We found our expected text in the simulation output. {extra_msg}"

            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
