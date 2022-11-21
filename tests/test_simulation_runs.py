import os
import time
import unittest
from subprocess import run, Popen, PIPE, STDOUT


class TestAPI(unittest.TestCase):
    def test_simulation_builds_and_boops_kernel(self):
        project_path = os.path.abspath(
            os.path.dirname(__file__) + "/../"
        )

        build_command = f"cd {project_path} && \
            make sim-build && \
            make software-build"

        run(build_command, shell=True, check=True)

        run_command = f"cd {project_path} && \
            make sim-run"

        process = Popen(run_command, stdout=PIPE, stderr=STDOUT, shell=True)

        output_lines = []
        start_time = time.time()
        max_seconds = 60
        expected_text = "about to boop the kernel"
        extra_msg = ""

        while True:
            binary_line = process.stdout.readline()
            if not binary_line:
                break

            line = binary_line.decode("utf-8")
            output_lines.append(line)

            if expected_text in line:
                # Leave the loop early, we do our assertion after to make the failure case look nicer.
                break
            if (time.time() - start_time) > max_seconds:
                extra_msg = f"Timeout of {max_seconds}s reached."
                break

        assert expected_text in "".join(
            output_lines), f"We found our expected text in the simulation output. {extra_msg}"
