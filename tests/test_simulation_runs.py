import os
import time
import unittest
import pytest
from subprocess import Popen, PIPE, STDOUT


class TestAPI(unittest.TestCase):
    def test_built_simulation_runs_and_boops_kernel(self):
        sim_path = os.path.abspath(
            os.path.dirname(__file__) + "/../my_design/sim"
        )
        sim_build_path = "/build/sim_soc"

        command = f"cd {sim_path} && .{sim_build_path}"

        assert True == os.path.exists(sim_path + sim_build_path), "Simulation binary has been build ready for us to run. If this fails, you might need to run the build before this test."

        process = Popen(command, stdout=PIPE, stderr=STDOUT, shell=True)

        output_lines = []
        start_time = time.time()
        max_seconds = 30
        expected_text = "about to boop the kernel"

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
                pytest.fail(
                    f"Timeout of {max_seconds}s reached, and we didn't find our expected text.")
                break

        assert expected_text in "".join(
            output_lines), "We found our expected text in the simulation output."
