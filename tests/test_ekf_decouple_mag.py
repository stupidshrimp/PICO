"""Regression guard for the magnetometer/roll-pitch decoupling EKF model.

Compiles and runs the host-side numerical verification in
flight_controller/tests/ekf_decouple_mag_test.cpp, which builds the real
firmware matrix library + EKF class and proves that:
  * the analytic yaw-measurement Jacobian matches finite difference,
  * the tilt-compensated heading innovation has the correct sign/magnitude,
  * a magnetometer disturbance moves yaw but not roll/pitch (the decoupling).

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "ekf_decouple_mag_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_ekf_decouple_mag_model(tmp_path):
    binary = os.path.join(tmp_path, "ekf_decouple_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
