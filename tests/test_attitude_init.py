"""Regression guard for the boot-time coarse attitude alignment (TRIAD).

Compiles and runs flight_controller/tests/attitude_init_test.cpp, which builds
the exact flight header (flight_controller/attitude_init.h) and proves that:
  * TRIAD recovers random attitudes in the EKF's quaternion parameterization,
  * magnetometer disturbance can only rotate the alignment about gravity
    (yaw), never tilt roll/pitch,
  * degenerate accel/mag observations are rejected instead of producing a
    garbage boot attitude.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "attitude_init_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_attitude_init_triad(tmp_path):
    binary = os.path.join(tmp_path, "attitude_init_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
