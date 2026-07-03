"""Regression guard for the body-frame handedness fix.

Compiles and runs flight_controller/tests/frame_consistency_test.cpp, which
builds the real firmware matrix library + EKF class plus the flight TRIAD
header and proves that:
  * the new proper (det +1) IMU->body mapping produces static roll/pitch/yaw
    outputs numerically identical to the previous pipeline's converged static
    outputs (the operator-verified conventions are preserved), and
  * through the real EKF, the new mapping tracks a tumbling truth with
    near-zero innovation while the old bare X<->Y swap (a det -1 reflection)
    exhibits the gyro-vs-accel/mag prediction/correction fight that caused the
    attitude drift and offset.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "frame_consistency_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_frame_consistency(tmp_path):
    binary = os.path.join(tmp_path, "frame_consistency_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
