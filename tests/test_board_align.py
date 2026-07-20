"""Regression guard for the board-alignment (level) trim.

Compiles and runs flight_controller/tests/board_align_test.cpp, which builds the
exact flight headers (flight_controller/board_align.h and attitude_init.h) and
proves that:
  * the trim rotation is a proper rotation (orthonormal, det +1) equal to the
    inverse of the offset attitude, and the identity for a zero offset,
  * an IMU mounted with the operator's reported offset reports that offset
    uncorrected on a level airframe and reads level once the trim is applied,
  * for random airframe attitudes and mount offsets, the trimmed accel/mag
    stream fed through the flight TRIAD init recovers the true airframe
    attitude, with the gyro rate transformed by the same rotation.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "board_align_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_board_align_trim(tmp_path):
    binary = os.path.join(tmp_path, "board_align_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
