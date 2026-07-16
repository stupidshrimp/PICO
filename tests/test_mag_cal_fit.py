"""Regression guard for the shared magnetometer-calibration fit.

Compiles and runs the host-side verification in
flight_controller/tests/mag_cal_fit_test.cpp, which exercises the exact
header (flight_controller/mag_cal_fit.h) both the bench MAGCAL helper and the
in-field runtime calibration compute their constants from, proving that:
  * a known hard-iron offset and diagonal soft-iron scaling are recovered
    from full-coverage rotation data,
  * the fitted constants restore a direction-independent field magnitude,
  * short runs and insufficient axis coverage are rejected with the right
    status instead of producing constants.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "mag_cal_fit_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_mag_cal_fit(tmp_path):
    binary = os.path.join(tmp_path, "mag_cal_fit_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
