"""Regression guard for the airspeed-hold throttle controller.

Compiles and runs flight_controller/tests/auto_throttle_test.cpp, which builds
the exact flight header (flight_controller/auto_throttle.h) with the exact
flight gains and proves that:
  * with no airflow and a non-zero target the loop drives the throttle to 100%
    (the bench diagnostic: a motor that instead holds one static speed proves
    the loop is not running, rather than being mistuned),
  * airspeed above the target backs the throttle off, and the loop settles on a
    commanded airspeed against a plant without parking at a clamp,
  * a short RC gap keeps the commanded trim (the velocity form's only state) so
    the loop resumes instead of re-ramping from idle, while a genuine link loss
    drops it,
  * the derivative is differentiated against the pitot sample interval, so a
    60 Hz sensor read at a 125 Hz control rate yields a ripple-free rate
    estimate even when reads are dropped,
  * deadband, output limit, command clamping, bumpless transfer from manual
    throttle, and the stale-airspeed decay rate.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "auto_throttle_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_auto_throttle_controller(tmp_path):
    binary = os.path.join(tmp_path, "auto_throttle_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
