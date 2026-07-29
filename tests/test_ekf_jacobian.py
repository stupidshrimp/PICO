"""Regression guard for the attitude-EKF analytic Jacobians (F and coupled H).

Compiles and runs the host-side finite-difference verification in
flight_controller/tests/ekf_jacobian_test.cpp, which checks that the hand-derived
state-transition Jacobian F (Main_bCalcJacobianF) and the legacy 3-axis
measurement Jacobian H (Main_bCalcJacobianH) match a central finite difference of
the state propagation f and measurement model h that Main.ino integrates.

A wrong sign or term in F/H does not crash or NaN in flight -- it silently
degrades the Kalman gain -- so this guards against that class of error in CI.
The decoupled yaw Jacobian is covered separately by test_ekf_decouple_mag.py.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "ekf_jacobian_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_ekf_jacobians(tmp_path):
    binary = os.path.join(tmp_path, "ekf_jacobian_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
