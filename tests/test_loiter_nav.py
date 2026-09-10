"""Firmware-side loiter mode (flight_controller/loiter_nav.h).

Compiles and runs flight_controller/tests/loiter_test.cpp, which builds the
real flight header and proves that:
  * only an explicit high CH10 requests the orbit, so a centred, failsafed or
    transient channel can never latch an autonomous mode;
  * the engage gate is a true conjunction -- exactly one of its 32 condition
    combinations may fly, so no future edit can quietly drop a requirement;
  * the commanded bank matches the configured angle and sign, sits well inside
    the FBW hard limit, and stays clamped there even if the constant is edited
    past the envelope.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "loiter_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
def test_loiter_nav(tmp_path):
    binary = os.path.join(tmp_path, "loiter_test")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror", "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
