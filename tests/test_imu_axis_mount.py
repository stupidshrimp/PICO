"""Regression guard for the IMU -> aircraft/EKF axis alignment.

Compiles and runs flight_controller/tests/imu_axis_mount_test.cpp, which rebuilds
the imuAxesToBody() transform from imu_axis_mount.h and proves it is a proper
rotation (determinant +1), i.e. NOT the determinant -1 reflection (the old bare
X/Y swap) that swapped roll/pitch, reversed/offset the heading, and made the
attitude solution drift. Both valid mount orientations (+1 / -1) are checked.

Skipped automatically when no C++ compiler is available.
"""
import os
import shutil
import subprocess

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FC_DIR = os.path.join(_REPO_ROOT, "flight_controller")
_SRC = os.path.join(_FC_DIR, "tests", "imu_axis_mount_test.cpp")


def _compiler():
    for cc in ("c++", "g++", "clang++"):
        path = shutil.which(cc)
        if path:
            return path
    return None


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler available")
@pytest.mark.parametrize("rotation", [-1, 1])
def test_imu_axis_mount_is_proper_rotation(tmp_path, rotation):
    binary = os.path.join(tmp_path, f"imu_axis_mount_test_{rotation}")
    compile_cmd = [
        _compiler(), "-std=c++17", "-I", _FC_DIR, "-O2",
        "-Wall", "-Wextra", "-Werror",
        f"-DFC_IMU_MOUNT_YAW_ROTATION={rotation}",
        "-o", binary, _SRC,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, f"compile failed:\n{compiled.stderr}"

    run = subprocess.run([binary], capture_output=True, text=True)
    assert run.returncode == 0, f"test reported failures:\n{run.stdout}\n{run.stderr}"
    assert "ALL TESTS PASSED" in run.stdout, run.stdout
