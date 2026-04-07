from glob import glob
import os
import sys

# Prefer the system setuptools shipped with ROS/Ubuntu over a newer user-local
# setuptools, which changes `setup.py develop` semantics in a way that breaks
# ament_python/colcon editable installs.
sys.path = [path for path in sys.path if "/.local/lib/python" not in path]

from setuptools import setup
from setuptools.command.develop import develop as _develop


package_name = "arm_teleop"

if "develop" in sys.argv:
    for flag in ("--uninstall", "--editable"):
        if flag in sys.argv:
            sys.argv.remove(flag)
    if "--build-directory" in sys.argv:
        idx = sys.argv.index("--build-directory")
        del sys.argv[idx : idx + 2]


class DevelopCommand(_develop):
    user_options = _develop.user_options + [
        ("script-dir=", None, "compat no-op for colcon"),
        ("install-scripts=", None, "compat no-op for colcon"),
    ]

    def initialize_options(self):
        super().initialize_options()
        self.script_dir = None
        self.install_scripts = None


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adip",
    maintainer_email="adipdas11@gmail.com",
    description="Webcam hand tracking and EXOTica teleoperation package.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "webcam_hand_tracker = arm_teleop.webcam_hand_tracker:main",
            "exotica_arm_teleop = arm_teleop.exotica_arm_teleop:main",
            "wait_for_exotica_ready = arm_teleop.wait_for_exotica_ready:main",
        ],
    },
    cmdclass={"develop": DevelopCommand},
)
