from glob import glob
import os

from setuptools import setup


package_name = "arm_teleop"


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
)
