import os
from glob import glob
from setuptools import find_packages, setup

package_name = "disassembly_skill"


def package_files(directory):
    paths = []
    for path, _directories, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


config_files = [path for path in glob("config/*") if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), config_files),
        (
            os.path.join("share", package_name, "config", "device_configs"),
            package_files("config/device_configs"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adip',
    maintainer_email='adipdas11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "motion_backend = disassembly_skill.motion_backend:main",
            "debug_feed_republisher = disassembly_skill.debug_feed_republisher:main",
            "object_hold_skill = disassembly_skill.object_hold_skill:main",
            "object_flip_skill = disassembly_skill.object_flip_skill:main",
            "object_flip_drop_skill = disassembly_skill.object_flip_drop_skill:main",
            "object_pickup_skill = disassembly_skill.object_pickup_skill:main",
            "unscrew_skill = disassembly_skill.unscrew_skill:main",
            "test_exotica_planner = disassembly_skill.test_exotica_planner:main",
            "master_agent = disassembly_skill.master_agent:main",
            "groq_master_agent = disassembly_skill.groq_master_agent:main",
            "config_runner = disassembly_skill.config_runner:main",
            "device_config_builder = disassembly_skill.config_builder.config_builder_app:main",
        ],
    },
)
