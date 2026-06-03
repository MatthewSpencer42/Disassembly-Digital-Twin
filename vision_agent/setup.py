import os
import sys
from glob import glob

# Prefer the system setuptools shipped with ROS/Ubuntu over a newer user-local
# setuptools, which changes `setup.py develop` semantics in a way that breaks
# ament_python/colcon editable installs.
sys.path = [path for path in sys.path if "/.local/lib/python" not in path]

from setuptools import setup
from setuptools.command.develop import develop as _develop

package_name = 'vision_agent'

# Colcon/ament_python may invoke `setup.py develop --uninstall` during
# incremental rebuilds. Newer setuptools drops that flag, so strip it here
# to keep rebuilds working in mixed system/user Python environments.
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
    version='0.0.1',
    # !!! CRITICAL: Include the sub-package here !!!
    packages=[
        package_name,
        'vision_agent.agents',
        'vision_agent.agents.yolo',
        'vision_agent.agents.rfdetr',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        
        # --- NEW ADDITION: Install the launch files ---
        # This tells colcon to copy files from the 'launch' folder to the install directory
        ('share/' + package_name + '/launch', ['launch/start_vision.launch.py',
                                               'launch/system_startup.launch.py',
                                               'launch/orbbec_camera.launch.py',
                                               ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adip',
    maintainer_email='adip@todo.todo',
    description='PhD Vision System Agents',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start_vision = vision_agent.agent_node:main',
            'start_vision_rtdetr = vision_agent.agent_node_v2:main',
            'start_vision_v2 = vision_agent.agent_node_v2:main',
            'start_vision_split = vision_agent.split_runtime:main',
            'start_vision_global = vision_agent.global_node:main',
            'start_vision_local = vision_agent.local_node:main',
            'start_vision_classifier = vision_agent.classifier_node:main',
            'start_vision_dashboard = vision_agent.dashboard_node:main',
            'dashboard_window = vision_agent.dashboard_window:main',
            'detect_workspace = vision_agent.workspace_detector:main',
            'crop_camera_streams = vision_agent.camera_crop_republisher:main',
        ],
    },
    cmdclass={"develop": DevelopCommand},
)
