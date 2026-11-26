import os
from glob import glob

from setuptools import find_packages, setup


package_name = "webots_autoware_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "map"), glob("map/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Minimal Webots adapter for Autoware 2.0.0",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vehicle_adapter = webots_autoware_demo.vehicle_adapter:main",
            "straight_trajectory_publisher = webots_autoware_demo.straight_trajectory_publisher:main",
            "planner_test_inputs = webots_autoware_demo.planner_test_inputs:main",
            "trajectory_probe = webots_autoware_demo.trajectory_probe:main",
            "n_command_delay = webots_autoware_demo.n_command_delay:main",
            "n_command_evaluator = webots_autoware_demo.n_command_evaluator:main",
            "safety_mode_bootstrap = webots_autoware_demo.safety_mode_bootstrap:main",
            ],
    },
)
