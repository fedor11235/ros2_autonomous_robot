import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fedor",
    maintainer_email="fedor11235@users.noreply.github.com",
    description="Autonomous waypoint navigation with reactive obstacle avoidance and GPS.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_navigator = robot_navigation.waypoint_navigator:main",
            "gps_localizer = robot_navigation.gps_localizer:main",
        ],
    },
)
