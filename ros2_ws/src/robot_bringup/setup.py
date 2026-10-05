import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Roshan Tiwari",
    maintainer_email="roshan.151tiwari@gmail.com",
    description="Launch files, parameters and the one-process node host.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bringup = robot_bringup.bringup:main",
        ],
    },
)
