from setuptools import find_packages, setup

package_name = "robot_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Roshan Tiwari",
    maintainer_email="roshan.151tiwari@gmail.com",
    description="Perception (camera, tracker, REST), gimbal and vision behaviors.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # Its own process: it makes network calls, which hang for seconds.
            "perception_node = robot_vision.perception_node:main",
            # These two normally run inside the `robot` process (bringup.py);
            # standalone entry points are for bench testing.
            "gimbal_node = robot_vision.gimbal_node:main",
            "behavior_node = robot_vision.behavior_node:main",
        ],
    },
)
