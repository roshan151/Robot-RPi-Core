from setuptools import find_packages, setup

package_name = "robot_voice"

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
    description="Gemini Live voice agent, driving the robot over ROS actions.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_node = robot_voice.voice_node:main",
        ],
    },
)
