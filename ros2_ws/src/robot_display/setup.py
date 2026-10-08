from setuptools import find_packages, setup

package_name = "robot_display"

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
    description="Status OLED and power-dip watcher.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "display_node = robot_display.display_node:main",
        ],
    },
)
