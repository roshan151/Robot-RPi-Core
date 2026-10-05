from setuptools import find_packages, setup

package_name = "robot_drivetrain"

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
    description="Drive/Turn action servers and the e-stop service.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "drivetrain_node = robot_drivetrain.drivetrain_node:main",
        ],
    },
)
