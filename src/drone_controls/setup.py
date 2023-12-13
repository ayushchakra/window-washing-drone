from setuptools import find_packages, setup
import os
from glob import glob

package_name = "drone_controls"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="achakraborty",
    maintainer_email="achakraborty@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_estimation = drone_controls.state_estimation:main",
            "command_drone = drone_controls.fly:main",
            "teleop = drone_controls.teleop:main",
        ],
    },
)
