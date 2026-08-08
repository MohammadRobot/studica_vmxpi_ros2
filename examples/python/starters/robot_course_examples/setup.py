from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_course_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="Student TODO package for the Studica ROS 2 classroom course.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_reporter = robot_course_examples.sensor_reporter:main",
        ],
    },
)
