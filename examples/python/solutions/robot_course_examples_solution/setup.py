from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_course_examples_solution"

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
    maintainer="Instructor",
    maintainer_email="instructor@example.com",
    description="Reference solution for Lab 5 of the Studica ROS 2 course.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_reporter_solution = "
            "robot_course_examples_solution.sensor_reporter:main",
        ],
    },
)
