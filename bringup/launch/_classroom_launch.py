# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Small helpers that keep nested advanced arguments out of beginner help output."""

from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def deferred_include(package_name, launch_file, launch_arguments):
    """Include another launch at runtime without expanding its arguments in --show-args."""

    def _include(context, *args, **kwargs):
        del context, args, kwargs
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), "launch", launch_file]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )
        ]

    return OpaqueFunction(function=_include)
