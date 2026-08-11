#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for the local Nav2 point-cloud parameter overlay."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml


LAUNCH_DIR = Path(__file__).parents[1] / "bringup" / "launch"
if str(LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(LAUNCH_DIR))

from _nav2_perception import (  # noqa: E402
    add_hardware_navigation_limits,
    add_local_point_cloud_sources,
    add_robot_footprint,
    footprint_from_profile,
    write_point_cloud_nav2_parameters,
)


def base_parameters(layer_name="voxel_layer"):
    return {
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "plugins": [layer_name, "inflation_layer"],
                    layer_name: {
                        "plugin": "nav2_costmap_2d::VoxelLayer",
                        "observation_sources": "scan",
                        "scan": {"topic": "/scan", "data_type": "LaserScan"},
                    },
                }
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "plugins": ["obstacle_layer"],
                    "obstacle_layer": {
                        "observation_sources": "scan",
                        "scan": {"topic": "/scan"},
                    },
                }
            }
        },
    }


def hardware_parameters():
    parameters = base_parameters()
    parameters.update(
        {
            "amcl": {"ros__parameters": {}},
            "controller_server": {
                "ros__parameters": {
                    "progress_checker": {},
                    "general_goal_checker": {},
                    "FollowPath": {},
                }
            },
            "behavior_server": {"ros__parameters": {}},
            "velocity_smoother": {"ros__parameters": {}},
        }
    )
    return parameters


def test_hardware_limits_are_conservative_and_do_not_modify_input():
    parameters = hardware_parameters()
    original = deepcopy(parameters)

    configured = add_hardware_navigation_limits(parameters)

    assert parameters == original
    amcl = configured["amcl"]["ros__parameters"]
    controller = configured["controller_server"]["ros__parameters"]
    smoother = configured["velocity_smoother"]["ros__parameters"]
    assert amcl["laser_max_range"] == 5.0
    assert amcl["update_min_d"] == 0.05
    assert controller["controller_frequency"] == 10.0
    assert controller["FollowPath"]["max_vel_x"] == 0.20
    assert controller["FollowPath"]["max_vel_theta"] == 0.35
    assert controller["progress_checker"]["required_movement_radius"] == 0.05
    assert smoother["max_velocity"] == [0.20, 0.0, 0.35]


def test_hardware_speed_arguments_override_defaults():
    configured = add_hardware_navigation_limits(
        hardware_parameters(),
        max_linear_speed="0.16",
        max_angular_speed="0.45",
    )
    controller = configured["controller_server"]["ros__parameters"]
    smoother = configured["velocity_smoother"]["ros__parameters"]
    assert controller["FollowPath"]["max_vel_x"] == 0.16
    assert controller["FollowPath"]["max_vel_theta"] == 0.45
    assert smoother["max_velocity"] == [0.16, 0.0, 0.45]


@pytest.mark.parametrize(
    ("linear", "angular"),
    [(0.0, 0.35), (0.31, 0.35), (0.20, 0.0), (0.20, 0.61), ("bad", 0.35)],
)
def test_hardware_speed_arguments_reject_unsafe_values(linear, angular):
    with pytest.raises(ValueError):
        add_hardware_navigation_limits(
            hardware_parameters(),
            max_linear_speed=linear,
            max_angular_speed=angular,
        )


@pytest.mark.parametrize(
    "missing_node",
    ["amcl", "controller_server", "behavior_server", "velocity_smoother"],
)
def test_hardware_limits_reject_incomplete_nav2_file(missing_node):
    parameters = hardware_parameters()
    parameters.pop(missing_node)
    with pytest.raises(ValueError):
        add_hardware_navigation_limits(parameters)


def test_overlay_marks_filtered_cloud_and_clears_with_raw_cloud():
    configured = add_local_point_cloud_sources(base_parameters())
    layer = configured["local_costmap"]["local_costmap"]["ros__parameters"][
        "voxel_layer"
    ]

    assert layer["observation_sources"] == "scan depth_mark depth_clear"
    assert layer["scan"]["topic"] == "/scan"
    assert layer["depth_mark"]["topic"] == "/camera/depth/points_filtered"
    assert layer["depth_mark"]["data_type"] == "PointCloud2"
    assert layer["depth_mark"]["marking"] is True
    assert layer["depth_mark"]["clearing"] is False
    assert layer["depth_clear"]["topic"] == "/camera/depth/points"
    assert layer["depth_clear"]["marking"] is False
    assert layer["depth_clear"]["clearing"] is True


def test_overlay_does_not_modify_input_or_global_costmap():
    parameters = base_parameters()
    original = deepcopy(parameters)
    configured = add_local_point_cloud_sources(parameters)

    assert parameters == original
    assert configured["global_costmap"] == original["global_costmap"]


def test_overlay_is_idempotent_and_accepts_list_source_form():
    parameters = base_parameters()
    layer = parameters["local_costmap"]["local_costmap"]["ros__parameters"][
        "voxel_layer"
    ]
    layer["observation_sources"] = ["scan", "depth_mark"]

    configured = add_local_point_cloud_sources(parameters)
    configured = add_local_point_cloud_sources(configured)
    names = configured["local_costmap"]["local_costmap"]["ros__parameters"][
        "voxel_layer"
    ]["observation_sources"].split()
    assert names == ["scan", "depth_mark", "depth_clear"]


@pytest.mark.parametrize(
    "parameters",
    [
        {},
        {"local_costmap": {}},
        {
            "local_costmap": {
                "local_costmap": {"ros__parameters": {"plugins": []}}
            }
        },
    ],
)
def test_overlay_rejects_incomplete_local_costmap(parameters):
    with pytest.raises(ValueError):
        add_local_point_cloud_sources(parameters)


def test_measured_profile_sets_both_costmap_footprints(tmp_path):
    profile_path = tmp_path / "robot_profile.yaml"
    profile_path.write_text(
        "xacro:\n  overall_length: 0.35\n  overall_width: 0.385\n",
        encoding="utf-8",
    )
    footprint = footprint_from_profile(profile_path)
    assert footprint == (
        "[[0.175000, 0.192500], [0.175000, -0.192500], "
        "[-0.175000, -0.192500], [-0.175000, 0.192500]]"
    )

    parameters = base_parameters()
    for costmap_name in ("local_costmap", "global_costmap"):
        parameters[costmap_name][costmap_name]["ros__parameters"][
            "robot_radius"
        ] = 0.22
    configured = add_robot_footprint(parameters, footprint)
    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = configured[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["footprint"] == footprint
        assert "robot_radius" not in costmap


def test_geometry_overlay_is_written_when_point_cloud_is_disabled(tmp_path):
    source_path = tmp_path / "nav2.yaml"
    source_path.write_text(yaml.safe_dump(base_parameters()), encoding="utf-8")
    profile_path = tmp_path / "robot_profile.yaml"
    profile_path.write_text(
        "xacro:\n  overall_length: 0.35\n  overall_width: 0.385\n",
        encoding="utf-8",
    )

    output_path = write_point_cloud_nav2_parameters(
        source_path,
        profile_file=profile_path,
        enable_point_cloud=False,
    )
    configured = yaml.safe_load(Path(output_path).read_text(encoding="utf-8"))
    local = configured["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["footprint"].startswith("[[0.175000, 0.192500]")
    assert "depth_mark" not in local["voxel_layer"]
