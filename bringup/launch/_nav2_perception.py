# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Add the project depth-camera sources to a complete Nav2 parameter file."""

from copy import deepcopy
import math
from pathlib import Path
import tempfile

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import yaml


FILTERED_POINT_CLOUD_TOPIC = "/camera/depth/points_filtered"
RAW_POINT_CLOUD_TOPIC = "/camera/depth/points"
MARKING_SOURCE = "depth_mark"
CLEARING_SOURCE = "depth_clear"
DEFAULT_HARDWARE_MAX_LINEAR_SPEED = 0.2
DEFAULT_HARDWARE_MAX_ANGULAR_SPEED = 0.35
HARDWARE_MAX_LINEAR_SPEED_LIMIT = 0.30
HARDWARE_MAX_ANGULAR_SPEED_LIMIT = 0.60


def _bounded_speed(value, name, upper_limit):
    """Return a finite positive speed no greater than the validated limit."""
    try:
        speed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric, got {value!r}") from error
    if not math.isfinite(speed) or speed <= 0.0 or speed > upper_limit:
        raise ValueError(f"{name} must be greater than 0 and at most {upper_limit}")
    return speed


def _node_parameters(parameters, node_name):
    """Return one Nav2 node's ros__parameters mapping with a clear error."""
    try:
        node_parameters = parameters[node_name]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"Nav2 parameters must contain {node_name}.ros__parameters"
        ) from error
    if not isinstance(node_parameters, dict):
        raise ValueError(f"{node_name} ros__parameters must be a mapping")
    return node_parameters


def add_hardware_navigation_limits(
    parameters,
    max_linear_speed=DEFAULT_HARDWARE_MAX_LINEAR_SPEED,
    max_angular_speed=DEFAULT_HARDWARE_MAX_ANGULAR_SPEED,
):
    """Return a copy with conservative real-robot localization and motion limits."""
    if not isinstance(parameters, dict):
        raise ValueError("Nav2 parameter file root must be a mapping")

    max_linear_speed = _bounded_speed(
        max_linear_speed,
        "hardware_max_linear_speed",
        HARDWARE_MAX_LINEAR_SPEED_LIMIT,
    )
    max_angular_speed = _bounded_speed(
        max_angular_speed,
        "hardware_max_angular_speed",
        HARDWARE_MAX_ANGULAR_SPEED_LIMIT,
    )
    configured = deepcopy(parameters)
    amcl = _node_parameters(configured, "amcl")
    controller = _node_parameters(configured, "controller_server")
    behavior = _node_parameters(configured, "behavior_server")
    velocity_smoother = _node_parameters(configured, "velocity_smoother")

    try:
        progress_checker = controller["progress_checker"]
        goal_checker = controller["general_goal_checker"]
        follow_path = controller["FollowPath"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "Nav2 controller_server needs progress_checker, "
            "general_goal_checker, and FollowPath mappings"
        ) from error
    if not all(
        isinstance(section, dict)
        for section in (progress_checker, goal_checker, follow_path)
    ):
        raise ValueError("Nav2 controller plugin parameters must be mappings")

    # The X2 is most reliable over its useful indoor range. Update AMCL at the
    # low speeds used by this high-grip 4WD platform instead of waiting for the
    # TurtleBot-oriented default travel thresholds.
    amcl.update(
        {
            "laser_min_range": 0.12,
            "laser_max_range": 5.0,
            "max_beams": 60,
            "update_min_d": 0.05,
            "update_min_a": 0.05,
        }
    )

    controller["controller_frequency"] = 10.0
    progress_checker.update(
        {"required_movement_radius": 0.05, "movement_time_allowance": 15.0}
    )
    goal_checker.update({"xy_goal_tolerance": 0.10, "yaw_goal_tolerance": 0.15})
    follow_path.update(
        {
            "min_vel_x": 0.0,
            "max_vel_x": max_linear_speed,
            "max_vel_theta": max_angular_speed,
            "min_speed_xy": 0.0,
            "max_speed_xy": max_linear_speed,
            "min_speed_theta": 0.0,
            "acc_lim_x": 0.18,
            "acc_lim_theta": 0.45,
            "decel_lim_x": -0.20,
            "decel_lim_theta": -0.50,
            "trans_stopped_velocity": 0.02,
        }
    )

    # Recovery turns must obey the same physical envelope as path following.
    behavior.update(
        {
            "max_rotational_vel": max_angular_speed,
            "min_rotational_vel": 0.08,
            "rotational_acc_lim": 0.45,
        }
    )
    velocity_smoother.update(
        {
            "smoothing_frequency": 10.0,
            "max_velocity": [max_linear_speed, 0.0, max_angular_speed],
            "min_velocity": [
                -min(max_linear_speed, 0.05),
                0.0,
                -max_angular_speed,
            ],
            "max_accel": [0.18, 0.0, 0.45],
            "max_decel": [-0.20, 0.0, -0.50],
            "velocity_timeout": 0.5,
        }
    )
    return configured


def _observation_source_names(value):
    """Return a stable list from Nav2's whitespace string or YAML list form."""
    if value is None:
        return []
    if isinstance(value, str):
        return value.split()
    if isinstance(value, list) and all(isinstance(item, str) for item in value):
        return value
    raise ValueError("local costmap observation_sources must be a string or string list")


def _local_obstacle_layer(parameters):
    """Find the configured local VoxelLayer or ObstacleLayer parameter mapping."""
    try:
        local_parameters = parameters["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "Nav2 parameters must contain "
            "local_costmap.local_costmap.ros__parameters"
        ) from error

    if not isinstance(local_parameters, dict):
        raise ValueError("local costmap ros__parameters must be a mapping")

    configured_plugins = local_parameters.get("plugins", [])
    if not isinstance(configured_plugins, list):
        raise ValueError("local costmap plugins must be a list")

    for preferred_name in ("voxel_layer", "obstacle_layer"):
        if preferred_name not in configured_plugins:
            continue
        layer = local_parameters.get(preferred_name)
        if isinstance(layer, dict):
            return layer

    for plugin_name in configured_plugins:
        layer = local_parameters.get(plugin_name)
        if not isinstance(layer, dict):
            continue
        plugin_type = str(layer.get("plugin", ""))
        if plugin_type.endswith(("VoxelLayer", "ObstacleLayer")):
            return layer

    raise ValueError(
        "local costmap needs a configured VoxelLayer or ObstacleLayer "
        "before camera sources can be added"
    )


def add_local_point_cloud_sources(parameters):
    """Return a copy with camera marking and clearing on the local costmap only."""
    if not isinstance(parameters, dict):
        raise ValueError("Nav2 parameter file root must be a mapping")

    configured = deepcopy(parameters)
    layer = _local_obstacle_layer(configured)
    source_names = _observation_source_names(layer.get("observation_sources"))
    for source_name in (MARKING_SOURCE, CLEARING_SOURCE):
        if source_name not in source_names:
            source_names.append(source_name)
    layer["observation_sources"] = " ".join(source_names)

    # Only the floor/body-filtered, base_link cloud is allowed to create costs.
    layer[MARKING_SOURCE] = {
        "topic": FILTERED_POINT_CLOUD_TOPIC,
        "data_type": "PointCloud2",
        "clearing": False,
        "marking": True,
        "min_obstacle_height": 0.04,
        "max_obstacle_height": 1.50,
        "obstacle_min_range": 0.15,
        "obstacle_max_range": 3.00,
        "raytrace_min_range": 0.0,
        "raytrace_max_range": 3.00,
        "observation_persistence": 0.0,
        "expected_update_rate": 0.0,
    }

    # Keep complete optical rays for clearing. The raw cloud never marks costs,
    # so floor returns cannot become obstacles.
    layer[CLEARING_SOURCE] = {
        "topic": RAW_POINT_CLOUD_TOPIC,
        "data_type": "PointCloud2",
        "clearing": True,
        "marking": False,
        "min_obstacle_height": -0.05,
        "max_obstacle_height": 2.00,
        "obstacle_min_range": 0.10,
        "obstacle_max_range": 3.00,
        "raytrace_min_range": 0.10,
        "raytrace_max_range": 3.00,
        "observation_persistence": 0.0,
        "expected_update_rate": 0.0,
    }
    return configured


def footprint_from_profile(profile_file):
    """Return a rectangular Nav2 footprint from the measured outer envelope."""
    profile_path = Path(profile_file).expanduser()
    if not profile_path.is_file():
        raise ValueError(f"robot profile file does not exist: {profile_path}")
    try:
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        geometry = profile["xacro"]
        overall_length = float(geometry["overall_length"])
        overall_width = float(geometry["overall_width"])
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(
            f"robot profile {profile_path} needs numeric xacro.overall_length "
            "and xacro.overall_width"
        ) from error
    if overall_length <= 0.0 or overall_width <= 0.0:
        raise ValueError("robot profile outer dimensions must be positive")

    half_length = overall_length / 2.0
    half_width = overall_width / 2.0
    return (
        f"[[{half_length:.6f}, {half_width:.6f}], "
        f"[{half_length:.6f}, {-half_width:.6f}], "
        f"[{-half_length:.6f}, {-half_width:.6f}], "
        f"[{-half_length:.6f}, {half_width:.6f}]]"
    )


def add_robot_footprint(parameters, footprint):
    """Return a copy with the same measured footprint on both Nav2 costmaps."""
    configured = deepcopy(parameters)
    for costmap_name in ("local_costmap", "global_costmap"):
        try:
            costmap_parameters = configured[costmap_name][costmap_name][
                "ros__parameters"
            ]
        except (KeyError, TypeError) as error:
            raise ValueError(
                f"Nav2 parameters must contain {costmap_name}.{costmap_name}."
                "ros__parameters"
            ) from error
        if not isinstance(costmap_parameters, dict):
            raise ValueError(f"{costmap_name} ros__parameters must be a mapping")
        costmap_parameters["footprint"] = footprint
        costmap_parameters.pop("robot_radius", None)
    return configured


def write_point_cloud_nav2_parameters(
    source_file,
    profile_file=None,
    enable_point_cloud=True,
    hardware_mode=False,
    hardware_max_linear_speed=DEFAULT_HARDWARE_MAX_LINEAR_SPEED,
    hardware_max_angular_speed=DEFAULT_HARDWARE_MAX_ANGULAR_SPEED,
):
    """Write a temporary full Nav2 YAML with geometry and runtime overlays."""
    source_path = Path(source_file).expanduser()
    if not source_path.is_file():
        raise ValueError(f"Nav2 parameter file does not exist: {source_path}")
    try:
        parameters = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"cannot parse Nav2 parameter file {source_path}: {error}") from error

    configured = deepcopy(parameters)
    if hardware_mode:
        configured = add_hardware_navigation_limits(
            configured,
            max_linear_speed=hardware_max_linear_speed,
            max_angular_speed=hardware_max_angular_speed,
        )
    if enable_point_cloud:
        configured = add_local_point_cloud_sources(configured)
    if profile_file:
        configured = add_robot_footprint(
            configured,
            footprint_from_profile(profile_file),
        )
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="studica_nav2_point_cloud_",
        suffix=".yaml",
        delete=False,
    ) as output:
        yaml.safe_dump(configured, output, sort_keys=False)
        return output.name


def _enabled(value):
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"use_point_cloud must be true or false, got {value!r}")


class Nav2PointCloudParams(Substitution):
    """Resolve Nav2 YAML with measured geometry and an optional camera overlay."""

    def __init__(
        self,
        source_file,
        enabled,
        profile_file=None,
        hardware_mode="false",
        hardware_max_linear_speed=str(DEFAULT_HARDWARE_MAX_LINEAR_SPEED),
        hardware_max_angular_speed=str(DEFAULT_HARDWARE_MAX_ANGULAR_SPEED),
    ):
        super().__init__()
        self._source_file = normalize_to_list_of_substitutions(source_file)
        self._enabled = normalize_to_list_of_substitutions(enabled)
        self._profile_file = (
            normalize_to_list_of_substitutions(profile_file)
            if profile_file is not None
            else None
        )
        self._hardware_mode = normalize_to_list_of_substitutions(hardware_mode)
        self._hardware_max_linear_speed = normalize_to_list_of_substitutions(
            hardware_max_linear_speed
        )
        self._hardware_max_angular_speed = normalize_to_list_of_substitutions(
            hardware_max_angular_speed
        )

    def perform(self, context):
        source_file = perform_substitutions(context, self._source_file)
        enabled = perform_substitutions(context, self._enabled)
        point_cloud_enabled = _enabled(enabled)
        hardware_mode = _enabled(
            perform_substitutions(context, self._hardware_mode)
        )
        hardware_max_linear_speed = DEFAULT_HARDWARE_MAX_LINEAR_SPEED
        hardware_max_angular_speed = DEFAULT_HARDWARE_MAX_ANGULAR_SPEED
        if hardware_mode:
            hardware_max_linear_speed = perform_substitutions(
                context, self._hardware_max_linear_speed
            )
            hardware_max_angular_speed = perform_substitutions(
                context, self._hardware_max_angular_speed
            )
        profile_file = (
            perform_substitutions(context, self._profile_file)
            if self._profile_file is not None
            else None
        )
        if not point_cloud_enabled and not profile_file and not hardware_mode:
            return source_file
        return write_point_cloud_nav2_parameters(
            source_file,
            profile_file=profile_file,
            enable_point_cloud=point_cloud_enabled,
            hardware_mode=hardware_mode,
            hardware_max_linear_speed=hardware_max_linear_speed,
            hardware_max_angular_speed=hardware_max_angular_speed,
        )

    def describe(self):
        return "Nav2 parameters with the optional local point-cloud overlay"
