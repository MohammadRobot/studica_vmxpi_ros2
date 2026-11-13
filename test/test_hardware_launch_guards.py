"""Validate launch decisions without executing any action or hardware process."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest

PATH = Path(__file__).resolve().parents[1] / 'bringup/launch/_robot_runtime.launch.py'
SPEC = importlib.util.spec_from_file_location('guarded_runtime', PATH)
RUNTIME = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNTIME)


def test_hardware_simulated_clock_rejected_before_profile_resolution():
    context = LaunchContext()
    context.launch_configurations.update(robot_profile='stack_4wd', use_hardware='true',
                                         use_imu_odometry='true', use_sim_time='true')
    with pytest.raises(ValueError, match='use_sim_time'):
        RUNTIME._set_runtime_controller_file(context)


def test_spawner_success_continues_and_failure_stops_session():
    assert RUNTIME._hardware_spawner_exited(SimpleNamespace(returncode=0), None) == []
    assert len(RUNTIME._hardware_spawner_exited(SimpleNamespace(returncode=1), None)) == 1
