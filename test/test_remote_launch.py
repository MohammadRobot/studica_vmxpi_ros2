"""Ensure the single-terminal modes select one command producer family."""
import importlib.util
from pathlib import Path
from unittest.mock import patch
import pytest
from launch import LaunchContext

path = Path(__file__).resolve().parents[1] / 'bringup/launch/remote.launch.py'
spec = importlib.util.spec_from_file_location('remote_launch', path)
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)


def context(mode, map_file=''):
    c = LaunchContext()
    c.launch_configurations.update(mode=mode, map=map_file)
    return c


def test_navigation_requires_map():
    with pytest.raises(RuntimeError, match='existing map'):
        remote.session(context('navigation', '/nonexistent-map.yaml'))


def test_modes(tmp_path):
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n')
    with patch.object(remote, 'deferred_include', return_value='include') as include:
        assert len(remote.session(context('drive'))) == 3
        include.assert_not_called()
        assert len(remote.session(context('slam'))) == 3
        assert include.call_args.args[2]['use_joystick'] == 'false'
        assert len(remote.session(context('navigation', str(map_file)))) == 3
        assert include.call_args.args[2]['navigation_output_topic'] == '/cmd_vel/navigation'
        assert include.call_args.args[1] == 'navigation.launch.py'


@pytest.mark.parametrize('value', ['nan', 'inf', '-0.1', '0', '0.31'])
def test_rejects_invalid_joystick_speed(value):
    c = context('drive')
    c.launch_configurations['linear_speed'] = value
    with pytest.raises(RuntimeError, match='linear_speed'):
        remote.session(c)
