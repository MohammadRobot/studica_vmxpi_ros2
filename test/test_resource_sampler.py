"""Synthetic Linux counters; no physical workload."""
from pathlib import Path
from unittest.mock import patch

from studica_robot_platform.resource_sampler import cpu_utilization, snapshot


def test_cpu_counter_reset_is_unknown():
    assert cpu_utilization(None, {'total': 100, 'idle': 50}) is None
    assert cpu_utilization({'total': 100, 'idle': 50}, {'total': 200, 'idle': 75}) == 75
    assert cpu_utilization({'total': 100, 'idle': 50}, {'total': 90, 'idle': 40}) is None
    assert cpu_utilization({'total': 100, 'idle': 50}, {'total': 110, 'idle': 80}) is None


def test_missing_metrics_remain_unavailable(tmp_path):
    with patch('shutil.disk_usage', side_effect=OSError('missing disk')):
        result = snapshot(tmp_path, tmp_path, tmp_path)
    assert result['cpu_ticks'] is None
    assert result['memory_bytes'] == {}
    assert len(result['observation_errors']) == 5


def test_synthetic_host_metrics(tmp_path):
    (tmp_path / 'stat').write_text('cpu 10 0 10 70 10 0 0 0 5 0\n')
    (tmp_path / 'meminfo').write_text('MemTotal: 1024 kB\nMemAvailable: 512 kB\n')
    (tmp_path / 'net').mkdir()
    (tmp_path / 'net/dev').write_text('header\nheader\n wlan0: 100 0 0 0 0 0 0 0 200 0 0 0 0 0 0 0\n')
    zone = tmp_path / 'thermal_zone0'
    zone.mkdir()
    (zone / 'type').write_text('cpu-thermal')
    (zone / 'temp').write_text('42500')
    result = snapshot(tmp_path, tmp_path, Path('/'))
    assert result['cpu_ticks'] == {'total': 100, 'idle': 80}
    assert result['memory_bytes']['MemAvailable'] == 512 * 1024
    assert result['network_bytes']['wlan0'] == {'rx': 100, 'tx': 200}
    assert result['temperature_c']['thermal_zone0:cpu-thermal'] == 42.5
    assert result['observation_errors'] == []
