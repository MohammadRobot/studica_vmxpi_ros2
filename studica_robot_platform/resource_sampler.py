"""Bounded host resource observations; no ROS, device drivers or control operations."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import socket
import time


def snapshot(proc=Path('/proc'), thermal=Path('/sys/class/thermal'), disk=Path('/')):
    errors = []
    result = {'timestamp_utc': datetime.now(timezone.utc).isoformat(),
              'host': socket.gethostname(), 'evidence_scope': 'host_resources',
              'cpu_ticks': None, 'memory_bytes': {}, 'network_bytes': {},
              'temperature_c': {}, 'disk_bytes': {}}
    try:
        values = [int(v) for v in (proc / 'stat').read_text().splitlines()[0].split()[1:9]]
        if len(values) != 8 or any(v < 0 for v in values):
            raise ValueError('invalid CPU counters')
        result['cpu_ticks'] = {'total': sum(values), 'idle': values[3] + values[4]}
    except (OSError, ValueError, IndexError) as error:
        errors.append('cpu: ' + str(error))
    try:
        for line in (proc / 'meminfo').read_text().splitlines():
            key, value = line.split(':', 1)
            if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
                result['memory_bytes'][key] = int(value.split()[0]) * 1024
        if (not {'MemTotal', 'MemAvailable'} <= result['memory_bytes'].keys()
                or any(value < 0 for value in result['memory_bytes'].values())):
            raise ValueError('missing or negative memory counters')
    except (OSError, ValueError, IndexError) as error:
        errors.append('memory: ' + str(error))
    try:
        for line in (proc / 'net/dev').read_text().splitlines()[2:]:
            name, data = line.split(':', 1)
            fields = data.split()
            result['network_bytes'][name.strip()] = {'rx': int(fields[0]), 'tx': int(fields[8])}
        if not result['network_bytes']:
            raise ValueError('no network counters')
    except (OSError, ValueError, IndexError) as error:
        errors.append('network: ' + str(error))
    for zone in sorted(thermal.glob('thermal_zone*')):
        try:
            label = (zone / 'type').read_text().strip()
            value = float((zone / 'temp').read_text()) / 1000
            if not math.isfinite(value):
                raise ValueError('nonfinite temperature')
            result['temperature_c'][zone.name + ':' + label] = value
        except (OSError, ValueError) as error:
            errors.append('thermal: ' + str(error))
    if not result['temperature_c']:
        errors.append('thermal: unavailable')
    try:
        usage = shutil.disk_usage(disk)
        result['disk_bytes'] = dict(total=usage.total, used=usage.used, free=usage.free)
    except OSError as error:
        errors.append('disk: ' + str(error))
    result['observation_errors'] = errors
    return result


def cpu_utilization(previous, current):
    if previous is None or current is None:
        return None
    total, idle = current['total'] - previous['total'], current['idle'] - previous['idle']
    if total <= 0 or idle < 0 or idle > total:
        return None
    return 100 * (total - idle) / total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-seconds', type=float, default=60)
    parser.add_argument('--interval-seconds', type=float, default=5)
    args = parser.parse_args()
    if (not math.isfinite(args.duration_seconds) or not 0 < args.duration_seconds <= 86400
            or not math.isfinite(args.interval_seconds) or not 1 <= args.interval_seconds <= 60):
        parser.error('duration must be (0, 86400]; interval must be [1, 60] seconds')
    try:
        with args.output.open('x', encoding='utf-8') as stream:
            started = time.monotonic()
            deadline = started + args.duration_seconds
            previous = None
            while True:
                observed = snapshot()
                observed['elapsed_monotonic_sec'] = time.monotonic() - started
                observed['cpu_percent'] = cpu_utilization(previous, observed['cpu_ticks'])
                previous = observed['cpu_ticks']
                stream.write(json.dumps(observed, allow_nan=False) + '\n')
                stream.flush()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(args.interval_seconds, remaining))
    except KeyboardInterrupt:
        return 130
    except OSError as error:
        parser.exit(2, f'resource sampler: {error}\n')
    return 0
