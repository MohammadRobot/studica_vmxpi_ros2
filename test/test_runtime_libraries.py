"""ELF fixture tests; no binary execution, native library load or physical access."""
import importlib.util
from pathlib import Path
import struct

import pytest

SPEC = importlib.util.spec_from_file_location(
    'runtime_libraries', Path(__file__).resolve().parents[1] / 'scripts/inspect_runtime_libraries.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def elf(path, machine=183):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = bytearray(64)
    data[:6] = b'\x7fELF\x02\x01'
    struct.pack_into('<H', data, 16, 3)
    struct.pack_into('<H', data, 18, machine)
    path.write_bytes(data)
    return path


def fixture(tmp_path):
    install, system = tmp_path / 'install', tmp_path / 'system'
    executable = elf(install / 'lib/pkg/node')
    library = elf(system / 'libvendor.so')

    def read(path):
        return dict(needed=['libvendor.so'] if path == executable else [], search_paths=[])
    return install, system, executable, library, read


def test_closure_and_content_binding(tmp_path):
    install, system, executable, library, read = fixture(tmp_path)
    report = MODULE.inspect(install, {'system': system}, read)
    assert report['runtime_tested'] is False
    assert report['artifacts']['install/lib/pkg/node']['needed'] == {'libvendor.so': 'system/libvendor.so'}
    assert report['artifacts']['system/libvendor.so']['sha256'] == MODULE.sha256(library)
    old = report['artifacts']['install/lib/pkg/node']['sha256']
    executable.write_bytes(executable.read_bytes() + b'changed')
    assert MODULE.inspect(install, {'system': system}, read)['artifacts']['install/lib/pkg/node']['sha256'] != old


def test_missing_dependency(tmp_path):
    install, system, _, library, read = fixture(tmp_path)
    library.unlink()
    with pytest.raises(MODULE.InventoryError, match='missing dependency'):
        MODULE.inspect(install, {'system': system}, read)


def test_wrong_dependency_architecture(tmp_path):
    install, system, _, library, read = fixture(tmp_path)
    elf(library, 62)
    with pytest.raises(MODULE.InventoryError, match='architecture'):
        MODULE.inspect(install, {'system': system}, read)


def test_conflicting_libraries(tmp_path):
    install, system, _, _, read = fixture(tmp_path)
    elf(install / 'lib/libvendor.so')
    with pytest.raises(MODULE.InventoryError, match='ambiguous'):
        MODULE.inspect(install, {'system': system}, read)


def test_symlink_escape(tmp_path):
    install, system, _, library, read = fixture(tmp_path)
    outside = elf(tmp_path / 'outside/libvendor.so')
    library.unlink()
    library.symlink_to(outside)
    with pytest.raises(MODULE.InventoryError, match='escapes'):
        MODULE.inspect(install, {'system': system}, read)


def test_parser_rejects_invalid_binary(tmp_path):
    path = elf(tmp_path / 'fake.so')
    path.write_bytes(b'invalid ELF input')
    with pytest.raises(MODULE.InventoryError, match='readelf rejected'):
        MODULE.dynamic_entries(path)


def test_parser_reads_real_host_binary_without_execution():
    # This is the metadata of an existing host ELF, not a target/ARM64 test.
    result = MODULE.dynamic_entries(Path('/usr/bin/true'))
    assert 'libc.so.6' in result['needed']
