#!/usr/bin/env python3
"""Inventory ELF dependencies without executing binaries or loading vendor libraries."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess


class InventoryError(ValueError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def elf_machine(path):
    with path.open('rb') as stream:
        header = stream.read(64)
    if header[:4] != b'\x7fELF':
        return None
    if len(header) < 64 or header[4] != 2 or header[5] not in (1, 2):
        raise InventoryError(f'invalid or non-64-bit ELF: {path}')
    order = '<H' if header[5] == 1 else '>H'
    if struct.unpack(order, header[16:18])[0] not in (2, 3):
        raise InventoryError(f'ELF is not an executable/shared object: {path}')
    return struct.unpack(order, header[18:20])[0]


def dynamic_entries(path):
    # readelf parses bytes; unlike ldd it does not invoke an ELF interpreter.
    result = subprocess.run(['/usr/bin/readelf', '--wide', '--dynamic', str(path)],
                            capture_output=True, text=True, timeout=20,
                            env=dict(os.environ, LC_ALL='C'), check=False)
    if result.returncode:
        raise InventoryError(f'readelf rejected {path.name}: {result.stderr[:200]}')
    entries = {'needed': [], 'search_paths': []}
    for kind, value in re.findall(r'\((NEEDED|RPATH|RUNPATH)\).*?\[([^\]]*)\]', result.stdout):
        if kind == 'NEEDED':
            if not value or '/' in value or value in ('.', '..'):
                raise InventoryError('DT_NEEDED must be a library basename')
            entries['needed'].append(value)
        else:
            entries['search_paths'].append({'kind': kind, 'value': value})
    return entries


def inspect(install, external_roots, dynamic_reader=dynamic_entries, expected_machine=183):
    """Check dependency availability, not dynamic-loader or hardware correctness."""
    install = Path(install).resolve(strict=True)
    roots = {'install': install}
    for label, root in external_roots.items():
        if not re.fullmatch(r'[a-z][a-z0-9_-]*', label) or label == 'install':
            raise InventoryError('invalid external root label')
        roots[label] = Path(root).resolve(strict=True)
    if any(not root.is_dir() for root in roots.values()):
        raise InventoryError('all roots must be directories')

    def identify(path):
        for label, root in roots.items():
            if path.is_relative_to(root):
                return label + '/' + path.relative_to(root).as_posix()
        raise InventoryError(f'library symlink escapes approved roots: {path}')

    libraries = {}
    initial = set()
    for label, root in roots.items():
        for path in sorted(root.rglob('*')):
            if not path.is_file():
                continue
            resolved = path.resolve(strict=True)
            if label == 'install':
                identify(resolved)
                if elf_machine(resolved) is not None:
                    initial.add(resolved)
            if '.so' in path.name:
                libraries.setdefault(path.name, set()).add(resolved)
    if not initial:
        raise InventoryError('install contains no ELF runtime artifacts')
    records, pending = {}, sorted(initial)
    while pending:
        path = pending.pop()
        identity = identify(path)
        if identity in records:
            continue
        if elf_machine(path) != expected_machine:
            raise InventoryError(f'wrong ELF architecture: {identity}')
        entries = dynamic_reader(path)
        dependencies = {}
        for name in entries['needed']:
            matches = libraries.get(name, set())
            if not matches:
                raise InventoryError(f'missing dependency {name} required by {identity}')
            if len(matches) != 1:
                raise InventoryError(f'ambiguous dependency {name} required by {identity}')
            dependency = next(iter(matches))
            dependencies[name] = identify(dependency)
            pending.append(dependency)
        records[identity] = dict(sha256=sha256(path), needed=dependencies,
                                 search_paths=entries['search_paths'])
    return dict(schema_version=1, evidence_scope='static_elf_dependency_availability',
                elf_machine=expected_machine, runtime_tested=False, activation_authorized=False,
                artifacts={key: records[key] for key in sorted(records)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix', type=Path, required=True)
    parser.add_argument('--library-root', action='append', default=[], metavar='LABEL=PATH')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        roots = {}
        for item in args.library_root:
            label, path = item.split('=', 1)
            if label in roots:
                raise InventoryError('duplicate root label')
            roots[label] = path
        report = inspect(args.install_prefix, roots)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write('\n')
        print(f'Static inventory: {len(report["artifacts"])} ELF artifacts; runtime testing pending')
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.exit(2, f'runtime inventory rejected: {error}\n')


if __name__ == '__main__':
    main()
