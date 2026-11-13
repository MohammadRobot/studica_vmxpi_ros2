"""Regression checks for local documentation link parsing; no ROS runtime."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'check_docs.py'
SPEC = importlib.util.spec_from_file_location('check_docs', SCRIPT)
CHECK_DOCS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK_DOCS)


@pytest.mark.parametrize('target', [
    '<guide with spaces.md>',
    '<guide with spaces.md> "Guide title"',
    'guide%20with%20spaces.md#build',
])
def test_existing_spaced_path(tmp_path, target):
    """Angle brackets and URL escapes must resolve to the full filename."""
    (tmp_path / 'guide with spaces.md').write_text('Guide', encoding='utf-8')
    (tmp_path / 'README.md').write_text(f'[Guide]({target})', encoding='utf-8')
    assert CHECK_DOCS.check(tmp_path) == []


def test_missing_link_and_empty_destination(tmp_path):
    """Empty destinations are valid; missing files must still be reported."""
    (tmp_path / 'README.md').write_text(
        '[Here]() [Missing](<missing guide.md>)', encoding='utf-8')
    failures = CHECK_DOCS.check(tmp_path)
    assert len(failures) == 1
    assert 'missing guide.md' in failures[0]
