"""Repository-local tests for the copyable Python course examples."""

import ast
from pathlib import Path


EXAMPLES_ROOT = Path(__file__).resolve().parents[1]


def test_every_python_example_has_valid_syntax():
    files = sorted(EXAMPLES_ROOT.rglob("*.py"))
    assert files
    for path in files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_starter_has_todos_and_solution_does_not():
    starter = EXAMPLES_ROOT / "starters" / "robot_course_examples"
    solution = EXAMPLES_ROOT / "solutions" / "robot_course_examples_solution"
    starter_text = "\n".join(path.read_text(encoding="utf-8") for path in starter.rglob("*.py"))
    solution_text = "\n".join(path.read_text(encoding="utf-8") for path in solution.rglob("*.py"))
    assert "TODO" in starter_text
    assert "TODO" not in solution_text


def test_examples_do_not_publish_robot_motion_or_tf():
    source_files = [
        *EXAMPLES_ROOT.joinpath("starters").rglob("*.py"),
        *EXAMPLES_ROOT.joinpath("solutions").rglob("*.py"),
    ]
    source = "\n".join(path.read_text(encoding="utf-8") for path in source_files)
    assert 'create_publisher(Twist, "/cmd_vel"' not in source
    assert "TransformBroadcaster" not in source
