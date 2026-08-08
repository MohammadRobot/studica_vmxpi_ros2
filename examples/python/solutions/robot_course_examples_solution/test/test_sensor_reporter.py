"""Unit tests for the Lab 5 reference solution."""

from robot_course_examples_solution.sensor_reporter import format_summary


def test_format_summary_uses_name_units_and_three_decimals():
    assert format_summary("class_4wd", (1.25, -0.5)) == (
        "class_4wd: x=1.250 m, y=-0.500 m"
    )
