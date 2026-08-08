#!/usr/bin/env python3
"""Check relative links in the classroom Markdown documentation."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
from urllib.parse import unquote


LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")


def relative_links(markdown: Path):
    """Yield source spelling and resolved path for local Markdown links."""
    text = markdown.read_text(encoding="utf-8")
    for match in LINK.finditer(text):
        target = match.group(1).strip()
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1]
        target = target.split(maxsplit=1)[0]
        if not target or target.startswith(("#", "http://", "https://", "mailto:")):
            continue
        file_part = unquote(target.split("#", maxsplit=1)[0])
        if not file_part:
            continue
        yield match.group(1), (markdown.parent / file_part).resolve()


def check(root: Path) -> list[str]:
    failures: list[str] = []
    for markdown in sorted([root / "README.md", *root.joinpath("docs").rglob("*.md")]):
        if not markdown.is_file():
            continue
        for spelling, resolved in relative_links(markdown):
            if not resolved.exists():
                failures.append(
                    f"{markdown.relative_to(root)}: missing link {spelling!r}"
                )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    failures = check(root)
    if failures:
        print("Documentation link check failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    count = 1 + len(list(root.joinpath("docs").rglob("*.md")))
    print(f"[check] Documentation links: {count} Markdown files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
