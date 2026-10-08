#!/usr/bin/env python3
"""Validate the lightweight README/AGENTS documentation contract."""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {
    ".git",
    ".pytest_cache",
    "build",
    "install",
    "log",
    "tmp",
    "__pycache__",
}
REQUIRED_FILES = (
    ROOT / "AGENTS.md",
    ROOT / "README.md",
    ROOT / "docs" / "agent" / "README.md",
    ROOT / "docs" / "agent" / "REPO_MAP.md",
    ROOT / "docs" / "agent" / "COMMANDS.md",
    ROOT / "docs" / "agent" / "DATA_POLICY.md",
    ROOT / "results" / "README.md",
)
MARKDOWN_LINK = re.compile(
    r"\[[^\]]+\]\((?:<(?P<angle>[^>]+)>|(?P<plain>[^)]+))\)"
)


def documentation_files() -> list[Path]:
    """Return the compact documentation contract outside generated directories."""
    files = set(REQUIRED_FILES)
    for path in ROOT.rglob("*.md"):
        if any(part in EXCLUDED_PARTS for part in path.relative_to(ROOT).parts):
            continue
        if path.name.startswith("README") or path.name in {
            "AGENTS.md",
            "AGENTS.override.md",
        }:
            files.add(path)
    return sorted(files)


def link_target(match: re.Match[str]) -> str:
    """Extract one Markdown link destination without an optional title."""
    target = match.group("angle") or match.group("plain") or ""
    target = target.strip()
    if match.group("plain") and " " in target:
        target = target.split(" ", 1)[0]
    return unquote(target)


def main() -> int:
    """Check required files, Markdown fences, and local link destinations."""
    issues: list[str] = []
    for path in REQUIRED_FILES:
        if not path.is_file():
            issues.append(f"missing required documentation: {path.relative_to(ROOT)}")

    files = documentation_files()
    for path in files:
        text = path.read_text(encoding="utf-8")
        if text.count("```") % 2:
            issues.append(f"unclosed fenced code block: {path.relative_to(ROOT)}")
        for match in MARKDOWN_LINK.finditer(text):
            target = link_target(match)
            if not target or target.startswith("#") or "://" in target:
                continue
            local_part = target.split("#", 1)[0]
            destination = (path.parent / local_part).resolve()
            if not destination.exists():
                issues.append(
                    f"broken local link in {path.relative_to(ROOT)}: {target}"
                )

    if issues:
        print("Documentation validation failed:")
        for issue in issues:
            print(f"- {issue}")
        return 1

    print(f"Documentation validation passed: {len(files)} contract files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
