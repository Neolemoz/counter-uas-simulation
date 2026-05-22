"""Block SA/federation/orchestration paths from RT bridge writes."""

from __future__ import annotations

from pathlib import Path


def repo_root_from(start: Path | None = None) -> Path:
    if start is None:
        start = Path(__file__).resolve()
    for parent in [start, *start.parents]:
        if (parent / "AGENTS.md").is_file():
            return parent
    raise RuntimeError("repository root not found")


def rt_sandbox_runs_dir(repo_root: Path | None = None) -> Path:
    root = repo_root or repo_root_from()
    path = root / "runs" / "rt_sandbox"
    path.mkdir(parents=True, exist_ok=True)
    return path


def assert_writable_path(path: Path, repo_root: Path | None = None) -> None:
    root = repo_root or repo_root_from()
    allowed_root = (root / "runs" / "rt_sandbox").resolve()
    resolved = path.resolve()
    if not resolved.is_relative_to(allowed_root):
        rel = resolved.relative_to(root.resolve()) if resolved.is_relative_to(root.resolve()) else resolved
        raise PermissionError(
            f"RT bridge writes limited to runs/rt_sandbox/ (got {rel.as_posix()})"
        )
