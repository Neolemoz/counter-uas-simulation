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


def rt_sandbox_captures_dir(repo_root: Path | None = None) -> Path:
    base = rt_sandbox_runs_dir(repo_root)
    path = base / "captures"
    path.mkdir(parents=True, exist_ok=True)
    return path


def rt_sandbox_sa_handoff_dir(repo_root: Path | None = None) -> Path:
    base = rt_sandbox_runs_dir(repo_root)
    path = base / "sa_handoff"
    path.mkdir(parents=True, exist_ok=True)
    return path


def assert_sa_handoff_writable(path: Path, repo_root: Path | None = None) -> None:
    root = repo_root or repo_root_from()
    allowed = rt_sandbox_sa_handoff_dir(root).resolve()
    resolved = path.resolve()
    if not resolved.is_relative_to(allowed):
        raise PermissionError(
            f"RT SA handoff writes limited to runs/rt_sandbox/sa_handoff/ (got {resolved})"
        )
    assert_writable_path(path, root)


def assert_writable_path(path: Path, repo_root: Path | None = None) -> None:
    root = repo_root or repo_root_from()
    allowed_root = (root / "runs" / "rt_sandbox").resolve()
    resolved = path.resolve()
    if not resolved.is_relative_to(allowed_root):
        rel = resolved.relative_to(root.resolve()) if resolved.is_relative_to(root.resolve()) else resolved
        raise PermissionError(
            f"RT bridge writes limited to runs/rt_sandbox/ (got {rel.as_posix()})"
        )


def assert_capture_writable(path: Path, repo_root: Path | None = None) -> None:
    root = repo_root or repo_root_from()
    allowed = (rt_sandbox_captures_dir(root)).resolve()
    resolved = path.resolve()
    if not resolved.is_relative_to(allowed):
        raise PermissionError(
            f"RT capture writes limited to runs/rt_sandbox/captures/ (got {resolved})"
        )
    assert_writable_path(path, root)


def assert_template_ref_blocked(ref: str, repo_root: Path | None = None) -> None:
    """Raise if template reference targets SA/corpus/orchestration surfaces."""
    root = repo_root or repo_root_from()
    low = ref.lower().replace("\\", "/")
    blocked_fragments = (
        "fixtures/scenarios",
        "platform/sa-r0-viewer",
        "fixtures/orchestration",
        "fixtures/sa_r0",
        "replay_federation",
        "federation_index",
        "scenario_pack_ref",
    )
    for frag in blocked_fragments:
        if frag in low:
            raise PermissionError(f"RT template ref blocked: {ref}")
    if ref.startswith("/") or ref.startswith(".."):
        raise PermissionError(f"RT template ref blocked: {ref}")
    try:
        assert_sa_path_blocked((root / ref).resolve(), root)
    except PermissionError:
        raise PermissionError(f"RT template ref blocked: {ref}") from None


def assert_sa_path_blocked(path: Path, repo_root: Path | None = None) -> None:
    """Raise if path targets SA viewer, fixtures corpus, or federation surfaces."""
    root = repo_root or repo_root_from()
    resolved = path.resolve()
    blocked_prefixes = (
        (root / "platform" / "sa-r0-viewer").resolve(),
        (root / "fixtures" / "sa_r0").resolve(),
        (root / "fixtures" / "orchestration").resolve(),
        (root / "fixtures" / "scenarios").resolve(),
    )
    for prefix in blocked_prefixes:
        if prefix.exists() and resolved.is_relative_to(prefix):
            raise PermissionError(f"RT export blocked for SA/corpus path: {resolved}")
    low = resolved.as_posix().lower()
    if "replay_federation" in low or "federation_index" in low:
        raise PermissionError(f"RT export blocked for federation path: {resolved}")
