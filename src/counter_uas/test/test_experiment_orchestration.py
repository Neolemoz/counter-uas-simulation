"""Tests for PLAT-SA-H3 offline experiment orchestration."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
_MANIFESTS = _REPO / "fixtures" / "orchestration" / "manifests"


def test_lint_manifests_check() -> None:
    proc = subprocess.run(
        [sys.executable, str(_EVAL / "lint_experiment_manifest.py"), str(_MANIFESTS), "--check"],
        cwd=_REPO,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr


def test_run_queue_dry_run_ridge() -> None:
    manifest = _MANIFESTS / "ridge_defense_synthetic.json"
    proc = subprocess.run(
        [
            sys.executable,
            str(_EVAL / "run_experiment_queue.py"),
            "--manifest",
            str(manifest),
            "--dry-run",
            "--no-write-queue",
        ],
        cwd=_REPO,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr + proc.stdout


def test_queue_snapshot_schema() -> None:
    path = _REPO / "fixtures" / "orchestration" / "queues" / "valley_ingress_validation_only_queue.json"
    if not path.is_file():
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["artifact_type"] == "experiment_run_queue_v1"
    assert data["governance_banner"]
    assert any(j.get("status") == "completed" for j in data.get("jobs") or [])


def test_runtime_capture_blocked_without_flag() -> None:
    manifest = _MANIFESTS / "capture_pipeline_template.json"
    proc = subprocess.run(
        [
            sys.executable,
            str(_EVAL / "run_experiment_queue.py"),
            "--manifest",
            str(manifest),
            "--job",
            "capture_template",
            "--no-write-queue",
        ],
        cwd=_REPO,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 1


def test_async_worker_blocked_without_flag() -> None:
    manifest = _MANIFESTS / "ridge_defense_synthetic.json"
    proc = subprocess.run(
        [
            sys.executable,
            str(_EVAL / "run_experiment_queue.py"),
            "--manifest",
            str(manifest),
            "--dry-run",
            "--no-write-queue",
        ],
        cwd=_REPO,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0
    report_proc = subprocess.run(
        [
            sys.executable,
            "-c",
            "import json,sys; from pathlib import Path; "
            "sys.path.insert(0,'scripts/evaluation'); "
            "from experiment_orchestration import run_manifest; "
            "r=run_manifest(Path(sys.argv[1]), dry_run=True, write_queue=False); "
            "print(json.dumps(r['report'].get('async_worker_governance',{})))",
            str(manifest),
        ],
        cwd=_REPO,
        capture_output=True,
        text=True,
    )
    meta = json.loads(report_proc.stdout.strip())
    assert meta.get("worker_provenance") == "blocked"
