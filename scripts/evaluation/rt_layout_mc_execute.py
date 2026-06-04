#!/usr/bin/env python3
"""Maintainer-only RT layout Monte Carlo execution manifests and lifecycle.

Prepare writes job artifacts and a command preview. Execute invokes the rendered
Monte Carlo command and updates job status (CLI only; no UI or bridge execution).
"""

from __future__ import annotations

import argparse
import json
import math
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parents[1]
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import rt_layout_mc_profile

JOB_STATUS_PREPARED = "prepared"
JOB_STATUS_RUNNING = "running"
JOB_STATUS_COMPLETED = "completed"
JOB_STATUS_FAILED = "failed"
EXECUTABLE_STATUSES = frozenset({JOB_STATUS_PREPARED, JOB_STATUS_FAILED})
STDERR_SUMMARY_MAX_CHARS = 4000

HANDOFF_SCHEMA_VERSION = "rt_layout_mc_handoff_v1"
JOB_PREVIEW_SCHEMA_VERSION = "rt_mc_job_preview_v1"
EXECUTION_MANIFEST_SCHEMA_VERSION = "rt_layout_mc_execution_manifest_v1"
JOB_STATUS_SCHEMA_VERSION = "rt_layout_mc_job_status_v1"
RESULT_SUMMARY_SCHEMA_VERSION = "rt_layout_mc_result_summary_v1"
RESULT_LINK_SCHEMA_VERSION = "rt_layout_mc_result_link_v1"
RESULT_SUMMARY_FILENAME = "result_summary.json"
RESULT_LINK_FILENAME = "result_link.json"
DEFAULT_JOBS_ROOT = Path("runs") / "rt_sandbox" / "mc_jobs"
DEFAULT_MC_OUT_DIR = Path("runs") / "mc"
SCENARIO_LABELS = {
    "single-target": "single",
    "multi-target": "multi",
    "bringup": "bringup",
    "single": "single",
    "multi": "multi",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _read_json_object(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _safe_id(value: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in value.strip())
    return cleaned.strip("._-") or "rt_layout_mc"


def _scenario_from_label(label: Any) -> str:
    raw = str(label or "").strip()
    scenario = SCENARIO_LABELS.get(raw)
    if not scenario:
        allowed = ", ".join(sorted(SCENARIO_LABELS))
        raise ValueError(f"unsupported scenario_label {raw!r}; expected one of: {allowed}")
    return scenario


def _int_positive(value: Any, field: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a positive integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a positive integer") from exc
    if parsed <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return parsed


def _validate_job_preview(data: dict[str, Any]) -> dict[str, Any]:
    if data.get("schema_version") != JOB_PREVIEW_SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {JOB_PREVIEW_SCHEMA_VERSION!r}")
    required = ("geometry_id", "run_count", "scenario_label", "launch_args", "source_layout_id")
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError("missing required job preview field(s): " + ", ".join(missing))

    geometry_id = str(data.get("geometry_id") or "").strip()
    source_layout_id = str(data.get("source_layout_id") or "").strip()
    if not geometry_id:
        raise ValueError("geometry_id must be a non-empty string")
    if not source_layout_id:
        raise ValueError("source_layout_id must be a non-empty string")

    return {
        "geometry_id": geometry_id,
        "source_layout_id": source_layout_id,
        "run_count": _int_positive(data.get("run_count"), "run_count"),
        "scenario": _scenario_from_label(data.get("scenario_label")),
        "launch_args": str(data.get("launch_args") or "").strip(),
        "warnings": [str(item) for item in data.get("warnings") or []],
    }


def _load_job_source(path: Path) -> dict[str, Any]:
    data = _read_json_object(path)
    schema = data.get("schema_version")
    if schema == HANDOFF_SCHEMA_VERSION:
        layout = data.get("layout")
        job = data.get("mc_job_preview")
        if not isinstance(layout, dict):
            raise ValueError("handoff layout must be an object")
        if not isinstance(job, dict):
            raise ValueError("handoff mc_job_preview must be an object")

        validation = rt_layout_mc_profile.validate_layout(layout)
        if not validation["ok"]:
            raise ValueError("invalid handoff layout: " + "; ".join(validation["issues"]))
        parsed = _validate_job_preview(job)
        warnings = list(parsed["warnings"])
        warnings.extend(str(item) for item in validation.get("warnings") or [])

        layout_geometry_id = rt_layout_mc_profile.geometry_fingerprint(layout)
        if layout_geometry_id != parsed["geometry_id"]:
            warnings.append(
                "handoff layout geometry_id does not match prepared job geometry_id; "
                "review stale layout/job pairing before execution"
            )
        parsed["warnings"] = warnings
        return parsed

    if schema == JOB_PREVIEW_SCHEMA_VERSION:
        return _validate_job_preview(data)

    raise ValueError(
        f"schema_version must be {HANDOFF_SCHEMA_VERSION!r} or {JOB_PREVIEW_SCHEMA_VERSION!r}"
    )


def _default_job_id(source_layout_id: str, geometry_id: str, created_utc: str) -> str:
    stamp = created_utc.replace("-", "").replace(":", "").replace("Z", "Z")
    suffix = geometry_id.rsplit(":", 1)[-1][:8] or "geometry"
    return _safe_id(f"rtmc_{stamp}_{source_layout_id}_{suffix}")[:96]


def render_command(manifest: dict[str, Any]) -> str:
    cmd = [
        "python3",
        "scripts/monte_carlo.py",
        "run",
        "--n",
        str(manifest["run_count"]),
        "--seed-base",
        str(manifest["seed_base"]),
        "--geometry-id",
        str(manifest["geometry_id"]),
        "--scenario",
        str(manifest["scenario"]),
        "--label",
        str(manifest["job_id"]),
        "--out-dir",
        str(DEFAULT_MC_OUT_DIR),
        "--cohort",
        str(manifest["cohort"]),
        "--launch-args",
        str(manifest.get("launch_args") or ""),
    ]
    return shlex.join(cmd)


def prepare_job(
    *,
    input_path: Path,
    jobs_root: Path = DEFAULT_JOBS_ROOT,
    seed_base: int = 1,
    cohort: str | None = None,
    job_id: str | None = None,
    force: bool = False,
) -> dict[str, Any]:
    source = _load_job_source(input_path)
    created_utc = _utc_now()
    resolved_job_id = _safe_id(
        job_id
        or _default_job_id(source["source_layout_id"], source["geometry_id"], created_utc)
    )
    resolved_cohort = (cohort or f"rt_layout_mc_{resolved_job_id}").strip()
    if not resolved_cohort:
        raise ValueError("cohort must be a non-empty string")
    if seed_base < 0:
        raise ValueError("seed_base must be non-negative")

    job_dir = jobs_root / resolved_job_id
    if job_dir.exists() and not force:
        raise FileExistsError(f"job directory already exists: {job_dir}")
    job_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "schema_version": EXECUTION_MANIFEST_SCHEMA_VERSION,
        "job_id": resolved_job_id,
        "geometry_id": source["geometry_id"],
        "source_layout_id": source["source_layout_id"],
        "run_count": source["run_count"],
        "seed_base": int(seed_base),
        "scenario": source["scenario"],
        "launch_args": source["launch_args"],
        "cohort": resolved_cohort,
        "created_utc": created_utc,
        "source_handoff_ref": str(input_path),
        "warnings": source["warnings"],
    }
    command_preview = render_command(manifest)
    status = {
        "schema_version": JOB_STATUS_SCHEMA_VERSION,
        "job_id": resolved_job_id,
        "status": "prepared",
        "progress_current": 0,
        "progress_total": source["run_count"],
        "output_paths": None,
        "command_preview": command_preview,
    }

    _write_json(job_dir / "manifest.json", manifest)
    _write_json(job_dir / "status.json", status)
    (job_dir / "command.txt").write_text(command_preview + "\n", encoding="utf-8")
    return {
        "job_dir": str(job_dir),
        "manifest_path": str(job_dir / "manifest.json"),
        "status_path": str(job_dir / "status.json"),
        "command_path": str(job_dir / "command.txt"),
        "manifest": manifest,
        "status": status,
    }


def _load_manifest_ref(ref: Path) -> dict[str, Any]:
    path = ref / "manifest.json" if ref.is_dir() else ref
    data = _read_json_object(path)
    if data.get("schema_version") != EXECUTION_MANIFEST_SCHEMA_VERSION:
        raise ValueError(f"manifest schema_version must be {EXECUTION_MANIFEST_SCHEMA_VERSION!r}")
    return data


def _load_status_ref(ref: Path) -> dict[str, Any]:
    path = ref / "status.json" if ref.is_dir() else ref
    data = _read_json_object(path)
    if data.get("schema_version") != JOB_STATUS_SCHEMA_VERSION:
        raise ValueError(f"status schema_version must be {JOB_STATUS_SCHEMA_VERSION!r}")
    return data


def _job_dir_from_ref(job_ref: Path) -> Path:
    if job_ref.is_dir():
        return job_ref
    if job_ref.name in ("manifest.json", "status.json", "command.txt"):
        return job_ref.parent
    raise ValueError(f"job_ref must be a job directory or manifest/status path: {job_ref}")


def _status_path(job_dir: Path) -> Path:
    return job_dir / "status.json"


def _write_status(job_dir: Path, status: dict[str, Any]) -> None:
    _write_json(_status_path(job_dir), status)


def _base_status_fields(status: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": JOB_STATUS_SCHEMA_VERSION,
        "job_id": status["job_id"],
        "progress_total": status["progress_total"],
        "command_preview": status["command_preview"],
    }


def _output_paths_for_job(job_id: str, mc_out_dir: Path) -> dict[str, str]:
    return {
        "summary_json": str(mc_out_dir / f"{job_id}.json"),
        "summary_csv": str(mc_out_dir / f"{job_id}.csv"),
    }


def _truncate_stderr(stderr: str) -> str:
    text = (stderr or "").strip()
    if len(text) <= STDERR_SUMMARY_MAX_CHARS:
        return text
    return text[: STDERR_SUMMARY_MAX_CHARS - 3] + "..."


def validate_mc_outputs(job_id: str, mc_out_dir: Path) -> list[str]:
    """Return post-run file/label validation issues (empty when outputs are acceptable)."""
    issues: list[str] = []
    json_path = mc_out_dir / f"{job_id}.json"
    csv_path = mc_out_dir / f"{job_id}.csv"
    if not json_path.is_file():
        issues.append(f"missing summary JSON: {json_path}")
    if not csv_path.is_file():
        issues.append(f"missing summary CSV: {csv_path}")
    if not json_path.is_file():
        return issues
    try:
        summary = _read_json_object(json_path)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        issues.append(f"invalid summary JSON: {exc}")
        return issues
    label = summary.get("label")
    if label != job_id:
        issues.append(f"summary label {label!r} does not match job_id {job_id!r}")
    return issues


def _finite_number(value: Any) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    parsed = float(value)
    if not math.isfinite(parsed):
        return None
    return parsed


def _extract_lightweight_mc_metrics(summary: dict[str, Any]) -> dict[str, Any]:
    """Copy a small subset of aggregate MC metrics (not full per-run results)."""
    metrics: dict[str, Any] = {}
    for key in ("n_runs", "n_success", "success_rate"):
        if key in summary:
            parsed = _finite_number(summary[key])
            if parsed is not None:
                metrics[key] = int(parsed) if key in ("n_runs", "n_success") else parsed
    miss = summary.get("miss_distance_m")
    if isinstance(miss, dict):
        for src_key, dest_key in (("mean", "miss_distance_m_mean"), ("p95", "miss_distance_m_p95")):
            parsed = _finite_number(miss.get(src_key))
            if parsed is not None:
                metrics[dest_key] = parsed
    intercept = summary.get("intercept_time_s")
    if isinstance(intercept, dict):
        for src_key, dest_key in (("mean", "intercept_time_s_mean"), ("p95", "intercept_time_s_p95")):
            parsed = _finite_number(intercept.get(src_key))
            if parsed is not None:
                metrics[dest_key] = parsed
    return metrics


def validate_mc_handoff(
    *,
    manifest: dict[str, Any],
    mc_out_dir: Path,
) -> tuple[list[str], dict[str, Any] | None]:
    """Validate MC outputs and manifest identifier consistency before handoff artifacts."""
    job_id = str(manifest["job_id"])
    issues: list[str] = []
    json_path = mc_out_dir / f"{job_id}.json"
    csv_path = mc_out_dir / f"{job_id}.csv"
    if not json_path.is_file():
        issues.append(f"missing summary JSON: {json_path}")
    if not csv_path.is_file():
        issues.append(f"missing summary CSV: {csv_path}")
    if not json_path.is_file():
        return issues, None
    try:
        summary = _read_json_object(json_path)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        issues.append(f"invalid summary JSON: {exc}")
        return issues, None
    label = summary.get("label")
    if label != job_id:
        issues.append(f"summary label {label!r} does not match job_id {job_id!r}")
    n_runs = summary.get("n_runs")
    run_count = manifest.get("run_count")
    if n_runs is not None and run_count is not None:
        try:
            if int(n_runs) != int(run_count):
                issues.append(
                    f"summary n_runs {n_runs!r} does not match manifest run_count {run_count!r}"
                )
        except (TypeError, ValueError):
            issues.append("summary n_runs must be an integer when present")
    for field in ("geometry_id", "source_layout_id", "cohort"):
        manifest_value = str(manifest.get(field) or "").strip()
        if not manifest_value:
            issues.append(f"manifest {field} must be a non-empty string")
    return issues, summary


def build_result_summary(
    *,
    manifest: dict[str, Any],
    output_paths: dict[str, str],
    mc_summary: dict[str, Any],
    completed_utc: str,
) -> dict[str, Any]:
    return {
        "schema_version": RESULT_SUMMARY_SCHEMA_VERSION,
        "job_id": manifest["job_id"],
        "geometry_id": manifest["geometry_id"],
        "source_layout_id": manifest["source_layout_id"],
        "cohort": manifest["cohort"],
        "run_count": manifest["run_count"],
        "output_paths": output_paths,
        "completed_utc": completed_utc,
        "mc_metrics": _extract_lightweight_mc_metrics(mc_summary),
    }


def build_result_link(
    *,
    manifest: dict[str, Any],
    job_dir: Path,
    output_paths: dict[str, str],
    result_summary_path: Path,
    linked_utc: str,
) -> dict[str, Any]:
    return {
        "schema_version": RESULT_LINK_SCHEMA_VERSION,
        "job_id": manifest["job_id"],
        "geometry_id": manifest["geometry_id"],
        "source_layout_id": manifest["source_layout_id"],
        "cohort": manifest["cohort"],
        "linked_utc": linked_utc,
        "manifest_path": str(job_dir / "manifest.json"),
        "status_path": str(job_dir / "status.json"),
        "result_summary_path": str(result_summary_path),
        "output_paths": output_paths,
    }


def write_result_handoff_artifacts(
    *,
    job_dir: Path,
    manifest: dict[str, Any],
    output_paths: dict[str, str],
    mc_summary: dict[str, Any],
    completed_utc: str,
) -> dict[str, str]:
    summary_path = job_dir / RESULT_SUMMARY_FILENAME
    summary_doc = build_result_summary(
        manifest=manifest,
        output_paths=output_paths,
        mc_summary=mc_summary,
        completed_utc=completed_utc,
    )
    _write_json(summary_path, summary_doc)
    link_path = job_dir / RESULT_LINK_FILENAME
    link_doc = build_result_link(
        manifest=manifest,
        job_dir=job_dir,
        output_paths=output_paths,
        result_summary_path=summary_path,
        linked_utc=completed_utc,
    )
    _write_json(link_path, link_doc)
    return {
        "result_summary_path": str(summary_path),
        "result_link_path": str(link_path),
    }


def _status_running(status: dict[str, Any], *, started_utc: str) -> dict[str, Any]:
    out = _base_status_fields(status)
    out.update(
        {
            "status": JOB_STATUS_RUNNING,
            "progress_current": 0,
            "output_paths": None,
            "started_utc": started_utc,
        }
    )
    return out


def _status_completed(
    status: dict[str, Any],
    *,
    manifest: dict[str, Any],
    completed_utc: str,
    output_paths: dict[str, str],
    progress_current: int,
    result_summary_path: str,
    result_link_path: str,
) -> dict[str, Any]:
    out = _base_status_fields(status)
    out.update(
        {
            "status": JOB_STATUS_COMPLETED,
            "progress_current": progress_current,
            "output_paths": output_paths,
            "completed_utc": completed_utc,
            "return_code": 0,
            "geometry_id": manifest["geometry_id"],
            "source_layout_id": manifest["source_layout_id"],
            "cohort": manifest["cohort"],
            "result_summary_path": result_summary_path,
            "result_link_path": result_link_path,
        }
    )
    return out


def _status_failed(
    status: dict[str, Any],
    *,
    failed_utc: str,
    return_code: int | None,
    stderr_summary: str,
    failure_reason: str,
) -> dict[str, Any]:
    out = _base_status_fields(status)
    out.update(
        {
            "status": JOB_STATUS_FAILED,
            "progress_current": status.get("progress_current", 0),
            "output_paths": None,
            "failed_utc": failed_utc,
            "return_code": return_code,
            "stderr_summary": stderr_summary,
            "failure_reason": failure_reason,
        }
    )
    return out


def execute_job(
    *,
    job_ref: Path,
    dry_run: bool = False,
    repo_root: Path | None = None,
    mc_out_dir: Path | None = None,
    run_subprocess: Callable[..., subprocess.CompletedProcess[str]] | None = None,
) -> dict[str, Any]:
    job_dir = _job_dir_from_ref(job_ref)
    manifest = _load_manifest_ref(job_dir)
    status = _load_status_ref(job_dir)
    job_id = str(manifest["job_id"])
    if status.get("job_id") != job_id:
        raise ValueError(f"status job_id {status.get('job_id')!r} does not match manifest {job_id!r}")

    command = render_command(manifest)
    resolved_mc_out = mc_out_dir or DEFAULT_MC_OUT_DIR
    workspace = repo_root or _REPO_ROOT

    if dry_run:
        return {
            "job_dir": str(job_dir),
            "job_id": job_id,
            "dry_run": True,
            "command": command,
            "status": status,
        }

    current = str(status.get("status") or "")
    if current not in EXECUTABLE_STATUSES:
        raise ValueError(
            f"job status {current!r} is not executable; expected one of: "
            + ", ".join(sorted(EXECUTABLE_STATUSES))
        )

    started_utc = _utc_now()
    running = _status_running(status, started_utc=started_utc)
    _write_status(job_dir, running)

    runner = run_subprocess or subprocess.run
    argv = shlex.split(command)
    proc = runner(
        argv,
        cwd=workspace,
        capture_output=True,
        text=True,
        check=False,
    )
    return_code = int(proc.returncode)
    stderr_text = _truncate_stderr(proc.stderr or "")

    if return_code != 0:
        failed = _status_failed(
            running,
            failed_utc=_utc_now(),
            return_code=return_code,
            stderr_summary=stderr_text,
            failure_reason=f"monte_carlo subprocess exited with code {return_code}",
        )
        _write_status(job_dir, failed)
        return {
            "job_dir": str(job_dir),
            "job_id": job_id,
            "dry_run": False,
            "return_code": return_code,
            "status": failed,
        }

    validation_issues, mc_summary = validate_mc_handoff(
        manifest=manifest,
        mc_out_dir=resolved_mc_out,
    )
    if validation_issues or mc_summary is None:
        reason = "post-run validation failed: " + "; ".join(validation_issues or ["missing MC summary"])
        failed = _status_failed(
            running,
            failed_utc=_utc_now(),
            return_code=return_code,
            stderr_summary=stderr_text,
            failure_reason=reason,
        )
        _write_status(job_dir, failed)
        return {
            "job_dir": str(job_dir),
            "job_id": job_id,
            "dry_run": False,
            "return_code": return_code,
            "status": failed,
            "validation_issues": validation_issues,
        }

    output_paths = _output_paths_for_job(job_id, resolved_mc_out)
    completed_utc = _utc_now()
    handoff_paths = write_result_handoff_artifacts(
        job_dir=job_dir,
        manifest=manifest,
        output_paths=output_paths,
        mc_summary=mc_summary,
        completed_utc=completed_utc,
    )
    completed = _status_completed(
        running,
        manifest=manifest,
        completed_utc=completed_utc,
        output_paths=output_paths,
        progress_current=int(status["progress_total"]),
        result_summary_path=handoff_paths["result_summary_path"],
        result_link_path=handoff_paths["result_link_path"],
    )
    _write_status(job_dir, completed)
    return {
        "job_dir": str(job_dir),
        "job_id": job_id,
        "dry_run": False,
        "return_code": 0,
        "status": completed,
        "output_paths": output_paths,
        "result_summary_path": handoff_paths["result_summary_path"],
        "result_link_path": handoff_paths["result_link_path"],
    }


def cmd_prepare(args: argparse.Namespace) -> int:
    try:
        result = prepare_job(
            input_path=args.input_json,
            jobs_root=args.jobs_root,
            seed_base=args.seed_base,
            cohort=args.cohort,
            job_id=args.job_id,
            force=args.force,
        )
    except (OSError, json.JSONDecodeError, ValueError, FileExistsError) as exc:
        print(f"rt_layout_mc_execute prepare: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {key: result[key] for key in ("job_dir", "manifest_path", "status_path", "command_path")},
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    try:
        status = _load_status_ref(args.job_ref)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"rt_layout_mc_execute status: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0


def cmd_render_command(args: argparse.Namespace) -> int:
    try:
        manifest = _load_manifest_ref(args.job_ref)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"rt_layout_mc_execute render-command: {exc}", file=sys.stderr)
        return 2
    print(render_command(manifest))
    return 0


def cmd_execute(args: argparse.Namespace) -> int:
    try:
        result = execute_job(job_ref=args.job_ref, dry_run=args.dry_run)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"rt_layout_mc_execute execute: {exc}", file=sys.stderr)
        return 2
    if args.dry_run:
        print(result["command"])
        return 0
    status_name = result["status"]["status"]
    print(
        json.dumps(
            {
                "job_dir": result["job_dir"],
                "job_id": result["job_id"],
                "status": status_name,
                "return_code": result.get("return_code"),
                "output_paths": result.get("output_paths"),
                "result_summary_path": result.get("result_summary_path"),
                "result_link_path": result.get("result_link_path"),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if status_name == JOB_STATUS_COMPLETED else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prepare and execute RT layout MC jobs (maintainer CLI only)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    prepare = sub.add_parser("prepare", help="Write manifest/status/command preview for a handoff or job JSON.")
    prepare.add_argument("input_json", type=Path)
    prepare.add_argument("--jobs-root", type=Path, default=DEFAULT_JOBS_ROOT)
    prepare.add_argument("--seed-base", type=int, default=1)
    prepare.add_argument("--cohort", default=None)
    prepare.add_argument("--job-id", default=None)
    prepare.add_argument("--force", action="store_true")
    prepare.set_defaults(func=cmd_prepare)

    status = sub.add_parser("status", help="Print prepared job status JSON.")
    status.add_argument("job_ref", type=Path, help="Job directory or status.json path.")
    status.set_defaults(func=cmd_status)

    render = sub.add_parser("render-command", help="Render the dry-run Monte Carlo command.")
    render.add_argument("job_ref", type=Path, help="Job directory or manifest.json path.")
    render.set_defaults(func=cmd_render_command)

    execute = sub.add_parser("execute", help="Run the rendered Monte Carlo command and update job status.")
    execute.add_argument("job_ref", type=Path, help="Job directory or manifest.json path.")
    execute.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the rendered command without invoking Monte Carlo or changing status.",
    )
    execute.set_defaults(func=cmd_execute)

    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
