#!/usr/bin/env python3
"""Offline experiment job orchestration core (PLAT-SA-H3)."""

from __future__ import annotations

import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]
_ORCH = _REPO / "fixtures" / "orchestration"
_AUDITS = _ORCH / "audits"
_QUEUES = _ORCH / "queues"
_MANIFESTS = _ORCH / "manifests"

if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_sa_scenario import lint_scenario_pack  # noqa: E402

ARTIFACT_MANIFEST = "experiment_job_manifest_v1"
ARTIFACT_QUEUE = "experiment_run_queue_v1"
ARTIFACT_REPORT = "experiment_run_report_v1"
ARTIFACT_VALIDATION_MIRROR = "experiment_validation_mirror_v1"

GOVERNANCE_BANNER = "ORCHESTRATION MIRROR — not live execution state"

RUNTIME_CAPTURE_TYPES = frozenset({"runtime_capture"})


def _iso_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def load_manifest(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("artifact_type") != ARTIFACT_MANIFEST:
        raise ValueError(f"expected artifact_type {ARTIFACT_MANIFEST}")
    return data


def lint_manifest(data: dict[str, Any], *, path: str = "") -> list[str]:
    issues: list[str] = []
    prefix = f"{path}: " if path else ""
    for key in ("artifact_type", "schema_version", "manifest_id", "title", "governance", "jobs"):
        if key not in data:
            issues.append(f"{prefix}missing {key}")
    gov = data.get("governance") or {}
    notice = str(gov.get("notice", "")).lower()
    if "not " not in notice and "explanatory" not in notice and "replay" not in notice:
        if notice:
            issues.append(f"{prefix}governance.notice should frame non-authoritative replay use")
    for job in data.get("jobs") or []:
        jid = job.get("job_id", "?")
        if not job.get("scenario_pack_ref"):
            issues.append(f"{prefix}job {jid}: missing scenario_pack_ref")
        if not job.get("pipeline"):
            issues.append(f"{prefix}job {jid}: empty pipeline")
        for step in job.get("pipeline") or []:
            if not step.get("step_type"):
                issues.append(f"{prefix}job {jid}: step missing step_type")
    return issues


def _run_cmd(cmd: list[str], *, dry_run: bool) -> dict[str, Any]:
    t0 = time.monotonic()
    if dry_run:
        return {
            "status": "dry_run",
            "duration_ms": 0,
            "command": " ".join(cmd),
        }
    try:
        subprocess.run(cmd, check=True, cwd=_REPO)
        status = "completed"
        hint = ""
    except subprocess.CalledProcessError as exc:
        status = "failed"
        hint = str(exc)
    return {
        "status": status,
        "duration_ms": int((time.monotonic() - t0) * 1000),
        "command": " ".join(cmd),
        "hint": hint,
    }


def _step_validate_scenario(job: dict[str, Any], step: dict[str, Any], *, dry_run: bool) -> dict[str, Any]:
    pack = _REPO / job["scenario_pack_ref"]
    if dry_run:
        return {
            "step_id": step.get("step_id", "validate"),
            "step_type": "validate_scenario",
            "status": "dry_run",
            "command": f"validate {pack}",
        }
    result = lint_scenario_pack(pack)
    status = "completed" if result.get("ok") else "failed"
    return {
        "step_id": step.get("step_id", "validate"),
        "step_type": "validate_scenario",
        "status": status,
        "duration_ms": 0,
        "command": f"lint_scenario_pack {pack}",
        "hint": "; ".join((result.get("issues") or [])[:3]),
        "validation": result,
    }


def _step_validation_mirror(
    job: dict[str, Any],
    step: dict[str, Any],
    *,
    dry_run: bool,
    manifest_id: str,
) -> dict[str, Any]:
    pack = _REPO / job["scenario_pack_ref"]
    out_dir = _ORCH / "validation_mirrors"
    pack_id = job.get("scenario_pack_id", "unknown")
    out_path = out_dir / f"{pack_id}_validation_mirror.json"
    rec: dict[str, Any] = {
        "step_id": step.get("step_id", "validation_mirror"),
        "step_type": "validation_mirror",
        "status": "dry_run" if dry_run else "completed",
        "command": f"write mirror {out_path}",
    }
    if dry_run:
        return rec
    result = lint_scenario_pack(pack)
    mirror = {
        "artifact_type": ARTIFACT_VALIDATION_MIRROR,
        "schema_version": ARTIFACT_VALIDATION_MIRROR,
        "manifest_id": manifest_id,
        "job_id": job.get("job_id"),
        "scenario_pack_id": pack_id,
        "scenario_pack_ref": job.get("scenario_pack_ref"),
        "checked_at": _iso_now(),
        "ok": bool(result.get("ok")),
        "issues": result.get("issues") or [],
        "warnings": result.get("warnings") or [],
        "governance_banner": GOVERNANCE_BANNER,
    }
    _write_json(out_path, mirror)
    rec["artifact_refs"] = [str(out_path.relative_to(_REPO))]
    return rec


def _step_subprocess_script(
    script: str,
    args: list[str],
    step: dict[str, Any],
    *,
    dry_run: bool,
) -> dict[str, Any]:
    cmd = [sys.executable, str(_EVAL / script), *args]
    out = _run_cmd(cmd, dry_run=dry_run)
    out["step_id"] = step.get("step_id", script)
    out["step_type"] = step.get("step_type", script)
    return out


def _step_runtime_capture(job: dict[str, Any], step: dict[str, Any], *, dry_run: bool, allow: bool) -> dict[str, Any]:
    if not allow:
        return {
            "step_id": step.get("step_id", "capture"),
            "step_type": "runtime_capture",
            "status": "skipped",
            "hint": "runtime_capture requires --allow-runtime-capture",
            "command": "run_capture.py (blocked)",
        }
    scenario = step.get("capture_scenario") or "single"
    timeout = str(step.get("timeout_s", 120))
    cmd = [
        sys.executable,
        str(_REPO / "scripts" / "run_capture.py"),
        "--scenario",
        scenario,
        "--timeout-s",
        timeout,
        "--launch-args",
        "use_gazebo_gui:=false",
    ]
    out = _run_cmd(cmd, dry_run=dry_run)
    out["step_id"] = step.get("step_id", "capture")
    out["step_type"] = "runtime_capture"
    return out


def execute_step(
    job: dict[str, Any],
    step: dict[str, Any],
    *,
    dry_run: bool,
    allow_runtime_capture: bool,
    manifest_id: str,
    corpus_regen_dry_run: bool,
) -> dict[str, Any]:
    if step.get("enabled") is False:
        return {
            "step_id": step.get("step_id", "?"),
            "step_type": step.get("step_type", "?"),
            "status": "skipped",
            "command": "(disabled)",
        }

    stype = step.get("step_type", "")
    extra = list(step.get("args") or [])

    if stype == "validate_scenario":
        return _step_validate_scenario(job, step, dry_run=dry_run)
    if stype == "validation_mirror":
        return _step_validation_mirror(job, step, dry_run=dry_run, manifest_id=manifest_id)
    if stype == "synthetic_demo_regen" or stype == "catalog_sync":
        return _step_subprocess_script("sync_sa_catalog.py", extra, step, dry_run=dry_run)
    if stype == "corpus_regen":
        args = ["--dry-run"] if corpus_regen_dry_run or dry_run else []
        args.extend(extra)
        return _step_subprocess_script("run_replay_corpus_regen.py", args, step, dry_run=dry_run)
    if stype == "bundle_verify":
        bundle_dir = _REPO / (job.get("outputs") or {}).get("bundle_dir", "")
        if dry_run:
            return {
                "step_id": step.get("step_id", "bundle_verify"),
                "step_type": stype,
                "status": "dry_run",
                "command": f"verify {bundle_dir}/index.json",
            }
        ok = (bundle_dir / "index.json").is_file()
        return {
            "step_id": step.get("step_id", "bundle_verify"),
            "step_type": stype,
            "status": "completed" if ok else "failed",
            "hint": "" if ok else f"missing {bundle_dir}/index.json",
            "artifact_refs": [str((bundle_dir / "index.json").relative_to(_REPO))] if ok else [],
        }
    if stype == "runtime_capture":
        return _step_runtime_capture(job, step, dry_run=dry_run, allow=allow_runtime_capture)

    return {
        "step_id": step.get("step_id", "?"),
        "step_type": stype,
        "status": "failed",
        "hint": f"unknown step_type: {stype}",
    }


def run_job(
    job: dict[str, Any],
    *,
    dry_run: bool,
    allow_runtime_capture: bool,
    manifest_id: str,
    manifest_ref: str,
    corpus_regen_dry_run: bool = True,
) -> dict[str, Any]:
    job_id = job.get("job_id", "unknown")
    started = _iso_now()
    job_rec: dict[str, Any] = {
        "job_id": job_id,
        "scenario_pack_id": job.get("scenario_pack_id"),
        "status": "running",
        "phase": "",
        "started_at": started,
        "finished_at": "",
        "artifact_refs": [],
        "error_hint": "",
        "provenance": {
            "scenario_pack_ref": job.get("scenario_pack_ref"),
            "bundle_path": (job.get("outputs") or {}).get("bundle_dir"),
            "corpus_ref": (job.get("outputs") or {}).get("corpus_entry_id"),
        },
    }
    steps_out: list[dict[str, Any]] = []

    for step in job.get("pipeline") or []:
        stype = step.get("step_type", "")
        if stype in RUNTIME_CAPTURE_TYPES and not allow_runtime_capture and not dry_run:
            job_rec["status"] = "failed"
            job_rec["error_hint"] = "runtime_capture blocked without --allow-runtime-capture"
            job_rec["finished_at"] = _iso_now()
            return {"job": job_rec, "steps": steps_out}

        job_rec["phase"] = step.get("step_id", stype)
        rec = execute_step(
            job,
            step,
            dry_run=dry_run,
            allow_runtime_capture=allow_runtime_capture,
            manifest_id=manifest_id,
            corpus_regen_dry_run=corpus_regen_dry_run,
        )
        steps_out.append(rec)
        for ref in rec.get("artifact_refs") or []:
            if ref not in job_rec["artifact_refs"]:
                job_rec["artifact_refs"].append(ref)
        if rec.get("status") == "failed":
            job_rec["status"] = "failed"
            job_rec["error_hint"] = rec.get("hint") or rec.get("step_type", "")
            job_rec["finished_at"] = _iso_now()
            return {"job": job_rec, "steps": steps_out}

    job_rec["status"] = "dry_run" if dry_run else "completed"
    job_rec["finished_at"] = _iso_now()
    return {"job": job_rec, "steps": steps_out}


def run_manifest(
    manifest_path: Path,
    *,
    dry_run: bool = False,
    job_filter: str | None = None,
    allow_runtime_capture: bool = False,
    allow_async_worker: bool = False,
    write_queue: bool = True,
    queue_id: str | None = None,
) -> dict[str, Any]:
    manifest = load_manifest(manifest_path)
    issues = lint_manifest(manifest, path=str(manifest_path))
    if issues:
        raise ValueError("; ".join(issues))

    manifest_id = manifest["manifest_id"]
    manifest_ref = str(manifest_path.relative_to(_REPO))
    qid = queue_id or f"{manifest_id}_queue"

    jobs_in = manifest.get("jobs") or []
    if job_filter:
        jobs_in = [j for j in jobs_in if j.get("job_id") == job_filter]
        if not jobs_in:
            raise ValueError(f"job not found: {job_filter}")

    all_jobs: list[dict[str, Any]] = []
    all_steps: list[dict[str, Any]] = []

    for job in jobs_in:
        result = run_job(
            job,
            dry_run=dry_run,
            allow_runtime_capture=allow_runtime_capture,
            manifest_id=manifest_id,
            manifest_ref=manifest_ref,
        )
        all_jobs.append(result["job"])
        all_steps.extend(result["steps"])

    queue = {
        "artifact_type": ARTIFACT_QUEUE,
        "schema_version": ARTIFACT_QUEUE,
        "queue_id": qid,
        "created_at": _iso_now(),
        "manifest_ref": manifest_ref,
        "manifest_id": manifest_id,
        "governance_banner": GOVERNANCE_BANNER,
        "dry_run": dry_run,
        "jobs": all_jobs,
        "steps": all_steps,
    }

    async_worker_meta = {
        "allow_async_worker": allow_async_worker,
        "worker_provenance": "enabled" if allow_async_worker else "blocked",
        "hint": (
            "use record_async_execution.py for worker records"
            if allow_async_worker
            else "async worker metadata blocked without --allow-async-worker"
        ),
    }
    report = {
        "artifact_type": ARTIFACT_REPORT,
        "schema_version": ARTIFACT_REPORT,
        "manifest_id": manifest_id,
        "manifest_ref": manifest_ref,
        "queue_id": qid,
        "dry_run": dry_run,
        "governance_banner": GOVERNANCE_BANNER,
        "async_worker_governance": async_worker_meta,
        "jobs": all_jobs,
        "steps": all_steps,
    }

    if write_queue:
        _write_json(_QUEUES / f"{qid}.json", queue)
        _write_json(_AUDITS / f"{qid}_report.json", report)

    return {"queue": queue, "report": report}
