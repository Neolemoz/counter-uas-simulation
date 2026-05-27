"""RT→SA manual import handoff helpers (PLAT-RT-SA1)."""

from __future__ import annotations

import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.capture import CaptureBundleError
from rt_sandbox.capture_normalize import validate_normalized_capture
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.export_boundary import (
    CONVERSION_STEPS,
    ORIGIN_RT_SANDBOX_CAPTURE,
    validate_conversion_manifest,
    validate_sa_import_record,
)
from rt_sandbox.isolation import (
    assert_sa_handoff_writable,
    repo_root_from,
    rt_sandbox_captures_dir,
    rt_sandbox_sa_handoff_dir,
)

HANDOFF_GOVERNANCE_BANNER = (
    "HANDOFF REVIEW — explanatory; not lineage authority"
)
HANDOFF_MANIFEST_BANNER = (
    "SA HANDOFF MANIFEST — maintainer pipeline; not replay authority"
)
IMPORT_RECORD_BANNER = (
    "SA IMPORT RECORD — corpus lineage begins here; RT session non-authoritative"
)

HANDOFF_EXPORT_EVENTS = frozenset(
    {
        "handoff_ready",
        "handoff_reviewed",
        "handoff_rejected",
        "handoff_import_deferred",
        "handoff_import_prepared",
        "handoff_import_committed",
    }
)

NON_IMPORTABLE_FAILURE_STATES = frozenset({"failed", "discarded", "runtime_crashed"})


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _write_json(path: Path, data: dict[str, Any], repo_root: Path) -> None:
    assert_sa_handoff_writable(path, repo_root)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def capture_staging_dir(repo_root: Path, capture_id: str) -> Path:
    return rt_sandbox_captures_dir(repo_root) / capture_id


def sa_handoff_dir(repo_root: Path, capture_id: str) -> Path:
    path = rt_sandbox_sa_handoff_dir(repo_root) / capture_id
    path.mkdir(parents=True, exist_ok=True)
    return path


def append_handoff_event(
    repo_root: Path,
    event_type: str,
    *,
    capture_candidate_id: str,
    session_id: str | None = None,
    result: str = "OK",
    detail: dict[str, Any] | None = None,
) -> None:
    if event_type not in HANDOFF_EXPORT_EVENTS:
        raise ValueError(f"unknown handoff event_type: {event_type}")
    ExportAuditLog(repo_root).append(
        event_type,
        capture_candidate_id=capture_candidate_id,
        session_id=session_id,
        result=result,
        detail=detail,
    )


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def check_handoff_preconditions(staging_dir: Path) -> list[str]:
    """Return list of errors; empty if ready for handoff_ready."""
    errors: list[str] = []
    cand_path = staging_dir / "candidate.json"
    if not cand_path.exists():
        return ["missing candidate.json"]
    candidate = _load_json(cand_path)
    if candidate.get("approval_status") == "rejected":
        errors.append("capture rejected")
    if candidate.get("normalization_status") != "normalized":
        errors.append("normalization_status must be normalized")
    norm_errors = validate_normalized_capture(staging_dir)
    if norm_errors:
        errors.extend(norm_errors)
    val_path = staging_dir / "normalization_validation.json"
    if val_path.exists():
        val_doc = _load_json(val_path)
        if not val_doc.get("valid", True):
            errors.append("normalization_validation valid=false")
    report_path = staging_dir / "capture_report.json"
    if report_path.exists():
        report = _load_json(report_path)
        failures = report.get("failure_states_observed") or []
        for state in failures:
            if str(state).lower() in NON_IMPORTABLE_FAILURE_STATES:
                errors.append(f"non-importable failure state: {state}")
    snapshot_path = staging_dir / "snapshot.json"
    if snapshot_path.exists():
        snap = _load_json(snapshot_path)
        lifecycle = str(snap.get("lifecycle_state") or snap.get("state") or "").lower()
        if lifecycle in NON_IMPORTABLE_FAILURE_STATES:
            errors.append(f"non-importable lifecycle in snapshot: {lifecycle}")
    return errors


def is_handoff_blocked(staging_dir: Path) -> bool:
    review_path = staging_dir / "handoff_review.json"
    if review_path.exists():
        review = _load_json(review_path)
        if review.get("decision") in {"rejected", "deferred"}:
            return True
    candidate_path = staging_dir / "candidate.json"
    if candidate_path.exists():
        cand = _load_json(candidate_path)
        if cand.get("approval_status") == "rejected":
            return True
    return False


def write_handoff_review_v1(
    staging_dir: Path,
    *,
    capture_id: str,
    decision: str,
    reviewer: str,
    notes: str | None = None,
    repo_root: Path | None = None,
    also_handoff_dir: Path | None = None,
) -> dict[str, Any]:
    allowed = {"ready_for_approval", "deferred", "rejected"}
    if decision not in allowed:
        raise CaptureBundleError("INVALID_STATE", f"decision must be one of {allowed}")
    root = repo_root or repo_root_from(staging_dir)
    review: dict[str, Any] = {
        "schema": "rt_handoff_review_v1",
        "capture_candidate_id": capture_id,
        "decision": decision,
        "reviewer": reviewer,
        "review_utc": _utc_now(),
        "governance_banner": HANDOFF_GOVERNANCE_BANNER,
    }
    if notes:
        review["notes"] = notes
    from rt_sandbox.capture import _write_json as capture_write

    review_path = staging_dir / "handoff_review.json"
    capture_write(review_path, review, root)
    if also_handoff_dir is not None:
        _write_json(also_handoff_dir / "handoff_review.json", review, root)
    if decision == "rejected":
        cand_path = staging_dir / "candidate.json"
        if cand_path.exists():
            cand = _load_json(cand_path)
            cand["approval_status"] = "rejected"
            capture_write(cand_path, cand, root)
    return review


def require_approved_for_import(staging_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    cand = _load_json(staging_dir / "candidate.json")
    if cand.get("approval_status") != "approved":
        raise CaptureBundleError("INVALID_STATE", "capture must be approved before SA import prepare")
    approval_path = staging_dir / "approval.json"
    conversion_path = staging_dir / "conversion.json"
    if not approval_path.exists():
        raise CaptureBundleError("INVALID_STATE", "missing approval.json")
    if not conversion_path.exists():
        raise CaptureBundleError("INVALID_STATE", "missing conversion.json")
    conversion = _load_json(conversion_path)
    err = validate_conversion_manifest(conversion)
    if err:
        raise CaptureBundleError("INVALID_STATE", f"conversion manifest invalid: {err}")
    return _load_json(approval_path), conversion


def build_handoff_manifest(
    repo_root: Path,
    capture_id: str,
    staging_dir: Path,
    conversion: dict[str, Any],
) -> dict[str, Any]:
    candidate = _load_json(staging_dir / "candidate.json")
    handoff_path = sa_handoff_dir(repo_root, capture_id)
    manifest: dict[str, Any] = {
        "schema": "rt_sa_handoff_manifest_v1",
        "capture_candidate_id": capture_id,
        "ephemeral_session_ref": candidate.get("ephemeral_session_ref")
        or candidate.get("session_id"),
        "origin": ORIGIN_RT_SANDBOX_CAPTURE,
        "governance_banner": HANDOFF_MANIFEST_BANNER,
        "prepared_utc": _utc_now(),
        "capture_staging_dir": str(staging_dir.relative_to(repo_root)),
        "handoff_dir": str(handoff_path.relative_to(repo_root)),
        "staging_refs": dict(conversion.get("staging_refs") or {}),
        "conversion_steps": list(conversion.get("conversion_steps") or CONVERSION_STEPS),
        "conversion_manifest_ref": str((staging_dir / "conversion.json").relative_to(repo_root)),
    }
    if conversion.get("scenario_pack_ref"):
        manifest["scenario_pack_ref"] = conversion["scenario_pack_ref"]
    if conversion.get("log_path"):
        manifest["log_path"] = conversion["log_path"]
    return manifest


def write_handoff_manifest(
    repo_root: Path,
    capture_id: str,
    staging_dir: Path,
    conversion: dict[str, Any],
) -> dict[str, Any]:
    if is_handoff_blocked(staging_dir):
        raise CaptureBundleError("INVALID_STATE", "handoff blocked (rejected or deferred)")
    manifest = build_handoff_manifest(repo_root, capture_id, staging_dir, conversion)
    handoff_path = sa_handoff_dir(repo_root, capture_id)
    out = handoff_path / "handoff_manifest.json"
    _write_json(out, manifest, repo_root)
    return manifest


def _step_dir(handoff_path: Path, step: str) -> Path:
    return handoff_path / "steps" / step


def _record_step_result(
    handoff_path: Path,
    step: str,
    result: dict[str, Any],
    repo_root: Path,
) -> None:
    out = _step_dir(handoff_path, step) / "result.json"
    _write_json(out, result, repo_root)


def run_conversion_step(
    step: str,
    *,
    repo_root: Path,
    handoff_manifest: dict[str, Any],
    handoff_path: Path,
    dry_run: bool = False,
) -> dict[str, Any]:
    if step not in CONVERSION_STEPS:
        raise CaptureBundleError("INVALID_STATE", f"unknown conversion step: {step}")
    step_out = _step_dir(handoff_path, step)
    result: dict[str, Any] = {
        "step": step,
        "dry_run": dry_run,
        "started_utc": _utc_now(),
        "ok": False,
    }
    if dry_run:
        result["ok"] = True
        result["message"] = "dry-run skip"
        result["completed_utc"] = _utc_now()
        _record_step_result(handoff_path, step, result, repo_root)
        return result

    py = subprocess.sys.executable
    eval_dir = repo_root / "scripts" / "evaluation"
    log_dir = step_out / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    try:
        if step == "validate_scenario_pack":
            pack_ref = handoff_manifest.get("scenario_pack_ref")
            if not pack_ref:
                raise CaptureBundleError("INVALID_STATE", "scenario_pack_ref required for validate_scenario_pack")
            pack_path = (repo_root / pack_ref).resolve()
            if not pack_path.is_dir():
                raise CaptureBundleError("INVALID_STATE", f"scenario pack not found: {pack_ref}")
            cmd = [py, str(eval_dir / "validate_scenario.py"), str(pack_path)]
            proc = subprocess.run(cmd, capture_output=True, text=True, cwd=str(repo_root), check=False)
            (log_dir / "stdout.txt").write_text(proc.stdout, encoding="utf-8")
            (log_dir / "stderr.txt").write_text(proc.stderr, encoding="utf-8")
            result["exit_code"] = proc.returncode
            result["ok"] = proc.returncode == 0
        elif step == "replay_observability":
            log_path = handoff_manifest.get("log_path")
            if not log_path:
                result["ok"] = True
                result["skipped"] = True
                result["message"] = "no log_path; step skipped"
            else:
                log_file = (repo_root / log_path).resolve()
                if not log_file.is_file():
                    raise CaptureBundleError("INVALID_STATE", f"log_path not found: {log_path}")
                out_json = step_out / "replay_observability.json"
                cmd = [
                    py,
                    str(eval_dir / "replay_observability.py"),
                    "bundle",
                    str(log_file),
                    "--out-json",
                    str(out_json),
                ]
                proc = subprocess.run(cmd, capture_output=True, text=True, cwd=str(repo_root), check=False)
                (log_dir / "stdout.txt").write_text(proc.stdout, encoding="utf-8")
                (log_dir / "stderr.txt").write_text(proc.stderr, encoding="utf-8")
                result["exit_code"] = proc.returncode
                result["ok"] = proc.returncode == 0
                result["output_json"] = str(out_json.relative_to(repo_root)) if out_json.exists() else None
        elif step == "replay_sa_bundle_pack":
            obs_path = step_out.parent / "replay_observability" / "replay_observability.json"
            if not obs_path.exists():
                raise CaptureBundleError(
                    "INVALID_STATE",
                    "replay_observability step output required before replay_sa_bundle_pack",
                )
            narrative_path = step_out / "narrative.json"
            if not narrative_path.exists():
                n_cmd = [
                    py,
                    str(eval_dir / "replay_observability.py"),
                    "narrative",
                    "--single-run-json",
                    str(obs_path),
                    "--out-json",
                    str(narrative_path),
                ]
                n_proc = subprocess.run(
                    n_cmd, capture_output=True, text=True, cwd=str(repo_root), check=False
                )
                (log_dir / "narrative_stdout.txt").write_text(n_proc.stdout, encoding="utf-8")
                (log_dir / "narrative_stderr.txt").write_text(n_proc.stderr, encoding="utf-8")
                if n_proc.returncode != 0:
                    result["exit_code"] = n_proc.returncode
                    result["ok"] = False
                    result["completed_utc"] = _utc_now()
                    _record_step_result(handoff_path, step, result, repo_root)
                    return result
            bundle_dir = handoff_path / "bundle"
            bundle_dir.mkdir(parents=True, exist_ok=True)
            cmd = [
                py,
                str(eval_dir / "replay_sa_bundle.py"),
                "pack",
                "--narrative-json",
                str(narrative_path),
                "--observability-json",
                str(obs_path),
                "--out-dir",
                str(bundle_dir),
            ]
            pack_ref = handoff_manifest.get("scenario_pack_ref")
            if pack_ref:
                cmd.extend(["--scenario-pack", str((repo_root / pack_ref).resolve())])
            staging_rel = handoff_manifest.get("capture_staging_dir")
            if staging_rel:
                staging_dir = (repo_root / staging_rel).resolve()
                annex_path = staging_dir / "tactical_annex.json"
                norm_path = staging_dir / "normalized_manifest.json"
                has_annex = annex_path.is_file()
                if not has_annex and norm_path.is_file():
                    norm_data = _load_json(norm_path)
                    has_annex = isinstance(norm_data.get("tactical_annex"), dict)
                if has_annex:
                    cmd.extend(["--rt-capture-staging", str(staging_dir)])
            proc = subprocess.run(cmd, capture_output=True, text=True, cwd=str(repo_root), check=False)
            (log_dir / "stdout.txt").write_text(proc.stdout, encoding="utf-8")
            (log_dir / "stderr.txt").write_text(proc.stderr, encoding="utf-8")
            result["exit_code"] = proc.returncode
            result["ok"] = proc.returncode == 0
            index = bundle_dir / "index.json"
            result["bundle_index"] = str(index.relative_to(repo_root)) if index.exists() else None
        elif step == "governance_lint":
            bundle_index = handoff_path / "bundle" / "index.json"
            if not bundle_index.exists():
                raise CaptureBundleError("INVALID_STATE", "bundle index required for governance_lint")
            cmd = [
                py,
                str(eval_dir / "replay_observability.py"),
                "governance-lint",
                str(bundle_index),
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, cwd=str(repo_root), check=False)
            (log_dir / "stdout.txt").write_text(proc.stdout, encoding="utf-8")
            (log_dir / "stderr.txt").write_text(proc.stderr, encoding="utf-8")
            result["exit_code"] = proc.returncode
            result["ok"] = proc.returncode == 0
        else:
            raise CaptureBundleError("INVALID_STATE", f"unhandled step: {step}")
    except CaptureBundleError:
        raise
    except Exception as exc:  # noqa: BLE001
        result["ok"] = False
        result["error"] = str(exc)
    result["completed_utc"] = _utc_now()
    _record_step_result(handoff_path, step, result, repo_root)
    return result


def validate_import_lineage(
    record: dict[str, Any],
    bundle_index: dict[str, Any] | None = None,
) -> str | None:
    err = validate_sa_import_record(record)
    if err:
        return err
    if bundle_index:
        meta = bundle_index.get("metadata") or bundle_index.get("provenance") or {}
        if isinstance(meta, dict):
            parent = meta.get("parent_ref")
            session = record.get("session_id") or (record.get("rt_capture_ref") or {}).get(
                "session_id"
            )
            if parent and session and parent == session:
                return "bundle metadata must not use session_id as parent_ref"
    return None


def write_import_record(
    handoff_path: Path,
    *,
    capture_id: str,
    corpus_dest: Path,
    bundle_rel: str,
    imported_by: str,
    ephemeral_session_ref: str | None,
    repo_root: Path,
) -> dict[str, Any]:
    corpus_ref = str(corpus_dest.relative_to(repo_root))
    record: dict[str, Any] = {
        "schema": "rt_sa_import_record_v1",
        "capture_candidate_id": capture_id,
        "rt_capture_ref": {
            "capture_candidate_id": capture_id,
            "ephemeral_session_ref": ephemeral_session_ref,
            "non_authoritative": True,
        },
        "corpus_ref": corpus_ref,
        "bundle_path": bundle_rel,
        "imported_at": _utc_now(),
        "imported_by": imported_by,
        "governance_banner": IMPORT_RECORD_BANNER,
    }
    err = validate_import_lineage(record)
    if err:
        raise CaptureBundleError("INVALID_STATE", err)
    _write_json(handoff_path / "import_record.json", record, repo_root)
    return record


def commit_bundle_to_corpus(
    repo_root: Path,
    capture_id: str,
    corpus_dest: Path,
    *,
    imported_by: str,
    dry_run: bool = False,
) -> dict[str, Any]:
    from rt_sandbox.export_boundary import assert_maintainer_corpus_write_allowed

    assert_maintainer_corpus_write_allowed(corpus_dest, repo_root)
    handoff_path = sa_handoff_dir(repo_root, capture_id)
    bundle_src = handoff_path / "bundle"
    if not bundle_src.is_dir() or not (bundle_src / "index.json").exists():
        raise CaptureBundleError("INVALID_STATE", "handoff bundle/ with index.json required before commit")
    staging_dir = capture_staging_dir(repo_root, capture_id)
    candidate = _load_json(staging_dir / "candidate.json")
    bundle_rel = str((corpus_dest / "index.json").relative_to(repo_root))
    if dry_run:
        return {
            "dry_run": True,
            "corpus_dest": str(corpus_dest),
            "bundle_src": str(bundle_src),
        }
    if corpus_dest.exists():
        raise CaptureBundleError("INVALID_STATE", f"corpus destination already exists: {corpus_dest}")
    shutil.copytree(bundle_src, corpus_dest)
    record = write_import_record(
        handoff_path,
        capture_id=capture_id,
        corpus_dest=corpus_dest,
        bundle_rel=bundle_rel,
        imported_by=imported_by,
        ephemeral_session_ref=candidate.get("ephemeral_session_ref") or candidate.get("session_id"),
        repo_root=repo_root,
    )
    corpus_record_path = corpus_dest / "rt_sa_import_record.json"
    corpus_record_path.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return record


def handoff_status_summary(repo_root: Path, capture_id: str) -> dict[str, Any]:
    staging = capture_staging_dir(repo_root, capture_id)
    handoff_path = rt_sandbox_sa_handoff_dir(repo_root) / capture_id
    out: dict[str, Any] = {"capture_candidate_id": capture_id, "staging_exists": staging.is_dir()}
    if staging.is_dir() and (staging / "candidate.json").exists():
        out["candidate"] = _load_json(staging / "candidate.json")
    if (staging / "handoff_review.json").exists():
        out["handoff_review"] = _load_json(staging / "handoff_review.json")
    if handoff_path.is_dir():
        manifest_path = handoff_path / "handoff_manifest.json"
        if manifest_path.exists():
            out["handoff_manifest"] = _load_json(manifest_path)
        steps = {}
        steps_dir = handoff_path / "steps"
        if steps_dir.is_dir():
            for step_dir in steps_dir.iterdir():
                res = step_dir / "result.json"
                if res.exists():
                    steps[step_dir.name] = _load_json(res)
        out["steps"] = steps
        if (handoff_path / "import_record.json").exists():
            out["import_record"] = _load_json(handoff_path / "import_record.json")
    export_path = repo_root / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    events: list[dict[str, Any]] = []
    if export_path.is_file():
        for line in export_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            if ev.get("capture_candidate_id") == capture_id and (
                ev.get("event_type", "").startswith("handoff_")
                or ev.get("event_type") in {"capture_approved", "conversion_manifest_written"}
            ):
                events.append(ev)
    out["export_events"] = events[-12:]
    return out


def tail_handoff_events(repo_root: Path, capture_id: str, limit: int = 12) -> list[dict[str, Any]]:
    return handoff_status_summary(repo_root, capture_id).get("export_events", [])[-limit:]
