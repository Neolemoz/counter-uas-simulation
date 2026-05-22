#!/usr/bin/env python3
"""Orchestration operations manifest and promotion helpers (PLAT-SA-I1)."""

from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from experiment_orchestration import (  # noqa: E402
    ARTIFACT_MANIFEST,
    ARTIFACT_QUEUE,
    ARTIFACT_REPORT,
    _QUEUES,
    _AUDITS,
    _MANIFESTS,
    lint_manifest,
    load_manifest,
)

_REPO = Path(__file__).resolve().parents[2]
_ORCH = _REPO / "fixtures" / "orchestration"
_OPS = _ORCH / "ops"
_SCENARIOS = _REPO / "fixtures" / "scenarios"

OPS_ARTIFACT_TYPE = "experiment_orchestration_ops_manifest_v1"
OPS_SCHEMA_VERSION = "1"
OPS_SUMMARY_FILENAME = "orchestration_operations_summary_v1.json"

OPERATIONS_STATUSES = (
    "pending",
    "validated",
    "queued",
    "executed",
    "replay_generated",
    "archived",
)
OPERATIONS_RANK = {s: i for i, s in enumerate(OPERATIONS_STATUSES)}
TERMINAL_STATUSES = frozenset({"archived"})

GOVERNANCE_DEFAULT: dict[str, Any] = {
    "notice": "Orchestration operations sidecar for offline experiment workflow only.",
    "anti_claims": [
        "not live execution authority",
        "not operational deployment state",
        "not parser contract",
    ],
}


def _iso_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def manifest_path_from_id(manifest_id: str) -> Path | None:
    for path in sorted(_MANIFESTS.glob("*.json")):
        try:
            data = load_manifest(path)
        except (json.JSONDecodeError, ValueError):
            continue
        if data.get("manifest_id") == manifest_id:
            return path
    return None


def manifest_path_for_file(manifest_file: Path) -> Path:
    return manifest_file.resolve()


def ops_path(manifest_id: str) -> Path:
    return _OPS / f"{manifest_id}_ops.json"


def compute_manifest_fingerprint(manifest_file: Path) -> str:
    h = hashlib.sha256()
    h.update(manifest_file.read_bytes())
    return f"sha256:{h.hexdigest()}"


def scenario_pack_ids_from_manifest(manifest: dict[str, Any]) -> list[str]:
    return [str(j.get("scenario_pack_id")) for j in (manifest.get("jobs") or []) if j.get("scenario_pack_id")]


def default_queue_id(manifest_id: str) -> str:
    if manifest_id.endswith("_validation_only"):
        return f"{manifest_id.removesuffix('_validation_only')}_validation_only_queue"
    return f"{manifest_id}_queue"


def queue_rel(manifest_id: str, queue_id: str | None = None) -> str:
    qid = queue_id or default_queue_id(manifest_id)
    return f"fixtures/orchestration/queues/{qid}.json"


def audit_rel(queue_id: str) -> str:
    return f"fixtures/orchestration/audits/{queue_id}_report.json"


def build_ops_draft(manifest_file: Path, *, operations_status: str = "pending") -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    return {
        "artifact_type": OPS_ARTIFACT_TYPE,
        "schema_version": OPS_SCHEMA_VERSION,
        "manifest_id": manifest_id,
        "scenario_pack_ids": scenario_pack_ids_from_manifest(manifest),
        "operations_status": operations_status,
        "operations_lineage": [],
        "authoring_pack_refs": scenario_pack_ids_from_manifest(manifest),
        "governance": dict(GOVERNANCE_DEFAULT),
    }


def load_ops_manifest(manifest_id: str) -> dict[str, Any] | None:
    path = ops_path(manifest_id)
    if not path.is_file():
        return None
    return _read_json(path)


def lint_ops_manifest(manifest_id: str, *, strict: bool = False) -> dict[str, Any]:
    issues: list[str] = []
    warnings: list[str] = []
    path = ops_path(manifest_id)
    if not path.is_file():
        return {"ok": True, "issues": [], "warnings": ["no ops sidecar (optional until init)"]}

    try:
        ops = _read_json(path)
    except (json.JSONDecodeError, OSError) as exc:
        return {"ok": False, "issues": [str(exc)], "warnings": []}

    if ops.get("artifact_type") != OPS_ARTIFACT_TYPE:
        issues.append(f"artifact_type must be {OPS_ARTIFACT_TYPE!r}")
    if str(ops.get("schema_version")) != OPS_SCHEMA_VERSION:
        issues.append(f"schema_version must be {OPS_SCHEMA_VERSION!r}")
    if ops.get("manifest_id") != manifest_id:
        issues.append(f"manifest_id mismatch: {ops.get('manifest_id')!r} vs {manifest_id!r}")

    status = ops.get("operations_status")
    if status not in OPERATIONS_STATUSES:
        issues.append(f"invalid operations_status: {status!r}")

    mf = manifest_path_from_id(manifest_id)
    if not mf:
        issues.append(f"no job manifest for manifest_id {manifest_id!r}")
    elif status in ("validated", "queued", "executed", "replay_generated"):
        if not ops.get("manifest_fingerprint"):
            msg = f"{manifest_id}: ops at {status} without manifest_fingerprint"
            (issues if strict else warnings).append(msg)

    return {"ok": len(issues) == 0, "issues": issues, "warnings": warnings}


def is_manifest_stale(ops: dict[str, Any], manifest_file: Path) -> bool:
    fp = ops.get("manifest_fingerprint")
    if not fp:
        return True
    return fp != compute_manifest_fingerprint(manifest_file)


def record_validation(manifest_file: Path, *, dry_run: bool = False) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    lint_issues = lint_manifest(manifest, path=str(manifest_file))
    if lint_issues:
        return {"ok": False, "error": "manifest lint failed", "issues": lint_issues}

    pack_issues: list[str] = []
    for pack_id in scenario_pack_ids_from_manifest(manifest):
        mirror = _ORCH / "validation_mirrors" / f"{pack_id}_validation_mirror.json"
        if not mirror.is_file():
            pack_issues.append(f"missing validation mirror: {mirror}")

    if pack_issues:
        return {"ok": False, "error": "validation mirrors missing", "issues": pack_issues}

    fp = compute_manifest_fingerprint(manifest_file)
    ops = load_ops_manifest(manifest_id) or build_ops_draft(manifest_file, operations_status="validated")
    ops["manifest_fingerprint"] = fp
    ops["operations_status"] = "validated"
    ops["updated_at"] = _iso_now()
    ops["scenario_pack_ids"] = scenario_pack_ids_from_manifest(manifest)

    if dry_run:
        return {"ok": True, "dry_run": True, "manifest_id": manifest_id, "ops": ops}

    _write_json(ops_path(manifest_id), ops)
    return {"ok": True, "manifest_id": manifest_id, "operations_status": "validated", "manifest_fingerprint": fp}


def record_queue(
    manifest_file: Path,
    *,
    queue_id: str | None = None,
    dry_run: bool = True,
) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    qid = queue_id or default_queue_id(manifest_id)
    queue_path = _REPO / queue_rel(manifest_id, qid)

    if not queue_path.is_file():
        return {"ok": False, "error": f"queue snapshot missing: {queue_path}"}

    ops = load_ops_manifest(manifest_id) or build_ops_draft(manifest_file)
    rank = OPERATIONS_RANK.get(str(ops.get("operations_status")), 0)
    if rank < OPERATIONS_RANK["validated"]:
        return {"ok": False, "error": "ops must be validated before queued"}

    ops["queue_snapshot_ref"] = queue_rel(manifest_id, qid)
    ops["audit_report_ref"] = audit_rel(qid)
    ops["manifest_fingerprint"] = compute_manifest_fingerprint(manifest_file)
    ops["operations_status"] = "queued" if dry_run else "executed"
    ops["updated_at"] = _iso_now()

    if dry_run:
        return {"ok": True, "dry_run": True, "manifest_id": manifest_id, "queue_id": qid, "ops": ops}

    _write_json(ops_path(manifest_id), ops)
    return {"ok": True, "manifest_id": manifest_id, "operations_status": ops["operations_status"], "queue_id": qid}


def verify_replay_outputs(manifest: dict[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    all_ok = True
    for job in manifest.get("jobs") or []:
        outputs = job.get("outputs") or {}
        bundle_dir = outputs.get("bundle_dir")
        pack_id = outputs.get("viewer_demo_pack_id") or job.get("scenario_pack_id")
        entry: dict[str, Any] = {"job_id": job.get("job_id"), "pack_id": pack_id}
        if bundle_dir:
            bundle_path = _REPO / bundle_dir
            index_path = bundle_path / "index.json"
            entry["bundle_dir"] = bundle_dir
            entry["bundle_exists"] = bundle_path.is_dir()
            entry["index_exists"] = index_path.is_file()
            if not entry["bundle_exists"] or not entry["index_exists"]:
                all_ok = False
        else:
            demo = _REPO / "fixtures" / "sa_r0" / f"demo_{pack_id}"
            entry["bundle_dir"] = str(demo.relative_to(_REPO))
            entry["bundle_exists"] = demo.is_dir()
            entry["index_exists"] = (demo / "index.json").is_file()
            if not entry["bundle_exists"]:
                all_ok = False
        checks.append(entry)
    return {"ok": all_ok, "checks": checks}


def promote_experiment_manifest(
    manifest_file: Path,
    *,
    target_status: str,
    notes: str = "",
    dry_run: bool = False,
    allow_stale: bool = False,
    record_validation_first: bool = False,
    record_queue_dry_run: bool = False,
) -> dict[str, Any]:
    if target_status not in OPERATIONS_STATUSES:
        return {"ok": False, "error": f"invalid operations_status: {target_status}"}

    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    ops = load_ops_manifest(manifest_id) or build_ops_draft(manifest_file)
    from_status = str(ops.get("operations_status") or "pending")

    try:
        import replay_sa_orchestration_async as orch_async  # noqa: E402

        if orch_async.is_quarantined(manifest_id):
            return {
                "ok": False,
                "error": "async_execution_status quarantined; clear via CLI audit",
            }
    except ImportError:
        pass

    if from_status in TERMINAL_STATUSES and target_status != from_status:
        return {"ok": False, "error": f"terminal status {from_status} blocks transition to {target_status}"}

    if record_validation_first and OPERATIONS_RANK[target_status] >= OPERATIONS_RANK["validated"]:
        rec = record_validation(manifest_file, dry_run=dry_run)
        if not rec.get("ok"):
            return rec
        ops = load_ops_manifest(manifest_id) or ops

    mf = manifest_path_for_file(manifest_file)
    if is_manifest_stale(ops, mf) and not allow_stale:
        if OPERATIONS_RANK.get(from_status, 0) >= OPERATIONS_RANK["validated"]:
            return {"ok": False, "error": "manifest_fingerprint stale; re-validate or use --allow-stale"}

    if target_status == "queued":
        rec = record_queue(manifest_file, dry_run=True)
        if not rec.get("ok"):
            return rec

    if target_status in ("executed", "replay_generated") and record_queue_dry_run:
        rec = record_queue(manifest_file, dry_run=True)
        if not rec.get("ok"):
            return rec

    if target_status == "replay_generated":
        replay = verify_replay_outputs(manifest)
        if not replay.get("ok"):
            return {"ok": False, "error": "replay outputs not verified", "replay": replay}

    to_rank = OPERATIONS_RANK[target_status]
    from_rank = OPERATIONS_RANK.get(from_status, 0)
    if to_rank < from_rank and target_status not in TERMINAL_STATUSES:
        return {"ok": False, "error": f"cannot downgrade {from_status} -> {target_status}"}

    event = {
        "event_id": f"ops-{_iso_now()}",
        "from_status": from_status,
        "to_status": target_status,
        "actor": "cli:promote_experiment_manifest",
        "notes": notes,
    }
    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "manifest_id": manifest_id,
            "from_status": from_status,
            "operations_status": target_status,
            "event": event,
        }

    ops["operations_status"] = target_status
    ops["updated_at"] = _iso_now()
    ops["operations_lineage"] = list(ops.get("operations_lineage") or []) + [event]
    if target_status in ("validated", "queued", "executed", "replay_generated"):
        ops["manifest_fingerprint"] = compute_manifest_fingerprint(mf)
    if target_status == "queued":
        qid = default_queue_id(manifest_id)
        ops["queue_snapshot_ref"] = queue_rel(manifest_id, qid)
        ops["audit_report_ref"] = audit_rel(qid)

    _write_json(ops_path(manifest_id), ops)
    summary = write_operations_summary(manifest_file, ops)
    return {
        "ok": True,
        "manifest_id": manifest_id,
        "from_status": from_status,
        "operations_status": target_status,
        "summary_path": str(summary.relative_to(_REPO)) if summary else "",
    }


def write_operations_summary(manifest_file: Path, ops: dict[str, Any]) -> Path | None:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    summary = {
        "artifact_type": "orchestration_operations_summary_v1",
        "schema_version": "1",
        "manifest_id": manifest_id,
        "operations_status": ops.get("operations_status"),
        "generated_at": _iso_now(),
        "scenario_pack_ids": ops.get("scenario_pack_ids") or scenario_pack_ids_from_manifest(manifest),
        "queue_snapshot_ref": ops.get("queue_snapshot_ref"),
        "governance_banner": "OPERATIONS SUMMARY — explanatory; CLI authoritative",
    }
    out = _OPS / f"{manifest_id}_{OPS_SUMMARY_FILENAME}"
    _write_json(out, summary)
    ops_path(manifest_id)
    ops_data = load_ops_manifest(manifest_id)
    if ops_data:
        ops_data["operations_summary_ref"] = str(out.relative_to(_REPO))
        _write_json(ops_path(manifest_id), ops_data)
    return out


def repro_check(manifest_file: Path) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    issues: list[str] = []
    lint_issues = lint_manifest(manifest, path=str(manifest_file))
    issues.extend(lint_issues)
    for pack_id in scenario_pack_ids_from_manifest(manifest):
        mirror = _ORCH / "validation_mirrors" / f"{pack_id}_validation_mirror.json"
        if not mirror.is_file():
            issues.append(f"missing validation mirror: {pack_id}")
    fp = compute_manifest_fingerprint(manifest_file)
    ops = load_ops_manifest(manifest_id)
    if ops and ops.get("manifest_fingerprint") and ops["manifest_fingerprint"] != fp:
        issues.append("manifest_fingerprint stale vs ops sidecar")
    return {"ok": len(issues) == 0, "manifest_id": manifest_id, "issues": issues, "fingerprint": fp}


def execution_lineage_report(manifest_file: Path) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    ops = load_ops_manifest(manifest_id) or {}
    queue_data: dict[str, Any] | None = None
    qref = ops.get("queue_snapshot_ref")
    if qref:
        qp = _REPO / qref
        if qp.is_file():
            queue_data = _read_json(qp)
    return {
        "manifest_id": manifest_id,
        "operations_status": ops.get("operations_status"),
        "jobs": manifest.get("jobs"),
        "queue_snapshot_ref": qref,
        "queue": queue_data,
        "audit_report_ref": ops.get("audit_report_ref"),
        "operations_lineage": ops.get("operations_lineage") or [],
    }


def replay_continuity_summary(manifest_file: Path) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    replay = verify_replay_outputs(manifest)
    ops = load_ops_manifest(str(manifest["manifest_id"])) or {}
    return {
        "manifest_id": manifest["manifest_id"],
        "operations_status": ops.get("operations_status"),
        "replay": replay,
    }


def check_stale(manifest_file: Path) -> dict[str, Any]:
    manifest_id = str(load_manifest(manifest_file)["manifest_id"])
    ops = load_ops_manifest(manifest_id)
    if not ops:
        return {"stale": False, "reason": "no ops sidecar"}
    stale = is_manifest_stale(ops, manifest_path_for_file(manifest_file))
    return {"stale": stale, "manifest_id": manifest_id, "stored": ops.get("manifest_fingerprint")}


def list_all_manifest_files() -> list[Path]:
    return sorted(_MANIFESTS.glob("*.json"))


def init_ops_for_manifest(manifest_file: Path, *, dry_run: bool = False) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    if ops_path(manifest_id).is_file() and not dry_run:
        return {"ok": False, "error": "ops sidecar already exists"}
    ops = build_ops_draft(manifest_file)
    if dry_run:
        return {"ok": True, "dry_run": True, "ops": ops}
    _write_json(ops_path(manifest_id), ops)
    return {"ok": True, "manifest_id": manifest_id, "path": str(ops_path(manifest_id))}
