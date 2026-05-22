#!/usr/bin/env python3
"""Async orchestration bookkeeping, fingerprints, and integrity (PLAT-SA-I2)."""

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

import replay_sa_orchestration_ops as orch_ops  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_ORCH = _REPO / "fixtures" / "orchestration"
_ASYNC = _ORCH / "async"
_CLAIMS = _ORCH / "claims"
_WORKERS = _ORCH / "workers"

ASYNC_ARTIFACT_TYPE = "experiment_orchestration_async_manifest_v1"
ASYNC_SCHEMA_VERSION = "1"
ASYNC_SUMMARY_FILENAME = "async_execution_summary_v1.json"

CLAIM_ARTIFACT_TYPE = "queue_claim_token_v1"
CLAIM_SCHEMA_VERSION = "1"

WORKER_ARTIFACT_TYPE = "worker_execution_record_v1"
WORKER_SCHEMA_VERSION = "1"

ASYNC_INTEGRITY_ARTIFACT_TYPE = "orchestration_async_integrity_report_v1"

ASYNC_EXECUTION_STATUSES = frozenset({"retrying", "failed", "quarantined", "superseded"})
TERMINAL_ASYNC_STATUSES = frozenset({"failed", "quarantined", "superseded"})
CLAIM_OPEN = "open"
CLAIM_CLOSED = "closed"

GOVERNANCE_DEFAULT: dict[str, Any] = {
    "notice": "Async orchestration sidecar for offline experiment bookkeeping only.",
    "anti_claims": [
        "not live execution authority",
        "not distributed worker registry",
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


def _sha256_bytes(data: bytes) -> str:
    return f"sha256:{hashlib.sha256(data).hexdigest()}"


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _canonical_json(obj: Any) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":")).encode("utf-8")


def async_path(manifest_id: str) -> Path:
    return _ASYNC / f"{manifest_id}_async.json"


def claim_path(queue_id: str) -> Path:
    return _CLAIMS / f"{queue_id}_claim.json"


def worker_path(worker_id: str, execution_attempt: int) -> Path:
    safe_id = worker_id.replace("/", "_")
    return _WORKERS / f"{safe_id}_{execution_attempt}.json"


def build_async_draft(manifest_id: str) -> dict[str, Any]:
    return {
        "artifact_type": ASYNC_ARTIFACT_TYPE,
        "schema_version": ASYNC_SCHEMA_VERSION,
        "manifest_id": manifest_id,
        "async_lineage": [],
        "worker_execution_refs": [],
        "governance": dict(GOVERNANCE_DEFAULT),
    }


def load_async_manifest(manifest_id: str) -> dict[str, Any] | None:
    path = async_path(manifest_id)
    if not path.is_file():
        return None
    return _read_json(path)


def lint_async_manifest(manifest_id: str, *, strict: bool = False) -> dict[str, Any]:
    issues: list[str] = []
    warnings: list[str] = []
    path = async_path(manifest_id)
    if not path.is_file():
        return {"ok": True, "issues": [], "warnings": ["no async sidecar (optional)"]}

    try:
        async_m = _read_json(path)
    except (json.JSONDecodeError, OSError) as exc:
        return {"ok": False, "issues": [str(exc)], "warnings": []}

    if async_m.get("artifact_type") != ASYNC_ARTIFACT_TYPE:
        issues.append(f"artifact_type must be {ASYNC_ARTIFACT_TYPE!r}")
    if str(async_m.get("schema_version")) != ASYNC_SCHEMA_VERSION:
        issues.append(f"schema_version must be {ASYNC_SCHEMA_VERSION!r}")
    if async_m.get("manifest_id") != manifest_id:
        issues.append(f"manifest_id mismatch: {async_m.get('manifest_id')!r}")

    status = async_m.get("async_execution_status")
    if status is not None and status not in ASYNC_EXECUTION_STATUSES:
        issues.append(f"invalid async_execution_status: {status!r}")

    gov = async_m.get("governance")
    if not isinstance(gov, dict) or not gov.get("notice"):
        (issues if strict else warnings).append(f"{manifest_id}: missing governance.notice")

    for ref in async_m.get("worker_execution_refs") or []:
        wp = _REPO / ref
        if not wp.is_file():
            (issues if strict else warnings).append(f"{manifest_id}: missing worker ref {ref}")

    cref = async_m.get("queue_claim_ref")
    if cref:
        cp = _REPO / cref
        if not cp.is_file():
            (issues if strict else warnings).append(f"{manifest_id}: missing claim ref {cref}")

    return {"ok": len(issues) == 0, "issues": issues, "warnings": warnings}


def is_quarantined(manifest_id: str) -> bool:
    async_m = load_async_manifest(manifest_id)
    if not async_m:
        return False
    return async_m.get("async_execution_status") == "quarantined"


def set_async_status(
    manifest_id: str,
    status: str,
    *,
    notes: str = "",
    dry_run: bool = False,
) -> dict[str, Any]:
    if status not in ASYNC_EXECUTION_STATUSES:
        return {"ok": False, "error": f"invalid async_execution_status: {status}"}

    async_m = load_async_manifest(manifest_id) or build_async_draft(manifest_id)
    from_status = async_m.get("async_execution_status")
    event = {
        "event_id": f"async-{_iso_now()}",
        "from_status": from_status,
        "to_status": status,
        "actor": "cli:record_async_execution",
        "notes": notes,
    }
    if dry_run:
        preview = dict(async_m)
        preview["async_execution_status"] = status
        preview["async_lineage"] = list(async_m.get("async_lineage") or []) + [event]
        return {"ok": True, "dry_run": True, "manifest_id": manifest_id, "async": preview}

    async_m["async_execution_status"] = status
    async_m["updated_at"] = _iso_now()
    async_m["async_lineage"] = list(async_m.get("async_lineage") or []) + [event]
    _write_json(async_path(manifest_id), async_m)
    return {"ok": True, "manifest_id": manifest_id, "async_execution_status": status}


def compute_queue_snapshot_hash(queue_path: Path) -> str:
    return _sha256_file(queue_path)


def _step_outcome_hashes(report: dict[str, Any] | None) -> dict[str, str]:
    if not report:
        return {}
    out: dict[str, str] = {}
    for step in report.get("steps") or []:
        sid = step.get("step_id")
        if not sid:
            continue
        payload = {
            "step_id": sid,
            "step_type": step.get("step_type"),
            "status": step.get("status"),
            "command": step.get("command"),
        }
        out[str(sid)] = _sha256_bytes(_canonical_json(payload))
    return out


def _manifest_output_refs(manifest: dict[str, Any]) -> list[str]:
    refs: list[str] = []
    for job in manifest.get("jobs") or []:
        outputs = job.get("outputs") or {}
        for key in ("bundle_dir", "corpus_entry_id", "viewer_demo_pack_id"):
            val = outputs.get(key)
            if val:
                refs.append(f"{key}:{val}")
    return sorted(refs)


def compute_execution_fingerprint(
    manifest_file: Path,
    queue_path: Path,
    report_path: Path | None = None,
    *,
    ops: dict[str, Any] | None = None,
) -> str:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    manifest_hash = orch_ops.compute_manifest_fingerprint(manifest_file)
    queue_hash = compute_queue_snapshot_hash(queue_path)
    report = _read_json(report_path) if report_path and report_path.is_file() else None
    step_hashes = _step_outcome_hashes(report)
    payload = {
        "manifest_id": manifest_id,
        "manifest_fingerprint": manifest_hash,
        "queue_snapshot_hash": queue_hash,
        "step_outcome_hashes": step_hashes,
        "output_refs": _manifest_output_refs(manifest),
    }
    if ops and ops.get("operations_status") == "replay_generated":
        payload["operations_status"] = "replay_generated"
    return _sha256_bytes(_canonical_json(payload))


def compute_replay_fingerprint(manifest: dict[str, Any]) -> str | None:
    for job in manifest.get("jobs") or []:
        outputs = job.get("outputs") or {}
        bundle_dir = outputs.get("bundle_dir")
        pack_id = outputs.get("viewer_demo_pack_id") or job.get("scenario_pack_id")
        if bundle_dir:
            index_path = _REPO / bundle_dir / "index.json"
        elif pack_id:
            index_path = _REPO / "fixtures" / "sa_r0" / f"demo_{pack_id}" / "index.json"
        else:
            continue
        if index_path.is_file():
            return _sha256_file(index_path)
    return None


def verify_replay_reproducibility(
    manifest_file: Path,
    *,
    ops: dict[str, Any] | None = None,
    async_m: dict[str, Any] | None = None,
) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    ops = ops or orch_ops.load_ops_manifest(manifest_id)
    async_m = async_m or load_async_manifest(manifest_id)
    current_replay_fp = compute_replay_fingerprint(manifest)
    issues: list[str] = []
    stored = None
    if async_m:
        stored = async_m.get("replay_fingerprint")
    if ops and ops.get("operations_status") == "replay_generated" and current_replay_fp:
        if stored and stored != current_replay_fp:
            issues.append("replay_fingerprint stale vs bundle index.json")
        if not stored:
            issues.append("replay_generated ops without async replay_fingerprint")
    return {
        "ok": len(issues) == 0,
        "manifest_id": manifest_id,
        "current_replay_fingerprint": current_replay_fp,
        "stored_replay_fingerprint": stored,
        "issues": issues,
    }


def _resolve_queue_report(manifest_id: str, ops: dict[str, Any] | None) -> tuple[Path | None, Path | None]:
    ops = ops or orch_ops.load_ops_manifest(manifest_id) or {}
    qref = ops.get("queue_snapshot_ref")
    if not qref:
        qid = orch_ops.default_queue_id(manifest_id)
        qref = orch_ops.queue_rel(manifest_id, qid)
    queue_path = _REPO / qref
    if not queue_path.is_file():
        return None, None
    qid = queue_path.stem
    aref = ops.get("audit_report_ref") or orch_ops.audit_rel(qid)
    report_path = _REPO / aref
    return queue_path, report_path if report_path.is_file() else None


def queue_execution_continuity(manifest_file: Path) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    async_m = load_async_manifest(manifest_id)
    queue_path, report_path = _resolve_queue_report(manifest_id, ops)
    claim: dict[str, Any] | None = None
    workers: list[dict[str, Any]] = []
    if async_m:
        cref = async_m.get("queue_claim_ref")
        if cref and (_REPO / cref).is_file():
            claim = _read_json(_REPO / cref)
        for wref in async_m.get("worker_execution_refs") or []:
            wp = _REPO / wref
            if wp.is_file():
                workers.append(_read_json(wp))
    fp = None
    if queue_path and queue_path.is_file():
        fp = compute_execution_fingerprint(
            manifest_file, queue_path, report_path, ops=ops
        )
    return {
        "manifest_id": manifest_id,
        "operations_status": ops.get("operations_status"),
        "async_execution_status": async_m.get("async_execution_status") if async_m else None,
        "queue_snapshot_ref": ops.get("queue_snapshot_ref"),
        "queue_snapshot_hash": compute_queue_snapshot_hash(queue_path) if queue_path and queue_path.is_file() else None,
        "execution_fingerprint": async_m.get("execution_fingerprint") if async_m else fp,
        "computed_execution_fingerprint": fp,
        "claim": claim,
        "worker_records": workers,
        "async_lineage": async_m.get("async_lineage") if async_m else [],
    }


def build_claim_token(
    manifest_file: Path,
    *,
    worker_id: str,
    queue_id: str | None = None,
    dry_run: bool = False,
) -> dict[str, Any]:
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    qid = queue_id or orch_ops.default_queue_id(manifest_id)
    queue_path = _REPO / orch_ops.queue_rel(manifest_id, qid)
    if not queue_path.is_file():
        return {"ok": False, "error": f"queue snapshot missing: {queue_path}"}

    claim_id = f"claim-{manifest_id}-{qid}"
    token = {
        "artifact_type": CLAIM_ARTIFACT_TYPE,
        "schema_version": CLAIM_SCHEMA_VERSION,
        "claim_id": claim_id,
        "manifest_id": manifest_id,
        "queue_id": qid,
        "snapshot_hash": compute_queue_snapshot_hash(queue_path),
        "manifest_fingerprint": orch_ops.compute_manifest_fingerprint(manifest_file),
        "worker_id": worker_id,
        "claim_status": CLAIM_OPEN,
        "queue_snapshot_ref": orch_ops.queue_rel(manifest_id, qid),
        "claimed_at": _iso_now(),
        "governance": {
            "notice": "Queue claim token for offline async bookkeeping only.",
            "anti_claims": ["not distributed lock", "not parser contract"],
        },
    }
    if dry_run:
        return {"ok": True, "dry_run": True, "claim": token}

    out = claim_path(qid)
    _write_json(out, token)
    async_m = load_async_manifest(manifest_id) or build_async_draft(manifest_id)
    async_m["queue_claim_ref"] = str(out.relative_to(_REPO))
    async_m["queue_snapshot_hash"] = token["snapshot_hash"]
    async_m["updated_at"] = _iso_now()
    _write_json(async_path(manifest_id), async_m)
    return {"ok": True, "claim_id": claim_id, "path": str(out.relative_to(_REPO))}


def build_worker_record(
    manifest_file: Path,
    *,
    worker_id: str,
    execution_attempt: int = 1,
    retry_parent_ref: str | None = None,
    dry_run: bool = False,
    allow_runtime_capture: bool = False,
    allow_async_worker: bool = True,
) -> dict[str, Any]:
    if not allow_async_worker:
        return {"ok": False, "error": "allow_async_worker required for worker records"}

    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest["manifest_id"])
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    async_m = load_async_manifest(manifest_id)
    if not async_m or not async_m.get("queue_claim_ref"):
        return {"ok": False, "error": "async sidecar missing queue_claim_ref; run --claim first"}

    claim = _read_json(_REPO / async_m["queue_claim_ref"])
    queue_path, report_path = _resolve_queue_report(manifest_id, ops)
    if not queue_path or not queue_path.is_file():
        return {"ok": False, "error": "queue snapshot missing"}

    step_hashes = _step_outcome_hashes(_read_json(report_path) if report_path else None)
    exec_fp = compute_execution_fingerprint(
        manifest_file, queue_path, report_path, ops=ops
    )
    retry_lineage: list[dict[str, Any]] = []
    if retry_parent_ref and (_REPO / retry_parent_ref).is_file():
        parent = _read_json(_REPO / retry_parent_ref)
        retry_lineage.append(
            {
                "worker_id": parent.get("worker_id"),
                "execution_attempt": parent.get("execution_attempt"),
                "ref": retry_parent_ref,
            }
        )

    record = {
        "artifact_type": WORKER_ARTIFACT_TYPE,
        "schema_version": WORKER_SCHEMA_VERSION,
        "worker_id": worker_id,
        "execution_attempt": execution_attempt,
        "manifest_id": manifest_id,
        "claim_id": claim.get("claim_id"),
        "snapshot_hash": claim.get("snapshot_hash"),
        "manifest_fingerprint": claim.get("manifest_fingerprint"),
        "started_at": _iso_now(),
        "finished_at": _iso_now(),
        "retry_lineage": retry_lineage,
        "retry_parent_ref": retry_parent_ref,
        "step_outcome_hashes": step_hashes,
        "execution_fingerprint": exec_fp,
        "deterministic_metadata": {
            "dry_run": True,
            "allow_runtime_capture": allow_runtime_capture,
            "allow_async_worker": allow_async_worker,
        },
        "governance": {
            "notice": "Worker execution record for offline provenance only.",
            "anti_claims": ["not live worker registry", "not parser contract"],
        },
    }
    if dry_run:
        return {"ok": True, "dry_run": True, "record": record}

    out = worker_path(worker_id, execution_attempt)
    _write_json(out, record)
    refs = list(async_m.get("worker_execution_refs") or [])
    rel = str(out.relative_to(_REPO))
    if rel not in refs:
        refs.append(rel)
    async_m["worker_execution_refs"] = refs
    async_m["execution_fingerprint"] = exec_fp
    async_m["updated_at"] = _iso_now()
    _write_json(async_path(manifest_id), async_m)
    close_claim(async_m["queue_claim_ref"], terminal_reason="worker_record_closed", dry_run=False)
    return {"ok": True, "path": rel, "execution_fingerprint": exec_fp}


def close_claim(
    claim_ref: str,
    *,
    terminal_reason: str = "closed",
    dry_run: bool = False,
) -> dict[str, Any]:
    cp = _REPO / claim_ref
    if not cp.is_file():
        return {"ok": False, "error": f"claim not found: {claim_ref}"}
    claim = _read_json(cp)
    if claim.get("claim_status") == CLAIM_CLOSED:
        return {"ok": True, "already_closed": True}
    if dry_run:
        return {"ok": True, "dry_run": True, "claim_id": claim.get("claim_id")}
    claim["claim_status"] = CLAIM_CLOSED
    claim["closed_at"] = _iso_now()
    claim["terminal_reason"] = terminal_reason
    _write_json(cp, claim)
    return {"ok": True, "claim_id": claim.get("claim_id")}


def init_async_for_manifest(manifest_file: Path, *, dry_run: bool = False) -> dict[str, Any]:
    manifest_id = str(load_manifest(manifest_file)["manifest_id"])
    if async_path(manifest_id).is_file() and not dry_run:
        return {"ok": False, "error": "async sidecar already exists"}
    draft = build_async_draft(manifest_id)
    if dry_run:
        return {"ok": True, "dry_run": True, "async": draft}
    _write_json(async_path(manifest_id), draft)
    return {"ok": True, "manifest_id": manifest_id, "path": str(async_path(manifest_id).relative_to(_REPO))}


def store_execution_fingerprint(manifest_file: Path, *, dry_run: bool = False) -> dict[str, Any]:
    manifest_id = str(load_manifest(manifest_file)["manifest_id"])
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    queue_path, report_path = _resolve_queue_report(manifest_id, ops)
    if not queue_path or not queue_path.is_file():
        return {"ok": False, "error": "queue snapshot missing"}
    fp = compute_execution_fingerprint(manifest_file, queue_path, report_path, ops=ops)
    replay_fp = compute_replay_fingerprint(load_manifest(manifest_file))
    async_m = load_async_manifest(manifest_id) or build_async_draft(manifest_id)
    async_m["execution_fingerprint"] = fp
    async_m["queue_snapshot_hash"] = compute_queue_snapshot_hash(queue_path)
    if replay_fp:
        async_m["replay_fingerprint"] = replay_fp
    async_m["updated_at"] = _iso_now()
    if dry_run:
        return {"ok": True, "dry_run": True, "execution_fingerprint": fp, "replay_fingerprint": replay_fp}
    _write_json(async_path(manifest_id), async_m)
    return {
        "ok": True,
        "manifest_id": manifest_id,
        "execution_fingerprint": fp,
        "replay_fingerprint": replay_fp,
    }


def write_async_execution_summary(manifest_file: Path) -> Path | None:
    manifest_id = str(load_manifest(manifest_file)["manifest_id"])
    async_m = load_async_manifest(manifest_id)
    if not async_m:
        return None
    summary = {
        "artifact_type": "async_execution_summary_v1",
        "schema_version": "1",
        "manifest_id": manifest_id,
        "async_execution_status": async_m.get("async_execution_status"),
        "execution_fingerprint": async_m.get("execution_fingerprint"),
        "generated_at": _iso_now(),
        "governance_banner": "ASYNC EXECUTION SUMMARY — explanatory; CLI authoritative",
    }
    out = _ASYNC / f"{manifest_id}_{ASYNC_SUMMARY_FILENAME}"
    _write_json(out, summary)
    async_m["async_execution_summary_ref"] = str(out.relative_to(_REPO))
    _write_json(async_path(manifest_id), async_m)
    return out


def _list_async_manifest_ids() -> list[str]:
    ids: list[str] = []
    if not _ASYNC.is_dir():
        return ids
    for path in sorted(_ASYNC.glob("*_async.json")):
        try:
            data = _read_json(path)
            if data.get("manifest_id"):
                ids.append(str(data["manifest_id"]))
        except (json.JSONDecodeError, OSError):
            pass
    return ids


def _audit_claim_file(claim_path_file: Path, *, strict: bool) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    try:
        claim = _read_json(claim_path_file)
    except (json.JSONDecodeError, OSError) as exc:
        return [{"kind": "invalid_claim", "manifest_id": "", "message": str(exc)}]

    manifest_id = str(claim.get("manifest_id") or "")
    qref = claim.get("queue_snapshot_ref")
    if qref:
        qp = _REPO / qref
        if not qp.is_file():
            issues.append(
                {
                    "kind": "orphan_claim",
                    "manifest_id": manifest_id,
                    "message": f"claim references missing queue: {qref}",
                }
            )
        else:
            expected = claim.get("snapshot_hash")
            actual = compute_queue_snapshot_hash(qp)
            if expected and expected != actual:
                issues.append(
                    {
                        "kind": "stale_snapshot",
                        "manifest_id": manifest_id,
                        "message": "claim snapshot_hash mismatch vs queue file",
                    }
                )

    if claim.get("claim_status") == CLAIM_OPEN:
        async_m = load_async_manifest(manifest_id) if manifest_id else None
        if async_m and async_m.get("async_execution_status") == "superseded":
            issues.append(
                {
                    "kind": "queue_reconciliation",
                    "manifest_id": manifest_id,
                    "message": "open claim on superseded async snapshot",
                }
            )

    return issues


def _collect_open_claims() -> dict[tuple[str, str], list[str]]:
    open_claims: dict[tuple[str, str], list[str]] = {}
    if not _CLAIMS.is_dir():
        return open_claims
    for cp in sorted(_CLAIMS.glob("*_claim.json")):
        try:
            claim = _read_json(cp)
        except (json.JSONDecodeError, OSError):
            continue
        if claim.get("claim_status") != CLAIM_OPEN:
            continue
        key = (str(claim.get("snapshot_hash") or ""), str(claim.get("manifest_id") or ""))
        open_claims.setdefault(key, []).append(str(claim.get("claim_id") or cp.stem))
    return open_claims


def audit_async_manifest(manifest_id: str, *, strict: bool) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []

    async_m = load_async_manifest(manifest_id)
    if not async_m:
        return {"manifest_id": manifest_id, "issues": issues, "warnings": warnings}

    lint = lint_async_manifest(manifest_id, strict=strict)
    for msg in lint.get("issues") or []:
        issues.append({"kind": "lint", "manifest_id": manifest_id, "message": msg})
    for msg in lint.get("warnings") or []:
        warnings.append({"kind": "lint", "manifest_id": manifest_id, "message": msg})

    mf = orch_ops.manifest_path_from_id(manifest_id)
    if mf:
        repro = verify_replay_reproducibility(mf, async_m=async_m)
        for msg in repro.get("issues") or []:
            issues.append({"kind": "stale_replay", "manifest_id": manifest_id, "message": msg})

        ops = orch_ops.load_ops_manifest(manifest_id) or {}
        if ops.get("operations_status") == "replay_generated":
            cref = async_m.get("queue_claim_ref")
            if cref and not (async_m.get("worker_execution_refs") or []):
                msg = "replay_generated with claim but no worker_execution_refs"
                (issues if strict else warnings).append(
                    {"kind": "replay_continuity", "manifest_id": manifest_id, "message": msg}
                )

        stored_fp = async_m.get("execution_fingerprint")
        queue_path, report_path = _resolve_queue_report(manifest_id, ops)
        if queue_path and queue_path.is_file() and stored_fp:
            computed = compute_execution_fingerprint(mf, queue_path, report_path, ops=ops)
            if stored_fp != computed:
                issues.append(
                    {
                        "kind": "execution_fingerprint",
                        "manifest_id": manifest_id,
                        "message": "stored execution_fingerprint stale vs frozen inputs",
                    }
                )

    for step_issue in _partial_execution_issues(manifest_id):
        issues.append(step_issue)

    return {"manifest_id": manifest_id, "issues": issues, "warnings": warnings}


def _partial_execution_issues(manifest_id: str) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    qref = ops.get("queue_snapshot_ref")
    if not qref:
        return issues
    queue = _read_json(_REPO / qref)
    qid = queue.get("queue_id") or Path(qref).stem
    report_path = _REPO / (ops.get("audit_report_ref") or orch_ops.audit_rel(str(qid)))
    report = _read_json(report_path) if report_path.is_file() else None
    for step in (queue.get("steps") or []):
        if step.get("status") == "running":
            rid = step.get("step_id")
            terminal = False
            if report:
                for rs in report.get("steps") or []:
                    if rs.get("step_id") == rid and rs.get("status") in ("completed", "failed", "dry_run", "skipped"):
                        terminal = True
                        break
            if not terminal:
                issues.append(
                    {
                        "kind": "partial_execution",
                        "manifest_id": manifest_id,
                        "message": f"step {rid} running without terminal report closure",
                    }
                )
    return issues


def run_async_integrity_audit(*, strict: bool = False) -> dict[str, Any]:
    all_issues: list[dict[str, Any]] = []
    all_warnings: list[dict[str, Any]] = []
    per_manifest: dict[str, Any] = {}

    for manifest_id in _list_async_manifest_ids():
        result = audit_async_manifest(manifest_id, strict=strict)
        per_manifest[manifest_id] = result
        all_issues.extend(result.get("issues") or [])
        all_warnings.extend(result.get("warnings") or [])

    if _CLAIMS.is_dir():
        for cp in sorted(_CLAIMS.glob("*_claim.json")):
            all_issues.extend(_audit_claim_file(cp, strict=strict))

    open_claims = _collect_open_claims()
    for key, claim_ids in open_claims.items():
        if len(claim_ids) > 1:
            all_issues.append(
                {
                    "kind": "duplicate_claim",
                    "manifest_id": key[1],
                    "message": f"duplicate open claims: {claim_ids}",
                }
            )

    for cp in sorted(_CLAIMS.glob("*_claim.json")):
        try:
            claim = _read_json(cp)
        except (json.JSONDecodeError, OSError):
            continue
        if claim.get("claim_status") != CLAIM_OPEN:
            continue
        manifest_id = str(claim.get("manifest_id") or "")
        qref = claim.get("queue_snapshot_ref")
        if qref and not (_REPO / qref).is_file():
            all_issues.append(
                {
                    "kind": "orphan_claim",
                    "manifest_id": manifest_id,
                    "message": f"orphan open claim without queue: {qref}",
                }
            )
        ops = orch_ops.load_ops_manifest(manifest_id) if manifest_id else None
        if ops:
            aref = ops.get("audit_report_ref")
            if aref and not (_REPO / aref).is_file():
                all_issues.append(
                    {
                        "kind": "orphan_claim",
                        "manifest_id": manifest_id,
                        "message": f"claim without audit report: {aref}",
                    }
                )

    ok = len(all_issues) == 0
    return {
        "artifact_type": ASYNC_INTEGRITY_ARTIFACT_TYPE,
        "schema_version": "1",
        "checked_at": _iso_now(),
        "strict": strict,
        "ok": ok,
        "issues": all_issues,
        "warnings": all_warnings,
        "per_manifest": per_manifest,
        "governance": {
            "notice": "Explanatory audit only. Not parser-visible authority.",
            "anti_claims": ["not operational readiness"],
        },
        "governance_banner": "ASYNC ORCHESTRATION INTEGRITY — explanatory audit; CLI remains authoritative",
    }
