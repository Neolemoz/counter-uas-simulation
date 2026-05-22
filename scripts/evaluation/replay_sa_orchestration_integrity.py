#!/usr/bin/env python3
"""Corpus-wide orchestration integrity checks (PLAT-SA-I1)."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_ops as orch_ops  # noqa: E402
import replay_sa_authoring as authoring  # noqa: E402
from experiment_orchestration import load_manifest, lint_manifest  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_ORCH = _REPO / "fixtures" / "orchestration"
_SCENARIOS = _REPO / "fixtures" / "scenarios"
_CATALOG = _SCENARIOS / "index.json"
_VIEWER_ORCH = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "orchestration"

INTEGRITY_ARTIFACT_TYPE = "orchestration_integrity_report_v1"

VALIDATION_ONLY_SUFFIX = "_validation_only"


def _iso_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def catalog_pack_ids() -> list[str]:
    if not _CATALOG.is_file():
        return []
    data = _read_json(_CATALOG)
    return [str(p.get("pack_id")) for p in (data.get("packs") or []) if p.get("pack_id")]


def _orch_index_ids(kind: str) -> set[str]:
    index_path = _VIEWER_ORCH / "index.json"
    if not index_path.is_file():
        index_path = _ORCH / "index.json"
    if not index_path.is_file():
        return set()
    data = _read_json(index_path)
    ids: set[str] = set()
    for entry in data.get("entries") or []:
        if isinstance(entry, dict) and entry.get("kind") == kind and entry.get("id"):
            ids.add(str(entry["id"]))
    return ids


def manifest_for_pack(pack_id: str) -> Path | None:
    expected = f"{pack_id}_validation.json"
    path = _ORCH / "manifests" / expected
    if path.is_file():
        return path
    for mf in orch_ops.list_all_manifest_files():
        try:
            m = load_manifest(mf)
        except (json.JSONDecodeError, ValueError):
            continue
        for job in m.get("jobs") or []:
            if job.get("scenario_pack_id") == pack_id:
                return mf
    return None


def audit_manifest_file(
    manifest_file: Path,
    *,
    catalog_ids: set[str],
    strict: bool,
    repro_strict: bool = False,
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    manifest = load_manifest(manifest_file)
    manifest_id = str(manifest.get("manifest_id", manifest_file.stem))
    pack_ids = orch_ops.scenario_pack_ids_from_manifest(manifest)

    lint_issues = lint_manifest(manifest, path=str(manifest_file))
    errors.extend(lint_issues)

    for pack_id in pack_ids:
        if pack_id not in catalog_ids:
            errors.append(f"{manifest_id}: job pack {pack_id} not in catalog")
        mirror = _ORCH / "validation_mirrors" / f"{pack_id}_validation_mirror.json"
        if not mirror.is_file():
            msg = f"{manifest_id}: missing validation mirror for {pack_id}"
            (errors if strict else warnings).append(msg)

    ops = orch_ops.load_ops_manifest(manifest_id)
    if not ops:
        msg = f"{manifest_id}: missing ops sidecar"
        (errors if strict else warnings).append(msg)
    else:
        lint_ops = orch_ops.lint_ops_manifest(manifest_id, strict=strict)
        errors.extend(lint_ops.get("issues") or [])
        warnings.extend(lint_ops.get("warnings") or [])
        if orch_ops.is_manifest_stale(ops, manifest_file):
            msg = f"{manifest_id}: manifest_fingerprint stale"
            (errors if strict else warnings).append(msg)

    qid = orch_ops.default_queue_id(manifest_id)
    queue_path = _ORCH / "queues" / f"{qid}.json"
    if not queue_path.is_file():
        msg = f"{manifest_id}: missing queue snapshot {qid}"
        (errors if strict else warnings).append(msg)
    else:
        queue = _read_json(queue_path)
        mref = str(queue.get("manifest_ref") or queue.get("manifest_id") or "")
        if mref and manifest_id not in mref and mref != manifest_id:
            warnings.append(f"{manifest_id}: queue manifest_ref mismatch")
        for job in queue.get("jobs") or []:
            jpid = job.get("scenario_pack_id")
            if jpid and jpid not in pack_ids:
                errors.append(f"{manifest_id}: queue job pack {jpid} not in manifest")

    audit_path = _ORCH / "audits" / f"{qid}_report.json"
    if not audit_path.is_file():
        warnings.append(f"{manifest_id}: missing audit report for {qid}")

    if strict and manifest_id.endswith(VALIDATION_ONLY_SUFFIX):
        for pack_id in pack_ids:
            auth = authoring.load_authoring_manifest(_SCENARIOS / pack_id)
            if auth:
                for ref_obj in auth.get("orchestration_handoff_refs") or []:
                    if not isinstance(ref_obj, dict):
                        continue
                    mid = ref_obj.get("manifest_id")
                    qmid = ref_obj.get("queue_mirror_id")
                    if mid and mid != manifest_id and mid not in (manifest_id,):
                        pass
                    if mid == manifest_id and qmid and qmid != qid:
                        warnings.append(f"{pack_id}: handoff queue_mirror_id {qmid} != {qid}")

    manifest_ids_index = _orch_index_ids("manifests")
    queue_ids_index = _orch_index_ids("queues")
    if manifest_ids_index and manifest_id not in manifest_ids_index:
        warnings.append(f"{manifest_id}: not in orchestration viewer index (run sync)")
    if queue_ids_index and qid not in queue_ids_index:
        warnings.append(f"{manifest_id}: queue {qid} not in viewer index")

    for pack_id in pack_ids:
        auth = authoring.load_authoring_manifest(_SCENARIOS / pack_id)
        if not auth:
            continue
        for ref_obj in auth.get("orchestration_handoff_refs") or []:
            if not isinstance(ref_obj, dict):
                continue
            mid = ref_obj.get("manifest_id")
            qmid = ref_obj.get("queue_mirror_id")
            if mid and mid not in manifest_ids_index and manifest_ids_index:
                warnings.append(f"{pack_id}: handoff manifest_id not in index: {mid}")
            if qmid and qmid not in queue_ids_index and queue_ids_index:
                warnings.append(f"{pack_id}: handoff queue_mirror_id not in index: {qmid}")

    replay = orch_ops.verify_replay_outputs(manifest)
    if manifest.get("jobs") and any((j.get("outputs") or {}).get("bundle_dir") for j in manifest["jobs"]):
        if not replay.get("ok"):
            warnings.append(f"{manifest_id}: replay bundle outputs incomplete")

    return {
        "manifest_id": manifest_id,
        "pack_ids": pack_ids,
        "errors": errors,
        "warnings": warnings,
    }


def audit_pack_coverage(pack_id: str, *, catalog_ids: set[str], strict: bool) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    if pack_id not in catalog_ids:
        errors.append(f"{pack_id}: not in catalog")
        return {"pack_id": pack_id, "errors": errors, "warnings": warnings}

    mf = manifest_for_pack(pack_id)
    if not mf:
        errors.append(f"{pack_id}: no orchestration job manifest")
        return {"pack_id": pack_id, "errors": errors, "warnings": warnings}

    mirror = _ORCH / "validation_mirrors" / f"{pack_id}_validation_mirror.json"
    if not mirror.is_file():
        errors.append(f"{pack_id}: missing validation mirror")

    result = audit_manifest_file(mf, catalog_ids=catalog_ids, strict=strict)
    errors.extend(result.get("errors") or [])
    warnings.extend(result.get("warnings") or [])
    return {"pack_id": pack_id, "errors": errors, "warnings": warnings}


def run_integrity_audit(
    *,
    strict: bool = False,
    pipeline_filter: str | None = None,
    repro_strict: bool = False,
) -> dict[str, Any]:
    catalog_ids = set(catalog_pack_ids())
    all_errors: list[str] = []
    all_warnings: list[str] = []
    per_manifest: dict[str, Any] = {}
    per_pack: dict[str, Any] = {}

    manifest_files = orch_ops.list_all_manifest_files()
    seen_errors: set[str] = set()
    seen_warnings: set[str] = set()

    def _add_errors(errs: list[str]) -> None:
        for e in errs:
            if e not in seen_errors:
                seen_errors.add(e)
                all_errors.append(e)

    def _add_warnings(warns: list[str]) -> None:
        for w in warns:
            if w not in seen_warnings:
                seen_warnings.add(w)
                all_warnings.append(w)

    for mf in manifest_files:
        manifest = load_manifest(mf)
        mid = str(manifest["manifest_id"])
        is_validation_only = mid.endswith(VALIDATION_ONLY_SUFFIX) or "validation" in mf.name
        is_synthetic = "synthetic" in mf.name
        if pipeline_filter == "validation-only" and not is_validation_only:
            continue
        if pipeline_filter == "full" and is_validation_only and not is_synthetic:
            continue
        if mid == "capture_pipeline_template":
            continue

        result = audit_manifest_file(mf, catalog_ids=catalog_ids, strict=strict, repro_strict=repro_strict)
        per_manifest[mid] = result
        _add_errors(result.get("errors") or [])
        _add_warnings(result.get("warnings") or [])

    for pack_id in sorted(catalog_ids):
        result = audit_pack_coverage(pack_id, catalog_ids=catalog_ids, strict=strict)
        per_pack[pack_id] = result
        _add_errors(result.get("errors") or [])
        _add_warnings(result.get("warnings") or [])

    orphan_queues = []
    queues_dir = _ORCH / "queues"
    manifest_ids = set()
    for mf in manifest_files:
        try:
            manifest_ids.add(str(load_manifest(mf)["manifest_id"]))
        except (json.JSONDecodeError, ValueError):
            pass
    for qp in sorted(queues_dir.glob("*.json")):
        qid = qp.stem
        expected_manifest = None
        for mid in manifest_ids:
            if orch_ops.default_queue_id(mid) == qid:
                expected_manifest = mid
                break
        if not expected_manifest:
            msg = f"orphan queue: {qid}"
            (orphan_queues.append(msg))
            if strict:
                all_errors.append(msg)
            else:
                all_warnings.append(msg)

    ok = len(all_errors) == 0
    return {
        "artifact_type": INTEGRITY_ARTIFACT_TYPE,
        "schema_version": "1",
        "checked_at": _iso_now(),
        "strict": strict,
        "ok": ok,
        "pack_count": len(catalog_ids),
        "manifest_count": len(per_manifest),
        "errors": all_errors,
        "warnings": all_warnings,
        "per_manifest": per_manifest,
        "per_pack": per_pack,
        "orphan_queues": orphan_queues,
        "governance_banner": "ORCHESTRATION INTEGRITY — explanatory audit; CLI remains authoritative",
    }


def lineage_continuity_report(manifest_id: str) -> dict[str, Any]:
    mf = orch_ops.manifest_path_from_id(manifest_id)
    if not mf:
        return {"ok": False, "error": f"unknown manifest_id: {manifest_id}"}
    manifest = load_manifest(mf)
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    packs = orch_ops.scenario_pack_ids_from_manifest(manifest)
    authoring_refs: list[dict[str, Any]] = []
    for pack_id in packs:
        auth = authoring.load_authoring_manifest(_SCENARIOS / pack_id)
        if auth:
            authoring_refs.append(
                {
                    "pack_id": pack_id,
                    "promotion_status": auth.get("promotion_status"),
                    "handoff_refs": auth.get("orchestration_handoff_refs") or [],
                }
            )
    return {
        "manifest_id": manifest_id,
        "operations_status": ops.get("operations_status"),
        "execution_lineage": orch_ops.execution_lineage_report(mf),
        "authoring_refs": authoring_refs,
        "replay_continuity": orch_ops.replay_continuity_summary(mf),
    }
