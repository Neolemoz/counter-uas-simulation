#!/usr/bin/env python3
"""Corpus-wide scenario authoring integrity checks (PLAT-SA-A2)."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_authoring as authoring  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_SCENARIOS = _REPO / "fixtures" / "scenarios"
_CATALOG = _SCENARIOS / "index.json"
_VIEWER_AUTH = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "authoring"
_ORCH_INDEX = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "orchestration" / "index.json"

INTEGRITY_ARTIFACT_TYPE = "scenario_authoring_integrity_report_v1"


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
    if not _ORCH_INDEX.is_file():
        return set()
    data = _read_json(_ORCH_INDEX)
    ids: set[str] = set()
    for entry in data.get("entries") or []:
        if isinstance(entry, dict) and entry.get("kind") == kind and entry.get("id"):
            ids.add(str(entry["id"]))
    return ids


def _orch_manifest_ids() -> set[str]:
    return _orch_index_ids("manifests")


def _orch_queue_ids() -> set[str]:
    return _orch_index_ids("queues")


def check_stale_validation(pack_id: str, *, strict: bool) -> list[str]:
    issues: list[str] = []
    pack_dir = _SCENARIOS / pack_id
    manifest = authoring.load_authoring_manifest(pack_dir)
    if not manifest:
        return issues
    if authoring.is_validation_stale(manifest, pack_dir):
        msg = f"{pack_id}: validation_pack_fingerprint stale"
        (issues if strict else []).append(msg) if strict else None
        if not strict:
            return [msg]  # warning path handled by caller
        issues.append(msg)
    return issues


def audit_pack(
    pack_id: str,
    *,
    catalog_ids: set[str],
    strict: bool,
) -> dict[str, Any]:
    pack_dir = _SCENARIOS / pack_id
    errors: list[str] = []
    warnings: list[str] = []

    manifest_path = authoring.manifest_path(pack_dir)
    has_manifest = manifest_path.is_file()
    in_catalog = pack_id in catalog_ids

    if in_catalog and not has_manifest:
        msg = f"{pack_id}: catalog pack missing authoring_manifest.json"
        if strict:
            errors.append(msg)
        else:
            warnings.append(msg)
        return {"pack_id": pack_id, "errors": errors, "warnings": warnings}

    if has_manifest and not in_catalog:
        errors.append(f"{pack_id}: orphan manifest (not in catalog)")

    if not has_manifest:
        return {"pack_id": pack_id, "errors": errors, "warnings": warnings}

    manifest = authoring._read_json(manifest_path)  # noqa: SLF001
    lint = authoring.lint_authoring_manifest(pack_dir, strict=strict)
    errors.extend(lint.get("issues") or [])
    warnings.extend(lint.get("warnings") or [])

    status = manifest.get("promotion_status") or "draft"
    rank = authoring.PROMOTION_RANK.get(str(status), 0)
    if rank >= authoring.PROMOTION_RANK["promoted"]:
        if not manifest.get("validation_pack_fingerprint"):
            errors.append(f"{pack_id}: promoted without validation_pack_fingerprint")
        if not manifest.get("validation_snapshot_ref"):
            warnings.append(f"{pack_id}: promoted without validation_snapshot_ref")
        if authoring.is_validation_stale(manifest, pack_dir):
            msg = f"{pack_id}: promoted but fingerprint stale"
            if strict:
                errors.append(msg)
            else:
                warnings.append(msg)

    ref = manifest.get("validation_snapshot_ref")
    if ref:
        ref_path = _REPO / ref
        if ref_path.is_file():
            mirror = _read_json(ref_path)
            if mirror.get("scenario_pack_id") and mirror.get("scenario_pack_id") != pack_id:
                errors.append(f"{pack_id}: validation mirror pack_id mismatch")
            if not mirror.get("checked_at"):
                warnings.append(f"{pack_id}: validation mirror missing checked_at")
        elif strict:
            errors.append(f"{pack_id}: validation_snapshot_ref missing file: {ref}")

    parent = manifest.get("parent_pack_id")
    if parent:
        if not (_SCENARIOS / str(parent)).is_dir():
            errors.append(f"{pack_id}: parent_pack_id directory missing: {parent}")
        chain = authoring._collect_parent_chain(pack_id)  # noqa: SLF001
        if len(chain) != len(set(chain)):
            errors.append(f"{pack_id}: parent chain cycle")

    meta_path = pack_dir / "metadata.json"
    if meta_path.is_file() and parent:
        meta = _read_json(meta_path)
        baseline = (meta.get("provenance") or {}).get("baseline_pack_id")
        variant = manifest.get("topology_variant_class")
        if variant in ("sensor_layout_experiment", "topology_layout_experiment") and not baseline:
            warnings.append(f"{pack_id}: experiment missing metadata.provenance.baseline_pack_id")

    viewer_mirror = _VIEWER_AUTH / f"{pack_id}.json"
    if not viewer_mirror.is_file():
        warnings.append(f"{pack_id}: viewer authoring mirror missing (run sync_authoring_mirrors)")
    elif viewer_mirror.is_file():
        try:
            vm = _read_json(viewer_mirror)
            if vm.get("pack_id") != pack_id:
                errors.append(f"{pack_id}: viewer mirror pack_id mismatch")
        except (json.JSONDecodeError, OSError) as exc:
            errors.append(f"{pack_id}: viewer mirror unreadable: {exc}")

    manifest_ids = _orch_manifest_ids()
    queue_ids = _orch_queue_ids()
    for ref_obj in manifest.get("orchestration_handoff_refs") or []:
        if not isinstance(ref_obj, dict):
            continue
        mid = ref_obj.get("manifest_id")
        qid = ref_obj.get("queue_mirror_id")
        if mid and mid not in manifest_ids:
            warnings.append(f"{pack_id}: handoff manifest_id not in orchestration index: {mid}")
        if qid and qid not in queue_ids:
            warnings.append(f"{pack_id}: handoff queue_mirror_id not in orchestration index: {qid}")

    return {"pack_id": pack_id, "errors": errors, "warnings": warnings}


def run_integrity_audit(*, strict: bool = False) -> dict[str, Any]:
    catalog_ids = set(catalog_pack_ids())
    packs = sorted(catalog_ids)
    orphan_dirs = [
        p.name
        for p in _SCENARIOS.iterdir()
        if p.is_dir() and (p / authoring.MANIFEST_FILENAME).is_file() and p.name not in catalog_ids
    ]

    all_errors: list[str] = []
    all_warnings: list[str] = []
    per_pack: dict[str, Any] = {}

    for pack_id in orphan_dirs:
        all_errors.append(f"{pack_id}: orphan manifest (not in catalog)")

    for pack_id in packs:
        result = audit_pack(pack_id, catalog_ids=catalog_ids, strict=strict)
        per_pack[pack_id] = result
        all_errors.extend(result.get("errors") or [])
        all_warnings.extend(result.get("warnings") or [])

    for path in sorted(_SCENARIOS.glob("*/authoring_manifest.json")):
        pid = path.parent.name
        if pid not in catalog_ids:
            continue

    ok = len(all_errors) == 0
    return {
        "artifact_type": INTEGRITY_ARTIFACT_TYPE,
        "schema_version": "1",
        "checked_at": _iso_now(),
        "strict": strict,
        "ok": ok,
        "pack_count": len(packs),
        "manifest_count": sum(1 for p in packs if (_SCENARIOS / p / authoring.MANIFEST_FILENAME).is_file()),
        "errors": all_errors,
        "warnings": all_warnings,
        "per_pack": per_pack,
        "governance_banner": "INTEGRITY REPORT — explanatory audit; CLI remains authoritative",
    }


def lineage_continuity_report(pack_id: str) -> dict[str, Any]:
    pack_dir = _SCENARIOS / pack_id
    manifest = authoring.load_authoring_manifest(pack_dir)
    meta = _read_json(pack_dir / "metadata.json") if (pack_dir / "metadata.json").is_file() else {}

    chain: list[dict[str, str]] = []
    current = pack_id
    seen: set[str] = set()
    while current and current not in seen:
        seen.add(current)
        m = authoring.load_authoring_manifest(_SCENARIOS / current)
        parent = None
        if m:
            parent = m.get("parent_pack_id")
        if not parent and current == pack_id:
            parent = (meta.get("provenance") or {}).get("baseline_pack_id")
        chain.append(
            {
                "pack_id": current,
                "parent_pack_id": str(parent) if parent else "",
                "promotion_status": str((m or {}).get("promotion_status") or "none"),
            }
        )
        if not parent or parent == current:
            break
        current = str(parent)

    baseline = (meta.get("provenance") or {}).get("baseline_pack_id")
    return {
        "pack_id": pack_id,
        "ancestor_chain": chain,
        "depth": len(chain),
        "topology_baseline_pack_id": baseline or "",
        "promotion_status": str((manifest or {}).get("promotion_status") or "none"),
    }
