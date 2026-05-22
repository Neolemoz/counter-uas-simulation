#!/usr/bin/env python3
"""Scenario authoring manifest and promotion helpers (PLAT-SA-A1)."""

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

import replay_sa_scenario as scenario_mod  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_SCENARIOS = _REPO / "fixtures" / "scenarios"
_ORCH_MIRRORS = _REPO / "fixtures" / "orchestration" / "validation_mirrors"

AUTHORING_ARTIFACT_TYPE = "scenario_authoring_manifest_v1"
AUTHORING_SCHEMA_VERSION = "1"
MANIFEST_FILENAME = "authoring_manifest.json"

PROMOTION_STATUSES = (
    "draft",
    "linted",
    "validated",
    "promoted",
    "orchestration_ready",
    "deprecated",
    "archived",
)
PROMOTION_RANK = {s: i for i, s in enumerate(PROMOTION_STATUSES)}
TERMINAL_STATUSES = frozenset({"archived"})
RETIRED_STATUSES = frozenset({"deprecated", "archived"})
PROMOTION_SUMMARY_FILENAME = "authoring_promotion_summary_v1.json"

TOPOLOGY_VARIANT_CLASSES = frozenset(
    {
        "baseline",
        "sensor_layout_experiment",
        "topology_layout_experiment",
        "ingress_timing_experiment",
        "synthetic_derivation",
        "maintainer_fork",
    }
)

CANONICAL_PACK_FILES = scenario_mod.REQUIRED_PACK_FILES + ("terrain.json",)

GOVERNANCE_DEFAULT: dict[str, Any] = {
    "notice": "Authoring manifest for fixture topology workflow only.",
    "anti_claims": [
        "not operational deployment state",
        "not simulation launch authority",
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


def manifest_path(pack_dir: Path) -> Path:
    return pack_dir.resolve() / MANIFEST_FILENAME


def validation_mirror_rel(pack_id: str) -> str:
    return f"fixtures/orchestration/validation_mirrors/{pack_id}_validation_mirror.json"


def validation_mirror_path(pack_id: str) -> Path:
    return _REPO / validation_mirror_rel(pack_id)


def compute_pack_fingerprint(pack_dir: Path) -> str:
    """SHA-256 over canonical pack JSON files (sorted paths)."""
    pack_dir = pack_dir.resolve()
    h = hashlib.sha256()
    for name in sorted(CANONICAL_PACK_FILES):
        path = pack_dir / name
        if path.is_file():
            h.update(name.encode("utf-8"))
            h.update(path.read_bytes())
    return f"sha256:{h.hexdigest()}"


def infer_topology_variant_class(metadata: dict[str, Any]) -> str:
    prov = metadata.get("provenance") or {}
    baseline = prov.get("baseline_pack_id")
    tags = set(metadata.get("topology_tags") or [])
    if not baseline:
        if "topology_experiment" not in tags and "sensor_experiment" not in tags:
            return "baseline"
    if "topology_experiment" in tags or "sensor_experiment" in tags:
        if "sensor_experiment" in tags or "sensor_placement" in (metadata.get("narrative_focus") or []):
            return "sensor_layout_experiment"
        return "topology_layout_experiment"
    if metadata.get("ingress_archetype") == "valley" and "delayed_track_confirmation" in tags:
        return "ingress_timing_experiment"
    prov = metadata.get("provenance") or {}
    if prov.get("baseline_pack_id") and prov.get("baseline_pack_id") != Path("").name:
        return "synthetic_derivation"
    return "baseline"


def build_manifest_draft(pack_dir: Path, *, promotion_status: str = "draft") -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    metadata = _read_json(pack_dir / "metadata.json")
    prov = metadata.get("provenance") or {}
    parent = prov.get("baseline_pack_id")
    if parent == pack_id:
        parent = None
    return {
        "artifact_type": AUTHORING_ARTIFACT_TYPE,
        "schema_version": AUTHORING_SCHEMA_VERSION,
        "pack_id": pack_id,
        "parent_pack_id": parent,
        "topology_variant_class": infer_topology_variant_class(metadata),
        "promotion_status": promotion_status,
        "promotion_lineage": [],
        "governance": dict(GOVERNANCE_DEFAULT),
    }


def load_authoring_manifest(pack_dir: Path) -> dict[str, Any] | None:
    path = manifest_path(pack_dir)
    if not path.is_file():
        return None
    return _read_json(path)


def _collect_parent_chain(pack_id: str, *, seen: set[str] | None = None) -> list[str]:
    seen = seen or set()
    if pack_id in seen:
        return list(seen)
    seen.add(pack_id)
    pack_dir = _SCENARIOS / pack_id
    manifest = load_authoring_manifest(pack_dir)
    if not manifest:
        meta_path = pack_dir / "metadata.json"
        if meta_path.is_file():
            meta = _read_json(meta_path)
            parent = (meta.get("provenance") or {}).get("baseline_pack_id")
            if parent and parent != pack_id:
                return _collect_parent_chain(str(parent), seen=seen)
        return list(seen)
    parent = manifest.get("parent_pack_id")
    if parent and parent != pack_id:
        return _collect_parent_chain(str(parent), seen=seen)
    return list(seen)


def lint_authoring_manifest(
    pack_dir: Path,
    *,
    strict: bool = False,
) -> dict[str, Any]:
    issues: list[str] = []
    warnings: list[str] = []

    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    path = manifest_path(pack_dir)
    if not path.is_file():
        return {"ok": True, "issues": [], "warnings": ["no authoring_manifest.json (optional)"]}

    try:
        manifest = _read_json(path)
    except (json.JSONDecodeError, OSError) as exc:
        return {"ok": False, "issues": [str(exc)], "warnings": []}

    if manifest.get("artifact_type") != AUTHORING_ARTIFACT_TYPE:
        issues.append(f"artifact_type must be {AUTHORING_ARTIFACT_TYPE!r}")
    if str(manifest.get("schema_version")) != AUTHORING_SCHEMA_VERSION:
        issues.append(f"schema_version must be {AUTHORING_SCHEMA_VERSION!r}")
    if manifest.get("pack_id") != pack_id:
        issues.append(f"pack_id {manifest.get('pack_id')!r} must match directory {pack_id!r}")

    status = manifest.get("promotion_status")
    if status not in PROMOTION_STATUSES:
        issues.append(f"invalid promotion_status: {status!r}")
    elif status in PROMOTION_STATUSES:
        pass

    variant = manifest.get("topology_variant_class")
    if variant is not None and variant not in TOPOLOGY_VARIANT_CLASSES:
        (issues if strict else warnings).append(f"unknown topology_variant_class: {variant!r}")

    parent = manifest.get("parent_pack_id")
    if parent == pack_id:
        issues.append("parent_pack_id must not equal pack_id")
    if parent:
        parent_dir = _SCENARIOS / str(parent)
        if not parent_dir.is_dir():
            issues.append(f"parent_pack_id directory missing: {parent}")
        chain = _collect_parent_chain(pack_id)
        if len(chain) != len(set(chain)):
            issues.append("parent_pack_id chain contains a cycle")

    meta_path = pack_dir / "metadata.json"
    if meta_path.is_file() and parent:
        meta = _read_json(meta_path)
        baseline = (meta.get("provenance") or {}).get("baseline_pack_id")
        if variant in ("sensor_layout_experiment", "topology_layout_experiment") and not baseline:
            warnings.append("topology experiment pack missing metadata.provenance.baseline_pack_id")

    ref = manifest.get("validation_snapshot_ref")
    if ref:
        ref_path = _REPO / ref if not Path(str(ref)).is_absolute() else Path(ref)
        if not ref_path.is_file():
            warnings.append(f"validation_snapshot_ref not found: {ref}")

    fp = manifest.get("validation_pack_fingerprint")
    if fp:
        current = compute_pack_fingerprint(pack_dir)
        if fp != current:
            warnings.append("validation_pack_fingerprint stale — re-validate before promote")

    for key in ("command", "engage", "readiness_score", "live_mode"):
        if key in manifest:
            issues.append(f"prohibited manifest field: {key!r}")

    lineage = manifest.get("promotion_lineage")
    if lineage is not None and not isinstance(lineage, list):
        issues.append("promotion_lineage must be an array")

    ok = len(issues) == 0
    return {"ok": ok, "issues": issues, "warnings": warnings}


def is_validation_stale(manifest: dict[str, Any], pack_dir: Path) -> bool:
    fp = manifest.get("validation_pack_fingerprint")
    if not fp:
        return True
    return fp != compute_pack_fingerprint(pack_dir)


def write_validation_mirror(pack_dir: Path, *, manifest_id: str = "authoring", job_id: str = "validate") -> Path:
    """Write experiment_validation_mirror_v1 and return path."""
    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    result = scenario_mod.lint_scenario_pack(pack_dir)
    out_path = validation_mirror_path(pack_id)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    mirror = {
        "artifact_type": "experiment_validation_mirror_v1",
        "schema_version": "experiment_validation_mirror_v1",
        "manifest_id": manifest_id,
        "job_id": job_id,
        "scenario_pack_id": pack_id,
        "scenario_pack_ref": f"fixtures/scenarios/{pack_id}",
        "checked_at": _iso_now(),
        "ok": bool(result.get("ok")),
        "issues": result.get("issues") or [],
        "warnings": result.get("warnings") or [],
        "governance_banner": "VALIDATION MIRROR — not certification or deployment authority",
    }
    _write_json(out_path, mirror)
    return out_path


def record_validation(
    pack_dir: Path,
    *,
    dry_run: bool = False,
) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    lint = scenario_mod.lint_scenario_pack(pack_dir)
    if not lint.get("ok"):
        return {"ok": False, "error": "lint failed", "lint": lint}

    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "pack_id": pack_id,
            "would_status": "validated",
        }

    mirror_path = write_validation_mirror(pack_dir)
    manifest = load_authoring_manifest(pack_dir) or build_manifest_draft(pack_dir)
    manifest["validation_snapshot_ref"] = str(mirror_path.relative_to(_REPO))
    manifest["validation_pack_fingerprint"] = compute_pack_fingerprint(pack_dir)
    manifest["promotion_status"] = "validated"
    manifest["updated_at"] = _iso_now()
    _write_json(manifest_path(pack_dir), manifest)
    return {
        "ok": True,
        "pack_id": pack_id,
        "promotion_status": "validated",
        "validation_snapshot_ref": manifest["validation_snapshot_ref"],
        "mirror": str(mirror_path),
    }


def _append_lineage(
    manifest: dict[str, Any],
    from_status: str,
    to_status: str,
    *,
    notes: str = "",
    pack_dir: Path | None = None,
) -> str:
    events = list(manifest.get("promotion_lineage") or [])
    recorded_at = _iso_now()
    event_id = f"promote-{recorded_at}"
    event: dict[str, Any] = {
        "event_id": event_id,
        "from_status": from_status,
        "to_status": to_status,
        "actor": "cli:promote_scenario_pack",
        "notes": notes,
        "recorded_at": recorded_at,
    }
    if pack_dir is not None:
        event["pack_fingerprint"] = compute_pack_fingerprint(pack_dir)
        ref = manifest.get("validation_snapshot_ref")
        if ref:
            ref_path = _REPO / ref
            if ref_path.is_file():
                try:
                    mirror = _read_json(ref_path)
                    event["validation_mirror_ok"] = bool(mirror.get("ok"))
                except (json.JSONDecodeError, OSError):
                    event["validation_mirror_ok"] = None
    events.append(event)
    manifest["promotion_lineage"] = events
    return event_id


def diff_manifests(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    keys = sorted(set(before.keys()) | set(after.keys()))
    changed: dict[str, dict[str, Any]] = {}
    for key in keys:
        if before.get(key) != after.get(key):
            changed[key] = {"before": before.get(key), "after": after.get(key)}
    return {"changed_fields": changed, "field_count": len(changed)}


def build_promotion_summary(
    pack_dir: Path,
    *,
    from_status: str,
    to_status: str,
    event_id: str,
) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    manifest = load_authoring_manifest(pack_dir) or {}
    return {
        "artifact_type": "authoring_promotion_summary_v1",
        "schema_version": "1",
        "pack_id": pack_dir.name,
        "from_status": from_status,
        "to_status": to_status,
        "event_id": event_id,
        "recorded_at": _iso_now(),
        "validation_snapshot_ref": manifest.get("validation_snapshot_ref"),
        "validation_pack_fingerprint": manifest.get("validation_pack_fingerprint"),
        "orchestration_handoff_refs": manifest.get("orchestration_handoff_refs") or [],
        "governance_banner": "PROMOTION SUMMARY — CLI audit trail; not operational authority",
    }


def write_promotion_summary(pack_dir: Path, summary: dict[str, Any]) -> Path:
    path = pack_dir.resolve() / PROMOTION_SUMMARY_FILENAME
    _write_json(path, summary)
    return path


def repro_check(pack_dir: Path) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    lint = scenario_mod.lint_scenario_pack(pack_dir)
    manifest = load_authoring_manifest(pack_dir)
    issues: list[str] = []
    if not lint.get("ok"):
        issues.append("topology lint failed")
    if not manifest:
        issues.append("no authoring_manifest.json")
    else:
        if is_validation_stale(manifest, pack_dir):
            issues.append("validation fingerprint stale")
        ref = manifest.get("validation_snapshot_ref")
        if ref:
            ref_path = _REPO / ref
            if not ref_path.is_file():
                issues.append(f"validation mirror missing: {ref}")
            else:
                mirror = _read_json(ref_path)
                if not mirror.get("ok"):
                    issues.append("validation mirror ok=false")
        else:
            issues.append("no validation_snapshot_ref")
    return {
        "ok": len(issues) == 0,
        "pack_id": pack_id,
        "issues": issues,
        "lint_ok": bool(lint.get("ok")),
    }


def diff_since_last_promote(pack_dir: Path) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    manifest = load_authoring_manifest(pack_dir)
    if not manifest:
        return {"ok": False, "error": "no manifest"}
    events = manifest.get("promotion_lineage") or []
    if not events:
        return {"ok": True, "pack_id": pack_dir.name, "diff": {}, "note": "no promotion events"}
    last = events[-1]
    before_status = last.get("from_status")
    current_status = manifest.get("promotion_status")
    return {
        "ok": True,
        "pack_id": pack_dir.name,
        "last_event": last,
        "status_transition_since_event": before_status != current_status,
        "current_fingerprint": compute_pack_fingerprint(pack_dir),
        "stored_fingerprint": manifest.get("validation_pack_fingerprint"),
        "fingerprint_changed": manifest.get("validation_pack_fingerprint")
        != compute_pack_fingerprint(pack_dir),
    }


def promote_scenario_pack(
    pack_dir: Path,
    *,
    target_status: str,
    notes: str = "",
    dry_run: bool = False,
    allow_stale: bool = False,
    record_validation_first: bool = False,
) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    pack_id = pack_dir.name
    if target_status not in PROMOTION_STATUSES:
        return {"ok": False, "error": f"invalid target_status: {target_status}"}

    lint = scenario_mod.lint_scenario_pack(pack_dir)
    if not lint.get("ok"):
        return {"ok": False, "error": "pack lint failed", "lint": lint}

    manifest = load_authoring_manifest(pack_dir) or build_manifest_draft(pack_dir)
    from_status = str(manifest.get("promotion_status") or "draft")

    if from_status in TERMINAL_STATUSES and target_status != from_status:
        return {
            "ok": False,
            "error": f"cannot transition from terminal status {from_status!r}",
        }

    if target_status not in RETIRED_STATUSES and from_status in RETIRED_STATUSES:
        return {
            "ok": False,
            "error": f"cannot promote from retired status {from_status!r} without explicit downgrade policy",
        }

    if PROMOTION_RANK[target_status] >= PROMOTION_RANK["validated"] and from_status != target_status:
        if from_status not in RETIRED_STATUSES:
            if record_validation_first or from_status in ("draft", "linted") or is_validation_stale(
                manifest, pack_dir
            ):
                if not dry_run and PROMOTION_RANK[target_status] >= PROMOTION_RANK["validated"]:
                    rec = record_validation(pack_dir, dry_run=False)
                    if not rec.get("ok"):
                        return rec
                    manifest = load_authoring_manifest(pack_dir) or manifest
                    from_status = str(manifest.get("promotion_status") or "validated")

    if (
        is_validation_stale(manifest, pack_dir)
        and PROMOTION_RANK[target_status] >= PROMOTION_RANK["promoted"]
        and target_status not in RETIRED_STATUSES
    ):
        if not allow_stale:
            return {
                "ok": False,
                "error": "validation stale — re-run record_validation or use --allow-stale",
                "fingerprint": manifest.get("validation_pack_fingerprint"),
                "current": compute_pack_fingerprint(pack_dir),
            }

    if (
        PROMOTION_RANK[target_status] < PROMOTION_RANK[from_status]
        and target_status not in RETIRED_STATUSES
        and from_status not in RETIRED_STATUSES
    ):
        return {
            "ok": False,
            "error": f"cannot downgrade {from_status!r} -> {target_status!r} without --force-downgrade",
        }

    if from_status == target_status:
        return {"ok": True, "pack_id": pack_id, "unchanged": True, "promotion_status": target_status}

    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "pack_id": pack_id,
            "from_status": from_status,
            "to_status": target_status,
        }

    before_manifest = dict(manifest)
    manifest["promotion_status"] = target_status
    manifest["updated_at"] = _iso_now()
    if notes:
        manifest["authoring_notes"] = notes
    event_id = _append_lineage(manifest, from_status, target_status, notes=notes, pack_dir=pack_dir)
    summary = build_promotion_summary(
        pack_dir,
        from_status=from_status,
        to_status=target_status,
        event_id=event_id,
    )
    summary_path = write_promotion_summary(pack_dir, summary)
    manifest["promotion_summary_ref"] = str(summary_path.relative_to(_REPO))
    _write_json(manifest_path(pack_dir), manifest)
    return {
        "ok": True,
        "pack_id": pack_id,
        "from_status": from_status,
        "promotion_status": target_status,
        "manifest": str(manifest_path(pack_dir)),
        "event_id": event_id,
        "promotion_summary_ref": manifest["promotion_summary_ref"],
        "manifest_diff": diff_manifests(before_manifest, manifest),
    }


def check_stale(pack_dir: Path) -> dict[str, Any]:
    pack_dir = pack_dir.resolve()
    manifest = load_authoring_manifest(pack_dir)
    if not manifest:
        return {"ok": True, "pack_id": pack_dir.name, "has_manifest": False, "stale": None}
    stale = is_validation_stale(manifest, pack_dir)
    return {
        "ok": True,
        "pack_id": pack_dir.name,
        "has_manifest": True,
        "stale": stale,
        "promotion_status": manifest.get("promotion_status"),
        "stored_fingerprint": manifest.get("validation_pack_fingerprint"),
        "current_fingerprint": compute_pack_fingerprint(pack_dir),
    }


def catalog_promotion_ready(pack_id: str) -> bool:
    manifest = load_authoring_manifest(_SCENARIOS / pack_id)
    if not manifest:
        return True
    status = manifest.get("promotion_status") or "draft"
    return PROMOTION_RANK.get(str(status), 0) >= PROMOTION_RANK["promoted"]
