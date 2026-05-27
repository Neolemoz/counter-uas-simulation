"""Embed RT tactical capture annex into replay_sa_bundle (PLAT-RT-SA3).

Evaluation-side only — not a parser contract.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

RT_TACTICAL_CAPTURE_ANNEX_SCHEMA = "rt_tactical_capture_annex_v1"
RT_TACTICAL_REPLAY_CONTINUITY_SCHEMA = "rt_tactical_replay_continuity_v1"
RT_SANDBOX_CAPTURE_ORIGIN = "rt_sandbox_capture_v1"
REPLAY_CONTINUITY_GOVERNANCE_BANNER = (
    "RT tactical continuity — explanatory replay only; not operational authority"
)
AUTHORITY_STOPPED_AT = "replay_sa_bundle_pack"
REQUIRED_ANNEX_AUTHORITY = "replay_boundary_scoped"


def _read_json(path: Path) -> dict[str, Any]:
    import json

    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def validate_rt_tactical_annex(annex: dict[str, Any]) -> str | None:
    if annex.get("schema") != RT_TACTICAL_CAPTURE_ANNEX_SCHEMA:
        return "invalid tactical annex schema"
    if annex.get("authority_label") != REQUIRED_ANNEX_AUTHORITY:
        return "tactical annex authority_label must be replay_boundary_scoped"
    if not annex.get("governance_banner"):
        return "tactical annex governance_banner required"
    if annex.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on tactical annex"
    return None


def load_rt_tactical_annex_from_capture_staging(staging_dir: Path) -> dict[str, Any] | None:
    """Load annex from sidecar or normalized manifest embed."""
    staging_dir = staging_dir.resolve()
    annex_path = staging_dir / "tactical_annex.json"
    if annex_path.is_file():
        annex = _read_json(annex_path)
    else:
        norm_path = staging_dir / "normalized_manifest.json"
        if not norm_path.is_file():
            return None
        norm = _read_json(norm_path)
        annex = norm.get("tactical_annex")
        if not isinstance(annex, dict):
            return None

    err = validate_rt_tactical_annex(annex)
    if err:
        raise ValueError(f"{staging_dir}: {err}")
    return annex


def _staging_rel_ref(staging_dir: Path, repo_root: Path | None) -> str:
    if repo_root is not None:
        try:
            return staging_dir.relative_to(repo_root.resolve()).as_posix()
        except ValueError:
            pass
    return staging_dir.as_posix()


def build_rt_tactical_replay_continuity_block(
    annex: dict[str, Any],
    *,
    capture_staging_dir: Path,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    annex_copy = copy.deepcopy(annex)
    annex_copy.pop("authoritative_parent_ref", None)

    provenance_json: dict[str, Any] | None = None
    prov_path = capture_staging_dir / "provenance.json"
    if prov_path.is_file():
        provenance_json = _read_json(prov_path)

    norm_embedded = (capture_staging_dir / "normalized_manifest.json").is_file()
    if norm_embedded:
        norm = _read_json(capture_staging_dir / "normalized_manifest.json")
        norm_embedded = isinstance(norm.get("tactical_annex"), dict)

    annex_ref = "tactical_annex.json"
    if provenance_json:
        refs = provenance_json.get("source_artifact_refs") or {}
        annex_ref = str(refs.get("tactical_annex_ref") or annex_ref)

    capture_id = str(
        annex_copy.get("capture_candidate_id")
        or (capture_staging_dir / "candidate.json").exists()
        and _read_json(capture_staging_dir / "candidate.json").get("capture_candidate_id")
        or ""
    )

    return {
        "schema": RT_TACTICAL_REPLAY_CONTINUITY_SCHEMA,
        "source": RT_SANDBOX_CAPTURE_ORIGIN,
        "continuity_available": True,
        "capture_candidate_id": capture_id,
        "governance_banner": REPLAY_CONTINUITY_GOVERNANCE_BANNER,
        "provenance": {
            "imported_from_rt_capture": True,
            "rt_capture_ref": _staging_rel_ref(capture_staging_dir, repo_root),
            "tactical_annex_ref": annex_ref,
            "normalized_manifest_embedded": norm_embedded,
            "authority_stopped_at": AUTHORITY_STOPPED_AT,
        },
        "tactical_annex": annex_copy,
    }


def attach_rt_tactical_replay_continuity(
    bundle: dict[str, Any],
    *,
    capture_staging_dir: Path | None,
    repo_root: Path | None = None,
    out_dir: Path | None = None,
) -> dict[str, Any]:
    if capture_staging_dir is None:
        return bundle
    annex = load_rt_tactical_annex_from_capture_staging(capture_staging_dir)
    if annex is None:
        return bundle

    block = build_rt_tactical_replay_continuity_block(
        annex,
        capture_staging_dir=capture_staging_dir,
        repo_root=repo_root,
    )
    bundle["rt_tactical_replay_continuity"] = block

    caveats = list(bundle.get("interpretation_caveats") or [])
    utc_note = (
        "RT tactical timelines are UTC-anchored sandbox records — not synchronized "
        "to log-line clock t unless separately correlated."
    )
    if utc_note not in caveats:
        caveats.append(utc_note)
    bundle["interpretation_caveats"] = caveats

    if out_dir is not None:
        import json

        sidecar = out_dir / "tactical_annex.json"
        sidecar.write_text(
            json.dumps(block["tactical_annex"], indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return bundle


def lint_rt_tactical_replay_continuity(payload: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    block = payload.get("rt_tactical_replay_continuity")
    if block is None:
        return issues
    if not isinstance(block, dict):
        issues.append("rt_tactical_replay_continuity must be an object")
        return issues
    if block.get("schema") != RT_TACTICAL_REPLAY_CONTINUITY_SCHEMA:
        issues.append("rt_tactical_replay_continuity schema invalid")
    if not block.get("governance_banner"):
        issues.append("rt_tactical_replay_continuity governance_banner required")
    annex = block.get("tactical_annex")
    if not isinstance(annex, dict):
        issues.append("rt_tactical_replay_continuity.tactical_annex required")
    else:
        err = validate_rt_tactical_annex(annex)
        if err:
            issues.append(f"rt_tactical_replay_continuity annex: {err}")
    for bad in ("authoritative_parent_ref", "authority_state", "command"):
        if bad in block:
            issues.append(f"rt_tactical_replay_continuity forbids {bad}")
    return issues
