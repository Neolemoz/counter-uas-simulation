#!/usr/bin/env python3
"""Batch governance lint for committed SA-R0 platform fixtures (PLAT-SA-STAB)."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

_REPO = Path(__file__).resolve().parents[2]
FIXTURES_SA = _REPO / "fixtures/sa_r0"

FORBIDDEN_MD_RE = re.compile(
    r"(?<!not )deployment\s+readiness|(?<!not )validated\s+effectiveness|(?<!not )validated\s+probability|"
    r"P\s*\(\s*kill\s*\)|tactical\s+superiority|recommend\s+intercept|should\s+deploy|tactical\s+doctrine",
    re.I,
)

FORBIDDEN_SUBSTRINGS = (
    "deployment readiness",
    "validated effectiveness",
    "tactical superiority",
    "P(kill)",
    "recommend intercept",
    "should deploy",
    "tactical doctrine",
)

FORBIDDEN_VIEWER_UI = (
    "battle management",
    "operator console",
    "live mission",
    "tactical dashboard",
    "command and control",
)

SA_ARTIFACT_TYPES = {
    "replay_linkage_index_v1",
    "cross_sweep_synthesis_v1",
    "replay_storyboard_v1",
    "replay_storyboard_index_v1",
    "replay_research_bundle_v1",
    "replay_mc_sweep_v1",
    "replay_compare_report_v1",
    "replay_review_report_v1",
    "replay_publication_report_v1",
    "replay_presentation_report_v1",
}


def lint_markdown_text(text: str, *, context: str = "") -> list[str]:
    issues: list[str] = []
    if FORBIDDEN_MD_RE.search(text):
        issues.append(f"{context}: forbidden operational phrasing in markdown")
    lower = text.lower()
    for phrase in FORBIDDEN_SUBSTRINGS:
        if phrase.lower() in lower and f"not {phrase}" not in lower:
            issues.append(f"{context}: forbidden phrase '{phrase}'")
    return issues


def lint_sa_json_artifact(payload: dict[str, Any], *, path: str = "") -> list[str]:
    issues: list[str] = []
    ctx = path or str(payload.get("artifact_type") or "artifact")
    artifact_type = str(payload.get("artifact_type") or "")
    if artifact_type and artifact_type not in SA_ARTIFACT_TYPES:
        if artifact_type.endswith("_v1") and "replay" in artifact_type:
            pass
        elif artifact_type in ("scenario_topology_catalog",):
            return issues

    governance = payload.get("governance")
    if not isinstance(governance, dict):
        if artifact_type in SA_ARTIFACT_TYPES or "sweep_id" in payload:
            issues.append(f"{ctx}: missing governance block")
        return issues

    notice = str(governance.get("notice") or "")
    if artifact_type in SA_ARTIFACT_TYPES and not notice:
        issues.append(f"{ctx}: empty governance notice")

    text = json.dumps(payload, sort_keys=True, default=str)
    if "operational readiness" in text.lower():
        anti = " ".join(str(a) for a in governance.get("anti_claims") or [])
        if "not operational readiness" not in anti.lower() and "not operational" not in notice.lower():
            issues.append(f"{ctx}: operational-readiness language without explicit anti-claim")

    for field in ("summary", "copy", "observation", "caveat"):
        val = payload.get(field)
        if isinstance(val, str):
            issues.extend(lint_markdown_text(val, context=f"{ctx}.{field}"))

    for bullet in payload.get("bullets") or []:
        if isinstance(bullet, dict):
            obs = str(bullet.get("observation") or "")
            cav = str(bullet.get("caveat") or "")
            issues.extend(lint_markdown_text(f"{obs} {cav}", context=f"{ctx}.bullet"))

    for scene in payload.get("scenes") or []:
        if isinstance(scene, dict):
            copy = str(scene.get("copy") or "")
            issues.extend(lint_markdown_text(copy, context=f"{ctx}.scene"))

    return issues


def lint_sa_markdown_file(path: Path) -> list[str]:
    if not path.is_file():
        return []
    return lint_markdown_text(path.read_text(encoding="utf-8"), context=str(path.relative_to(_REPO)))


def lint_viewer_ui_copy() -> list[str]:
    """Scan sa-r0-viewer source for operational UI phrasing (PLAT-SA-H5)."""
    issues: list[str] = []
    viewer_src = _REPO / "platform/sa-r0-viewer/src"
    if not viewer_src.is_dir():
        return issues
    for path in sorted(viewer_src.rglob("*.tsx")):
        text = path.read_text(encoding="utf-8").lower()
        for phrase in FORBIDDEN_VIEWER_UI:
            if phrase in text:
                issues.append(
                    f"{path.relative_to(_REPO)}: forbidden viewer UI phrase '{phrase}'"
                )
    return issues


def batch_lint_sa_fixtures() -> list[str]:
    """Lint committed SA fixture JSON/MD; returns human-readable issue strings."""
    issues: list[str] = []

    json_paths: list[Path] = []
    if (FIXTURES_SA / "synthesis").is_dir():
        json_paths.extend((FIXTURES_SA / "synthesis").glob("*.json"))
    if (FIXTURES_SA / "presentations").is_dir():
        json_paths.extend((FIXTURES_SA / "presentations").glob("*.json"))
    for sweep_json in (FIXTURES_SA / "sweeps").glob("*/sweep.json"):
        json_paths.append(sweep_json)
    for sweep_json in (FIXTURES_SA / "sweeps").glob("*/reports/replay_*_v1.json"):
        json_paths.append(sweep_json)
    bundle_manifest = FIXTURES_SA / "research_bundles/sa_r0_corpus_r1/manifest.json"
    if bundle_manifest.is_file():
        json_paths.append(bundle_manifest)

    seen: set[Path] = set()
    for path in sorted(json_paths):
        if path in seen:
            continue
        seen.add(path)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            issues.append(f"{path.relative_to(_REPO)}: invalid JSON ({exc})")
            continue
        if isinstance(payload, dict):
            issues.extend(
                lint_sa_json_artifact(payload, path=str(path.relative_to(_REPO)))
            )

    md_roots = [
        FIXTURES_SA / "synthesis",
        FIXTURES_SA / "sweeps",
    ]
    for root in md_roots:
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*.md")):
            if "reports" in path.parts or root.name == "synthesis":
                issues.extend(lint_sa_markdown_file(path))

    return issues


def main() -> None:
    issues = batch_lint_sa_fixtures() + lint_viewer_ui_copy()
    if issues:
        for item in issues:
            print(f"governance_lint_sa: {item}", file=sys.stderr)
        raise SystemExit(1)
    print("governance_lint_sa OK")


if __name__ == "__main__":
    main()
