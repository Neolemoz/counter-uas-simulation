#!/usr/bin/env python3
"""Static replay visualization for reviewer comprehension workflows.

All outputs are derived evaluation artifacts. They consume frozen replay narrative
and observability JSON without modifying runtime behavior, parser contracts, or
tactical authority surfaces.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL_DIR = Path(__file__).resolve().parent
for p in (_EVAL_DIR,):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from replay_viz_comprehension import build_comprehension_digest  # noqa: E402
from replay_viz_figures import RENDER_PROFILE  # noqa: E402
from replay_viz_figures import render_comprehension_panel  # noqa: E402
from replay_viz_figures import render_engagement_series  # noqa: E402
from replay_viz_figures import render_sparse_topdown  # noqa: E402
from replay_viz_html import render_composite_html  # noqa: E402

VISUALIZATION_SCHEMA_VERSION = "replay_static_visualization_v1"
NON_AUTHORITATIVE_NOTICE = (
    "Derived evaluation artifact only. Does not replace parser-visible summaries, "
    "runtime topics, tactical authority, lifecycle semantics, or replay contracts."
)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable JSON: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _governance_block() -> dict[str, Any]:
    return {
        "schema_version": VISUALIZATION_SCHEMA_VERSION,
        "notice": NON_AUTHORITATIVE_NOTICE,
        "constraints": [
            "evaluation-side only",
            "additive-only",
            "parser-safe",
            "replay-safe",
            "non-authoritative",
            "static-rendered",
            "preserve original replay lineage",
        ],
        "anti_claims": [
            "not operational readiness evidence",
            "not hardware readiness evidence",
            "not a tactical authority surface",
            "not a lifecycle semantic replacement",
            "not a parser contract",
            "not governance approval",
            "not a live dashboard",
        ],
    }


def _copy_lineage(narrative: dict[str, Any], observability: dict[str, Any] | None) -> dict[str, Any]:
    lineage = dict(narrative.get("lineage") or {})
    if observability and isinstance(observability.get("bundle"), dict):
        bundle_lineage = observability["bundle"].get("lineage") or {}
        if isinstance(bundle_lineage, dict):
            for key in ("log_path", "meta_path", "run_id", "seed", "seed_source", "cohort", "git_commit", "git_dirty"):
                if key not in lineage and key in bundle_lineage:
                    lineage[key] = bundle_lineage[key]
    return lineage


def build_visualization_manifest(
    narrative: dict[str, Any],
    *,
    observability: dict[str, Any] | None = None,
    input_paths: dict[str, str] | None = None,
    include_engagement_series: bool = False,
    include_sparse_topdown: bool = False,
) -> dict[str, Any]:
    """Build a deterministic visualization manifest from frozen replay artifacts."""

    if narrative.get("artifact_type") != "replay_narrative_report":
        raise ValueError("expected artifact_type replay_narrative_report")
    if narrative.get("narrative_schema_version") != "replay_narrative_v1":
        raise ValueError("expected narrative_schema_version replay_narrative_v1")

    lineage = _copy_lineage(narrative, observability)
    summary = narrative.get("summary") if isinstance(narrative.get("summary"), dict) else {}
    warnings = list(narrative.get("warnings") or [])
    if observability and isinstance(observability.get("bundle"), dict):
        warnings.extend(observability["bundle"].get("warnings") or [])
    warnings = sorted(dict.fromkeys(str(w) for w in warnings))

    skipped_figures: list[dict[str, str]] = []
    if include_engagement_series:
        log_path = lineage.get("log_path")
        if not log_path or not Path(str(log_path)).is_file():
            skipped_figures.append(
                {
                    "figure_category": "engagement_series",
                    "reason": "no_parseable_metrics_evidence (missing or unreadable log_path)",
                }
            )
    if include_sparse_topdown:
        log_path = lineage.get("log_path")
        if not log_path or not Path(str(log_path)).is_file():
            skipped_figures.append(
                {
                    "figure_category": "sparse_topdown",
                    "reason": "no_parseable_position_evidence (missing or unreadable log_path)",
                }
            )

    core_figure_count = 4  # timeline, divergence, lifecycle, comprehension_panel
    manifest_summary = {
        "run_id": summary.get("run_id") or lineage.get("run_id"),
        "event_count": summary.get("event_count"),
        "window_count": summary.get("window_count"),
        "selection_oracle_divergence_class": summary.get("selection_oracle_divergence_class"),
        "figure_count_planned": core_figure_count + int(include_engagement_series) + int(include_sparse_topdown),
    }

    return {
        "artifact_type": "replay_static_visualization_manifest",
        "visualization_schema_version": VISUALIZATION_SCHEMA_VERSION,
        "comprehension": build_comprehension_digest(narrative),
        "governance": _governance_block(),
        "render_profile": RENDER_PROFILE,
        "lineage": lineage,
        "source_artifacts": {
            "narrative_schema_version": narrative.get("narrative_schema_version"),
            "observability_artifact_type": (
                observability.get("artifact_type") if isinstance(observability, dict) else None
            ),
            "input_paths": input_paths or {},
        },
        "summary": manifest_summary,
        "figures": [],
        "skipped_figures": skipped_figures,
        "warnings": warnings,
        "interpretation_caveats": [
            "Static replay visualizations are explanatory visualization layers, not a unified authoritative replay state.",
            "Figure overlays localize replay evidence in time; they are not causal proof.",
            "Sparse trajectory or engagement views show log-evidenced samples only when present.",
            "Warnings are interpretation prompts, not approval, severity, readiness, or governance status.",
        ],
    }


def lint_governance_artifact(payload: dict[str, Any]) -> dict[str, Any]:
    """Check a visualization manifest for minimum governance labels."""

    issues: list[str] = []
    governance = payload.get("governance")
    if not isinstance(governance, dict):
        issues.append("missing governance block")
    else:
        if governance.get("schema_version") != VISUALIZATION_SCHEMA_VERSION:
            issues.append("missing or unexpected governance schema_version")
        notice = str(governance.get("notice") or "")
        if "Derived evaluation artifact" not in notice:
            issues.append("governance notice must identify derived evaluation artifact status")
        constraints = set(governance.get("constraints") or [])
        for required in ("evaluation-side only", "additive-only", "parser-safe", "replay-safe", "non-authoritative"):
            if required not in constraints:
                issues.append(f"missing governance constraint: {required}")
        anti_claims = set(governance.get("anti_claims") or [])
        for required in (
            "not operational readiness evidence",
            "not a tactical authority surface",
            "not a lifecycle semantic replacement",
            "not a parser contract",
            "not a live dashboard",
        ):
            if required not in anti_claims:
                issues.append(f"missing governance anti-claim: {required}")

    if payload.get("artifact_type") != "replay_static_visualization_manifest":
        issues.append("missing or unexpected artifact_type")
    if payload.get("visualization_schema_version") != VISUALIZATION_SCHEMA_VERSION:
        issues.append("missing or unexpected visualization_schema_version")
    if payload.get("render_profile") != RENDER_PROFILE:
        issues.append("missing or unexpected render_profile")

    caveat_text = " ".join(str(c) for c in payload.get("interpretation_caveats") or [])
    if "unified authoritative replay state" not in caveat_text:
        issues.append("visualization manifest must reject unified authoritative replay-state interpretation")
    if "causal proof" not in caveat_text:
        issues.append("visualization manifest must reject causal-proof interpretation")

    return {
        "artifact_type": "governance_lint_result",
        "governance": _governance_block(),
        "ok": not issues,
        "issues": issues,
    }


def _resolve_optional_figure_flags(
    lineage: dict[str, Any],
    *,
    include_engagement_series: bool,
    include_sparse_topdown: bool,
    no_optional_figures: bool,
) -> tuple[bool, bool]:
    """Auto-enable optional figures when log_path is readable unless disabled."""

    if no_optional_figures:
        return False, False
    if include_engagement_series or include_sparse_topdown:
        return include_engagement_series, include_sparse_topdown
    log_path = lineage.get("log_path")
    if log_path and Path(str(log_path)).is_file():
        return True, True
    return include_engagement_series, include_sparse_topdown


def _normalize_figure_path(figure: dict[str, Any], out_dir: Path) -> dict[str, Any]:
    record = dict(figure)
    raw_path = Path(str(record.get("path") or ""))
    try:
        record["path"] = str(raw_path.relative_to(out_dir))
    except ValueError:
        record["path"] = raw_path.name
    return record


def render_figures(
    manifest: dict[str, Any],
    narrative: dict[str, Any],
    out_dir: Path,
    *,
    include_engagement_series: bool = False,
    include_sparse_topdown: bool = False,
) -> dict[str, Any]:
    """Render figures into out_dir and return manifest with populated figures[]."""

    out_dir.mkdir(parents=True, exist_ok=True)
    figures: list[dict[str, Any]] = []
    skipped = list(manifest.get("skipped_figures") or [])

    from replay_viz_figures import render_divergence_overlay  # noqa: E402
    from replay_viz_figures import render_lifecycle_strip  # noqa: E402
    from replay_viz_figures import render_timeline_band  # noqa: E402

    figures.append(_normalize_figure_path(render_timeline_band(narrative, out_dir / "timeline_band.png"), out_dir))
    figures.append(_normalize_figure_path(render_divergence_overlay(narrative, out_dir / "divergence_overlay.png"), out_dir))
    figures.append(_normalize_figure_path(render_lifecycle_strip(narrative, out_dir / "lifecycle_strip.png"), out_dir))
    figures.append(
        _normalize_figure_path(render_comprehension_panel(narrative, out_dir / "comprehension_panel.png"), out_dir)
    )

    lineage = manifest.get("lineage") if isinstance(manifest.get("lineage"), dict) else {}
    log_path = Path(str(lineage.get("log_path") or ""))
    meta_path = Path(str(lineage.get("meta_path") or "")) if lineage.get("meta_path") else None

    if include_engagement_series:
        if log_path.is_file():
            fig = render_engagement_series(log_path, out_dir / "engagement_series.png")
            if fig is None:
                skipped.append(
                    {
                        "figure_category": "engagement_series",
                        "reason": "no_parseable_metrics_evidence",
                    }
                )
            else:
                figures.append(_normalize_figure_path(fig, out_dir))
        else:
            skipped.append(
                {
                    "figure_category": "engagement_series",
                    "reason": "no_parseable_metrics_evidence (missing log_path)",
                }
            )

    if include_sparse_topdown:
        if log_path.is_file():
            fig = render_sparse_topdown(log_path, out_dir / "sparse_topdown.png", meta_path=meta_path)
            if fig is None:
                skipped.append(
                    {
                        "figure_category": "sparse_topdown",
                        "reason": "no_parseable_position_evidence",
                    }
                )
            else:
                figures.append(_normalize_figure_path(fig, out_dir))
        else:
            skipped.append(
                {
                    "figure_category": "sparse_topdown",
                    "reason": "no_parseable_position_evidence (missing log_path)",
                }
            )

    updated = dict(manifest)
    updated["figures"] = sorted(figures, key=lambda f: str(f.get("figure_id") or ""))
    updated["skipped_figures"] = skipped
    updated["summary"] = dict(updated.get("summary") or {})
    updated["summary"]["figure_count_rendered"] = len(figures)
    updated["summary"]["figure_count_skipped"] = len(skipped)
    return updated


def build_composite_report(
    narrative: dict[str, Any],
    *,
    observability: dict[str, Any] | None = None,
    out_dir: Path,
    input_paths: dict[str, str] | None = None,
    include_engagement_series: bool = False,
    include_sparse_topdown: bool = False,
    no_optional_figures: bool = False,
) -> dict[str, Any]:
    """Build manifest, render figures, and write composite HTML."""

    lineage = _copy_lineage(narrative, observability)
    include_engagement_series, include_sparse_topdown = _resolve_optional_figure_flags(
        lineage,
        include_engagement_series=include_engagement_series,
        include_sparse_topdown=include_sparse_topdown,
        no_optional_figures=no_optional_figures,
    )

    manifest = build_visualization_manifest(
        narrative,
        observability=observability,
        input_paths=input_paths,
        include_engagement_series=include_engagement_series,
        include_sparse_topdown=include_sparse_topdown,
    )
    manifest = render_figures(
        manifest,
        narrative,
        out_dir,
        include_engagement_series=include_engagement_series,
        include_sparse_topdown=include_sparse_topdown,
    )
    _write_json(out_dir / "replay_static_visualization.json", manifest)
    html_text = render_composite_html(manifest, base_dir=out_dir)
    (out_dir / "replay_static_visualization.html").write_text(html_text, encoding="utf-8")
    return manifest


def _cmd_manifest(args: argparse.Namespace) -> int:
    narrative = _read_json(args.narrative_json)
    observability = _read_json(args.observability_json) if args.observability_json else None
    manifest = build_visualization_manifest(
        narrative,
        observability=observability,
        input_paths={
            "narrative_json": str(args.narrative_json),
            **({"observability_json": str(args.observability_json)} if args.observability_json else {}),
        },
        include_engagement_series=bool(args.engagement_series),
        include_sparse_topdown=bool(args.sparse_topdown),
    )
    _write_json(args.out_json, manifest)
    return 0


def _cmd_figures(args: argparse.Namespace) -> int:
    narrative = _read_json(args.narrative_json)
    observability = _read_json(args.observability_json) if args.observability_json else None
    manifest = build_visualization_manifest(
        narrative,
        observability=observability,
        input_paths={"narrative_json": str(args.narrative_json)},
        include_engagement_series=bool(args.engagement_series),
        include_sparse_topdown=bool(args.sparse_topdown),
    )
    manifest = render_figures(
        manifest,
        narrative,
        args.out_dir,
        include_engagement_series=bool(args.engagement_series),
        include_sparse_topdown=bool(args.sparse_topdown),
    )
    _write_json(args.out_dir / "replay_static_visualization.json", manifest)
    return 0


def _cmd_composite(args: argparse.Namespace) -> int:
    narrative = _read_json(args.narrative_json)
    observability = _read_json(args.observability_json) if args.observability_json else None
    build_composite_report(
        narrative,
        observability=observability,
        out_dir=args.out_dir,
        input_paths={
            "narrative_json": str(args.narrative_json),
            **({"observability_json": str(args.observability_json)} if args.observability_json else {}),
        },
        include_engagement_series=bool(args.engagement_series),
        include_sparse_topdown=bool(args.sparse_topdown),
        no_optional_figures=bool(args.no_optional_figures),
    )
    return 0


def _cmd_governance_lint(args: argparse.Namespace) -> int:
    payload = _read_json(args.in_json)
    result = lint_governance_artifact(payload)
    _write_json(args.out_json, result)
    return 0 if result.get("ok") else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Static replay visualization (evaluation-side only).")
    sub = parser.add_subparsers(dest="command", required=True)

    p_manifest = sub.add_parser("manifest", help="Build visualization manifest JSON only.")
    p_manifest.add_argument("--narrative-json", type=Path, required=True)
    p_manifest.add_argument("--observability-json", type=Path, default=None)
    p_manifest.add_argument("--out-json", type=Path, required=True)
    p_manifest.add_argument("--engagement-series", action="store_true")
    p_manifest.add_argument("--sparse-topdown", action="store_true")
    p_manifest.set_defaults(func=_cmd_manifest)

    p_figures = sub.add_parser("figures", help="Render figure files and manifest into out-dir.")
    p_figures.add_argument("--narrative-json", type=Path, required=True)
    p_figures.add_argument("--observability-json", type=Path, default=None)
    p_figures.add_argument("--out-dir", type=Path, required=True)
    p_figures.add_argument("--engagement-series", action="store_true")
    p_figures.add_argument("--sparse-topdown", action="store_true")
    p_figures.set_defaults(func=_cmd_figures)

    p_composite = sub.add_parser("composite", help="Render figures and composite HTML report.")
    p_composite.add_argument("--narrative-json", type=Path, required=True)
    p_composite.add_argument("--observability-json", type=Path, default=None)
    p_composite.add_argument("--out-dir", type=Path, required=True)
    p_composite.add_argument("--engagement-series", action="store_true")
    p_composite.add_argument("--sparse-topdown", action="store_true")
    p_composite.add_argument(
        "--no-optional-figures",
        action="store_true",
        help="Disable auto-enabled engagement and sparse top-down figures when log_path exists.",
    )
    p_composite.set_defaults(func=_cmd_composite)

    p_lint = sub.add_parser("governance-lint", help="Lint a visualization manifest JSON.")
    p_lint.add_argument("--in-json", type=Path, required=True)
    p_lint.add_argument("--out-json", type=Path, required=True)
    p_lint.set_defaults(func=_cmd_governance_lint)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
