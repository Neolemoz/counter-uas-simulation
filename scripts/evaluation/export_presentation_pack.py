#!/usr/bin/env python3
"""Export E1 presentation and walkthrough review artifacts."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_replay_storytelling import build_storytelling_sections, lint_storytelling_sections  # noqa: E402
from export_replay_analytics_report import export_sweep, _lint_markdown  # noqa: E402
from replay_mc_sweep import validate_sweep  # noqa: E402
from replay_publication_html import (  # noqa: E402
    render_cross_sweep_appendix,
    render_citations_section,
    render_figures_section,
    replay_citation,
    replay_viewer_url,
    wrap_publication_html,
)

_REPO = Path(__file__).resolve().parents[2]
SWEEP_IDS = (
    "valley_sensor_sweep",
    "ridge_overlap_sweep",
    "delayed_detection_sweep",
    "saturation_assignment_sweep",
)


def _load_sweep(sweep_id: str) -> dict[str, Any]:
    path = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "sweep.json"
    return json.loads(path.read_text(encoding="utf-8"))


def render_presentation_summary(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    walk = manifest.get("presentation_walkthrough") or {}
    narrative = manifest.get("replay_narrative_summary") or {}
    lines = [
        f"# Presentation summary — `{sid}`",
        "",
        manifest.get("governance", {}).get("notice", ""),
        "",
        f"## {walk.get('headline', narrative.get('headline', 'Replay presentation'))}",
        "",
    ]
    for step in walk.get("steps") or []:
        lines.append(f"- **{step.get('label')}** ({step.get('kind')}): {step.get('copy')}")
    sections = build_storytelling_sections(manifest=manifest)
    if sections:
        lines.extend(["", "## Storytelling sections", ""])
        for key, text in sections.items():
            lines.append(f"- **{key}**: {text}")
    lines.append("")
    return "\n".join(lines)


def render_guided_walkthrough_report(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    walk = manifest.get("presentation_walkthrough") or {}
    lines = [
        f"# Guided walkthrough report — `{sid}`",
        "",
        manifest.get("governance", {}).get("notice", ""),
        "",
        f"Walkthrough ID: `{walk.get('walkthrough_id', sid)}`",
        "",
    ]
    for i, step in enumerate(walk.get("steps") or []):
        lines.append(f"## Step {i + 1}: {step.get('label')}")
        lines.append("")
        lines.append(f"- Kind: `{step.get('kind')}`")
        lines.append(f"- {step.get('copy')}")
        if step.get("filmstrip_indices"):
            lines.append(f"- Filmstrip indices: `{step.get('filmstrip_indices')}`")
        if step.get("chapter_index") is not None:
            lines.append(f"- Chapter index: `{step.get('chapter_index')}`")
        lines.append("")
    return "\n".join(lines)


def render_topology_walkthrough_report(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    baseline = manifest.get("baseline_topology_key")
    sections = build_storytelling_sections(manifest=manifest, baseline_topology_key=str(baseline))
    lines = [
        f"# Topology walkthrough report — `{sid}`",
        "",
        f"Baseline topology: `{baseline}`",
        "",
    ]
    for key, text in sections.items():
        lines.append(f"## {key.replace('_', ' ').title()}")
        lines.append("")
        lines.append(text)
        lines.append("")
    return "\n".join(lines)


def render_sweep_presentation_packet(manifest: dict[str, Any]) -> str:
    parts = [
        render_presentation_summary(manifest),
        render_guided_walkthrough_report(manifest),
        render_topology_walkthrough_report(manifest),
    ]
    return "\n---\n\n".join(parts)


def render_presentation_report_v1(manifest: dict[str, Any]) -> dict[str, Any]:
    sections = build_storytelling_sections(manifest=manifest)
    return {
        "artifact_type": "replay_presentation_report_v1",
        "schema_version": "replay_presentation_report_v1",
        "sweep_id": manifest.get("sweep_id"),
        "governance": manifest.get("governance"),
        "presentation_walkthrough": manifest.get("presentation_walkthrough"),
        "storytelling_sections": sections,
        "replay_narrative_summary": manifest.get("replay_narrative_summary"),
        "interpretation_caveats": [
            "Presentation report — explanatory replay review only.",
            "Walkthrough steps are deterministic guidance, not tactical recommendations.",
        ],
    }


def _load_synthesis() -> dict[str, Any] | None:
    path = _REPO / "fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json"
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _sweep_figure_specs(sweep_id: str) -> list[dict[str, Any]]:
    fig_dir = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "reports" / "figures"
    specs = [
        (f"{sweep_id}_ambiguity_hotspot.png", "Ambiguity hotspot spatial grid"),
        (f"{sweep_id}_los_degraded.png", "LOS degradation spatial grid"),
        (f"{sweep_id}_topology_sensitivity.png", "Topology sensitivity spatial grid"),
        (f"{sweep_id}_cohort_strip.png", "Cohort pattern strip"),
    ]
    figures: list[dict[str, Any]] = []
    for fname, caption in specs:
        path = fig_dir / fname
        if path.is_file():
            figures.append({"path": str(path), "label": fname, "caption": caption})
    return figures


def _sweep_citations(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    sid = manifest.get("sweep_id")
    citations = [{"label": replay_citation(str(sid)), "url": replay_viewer_url(str(sid))}]
    for m in manifest.get("members") or []:
        mid = m.get("member_id")
        citations.append(
            {
                "label": replay_citation(str(sid), str(mid)),
                "url": replay_viewer_url(str(sid), str(mid)),
            }
        )
    return citations


def render_publication_report_v1(
    manifest: dict[str, Any],
    *,
    synthesis: dict[str, Any] | None = None,
) -> dict[str, Any]:
    sid = manifest.get("sweep_id")
    walk = manifest.get("presentation_walkthrough") or {}
    chapters = [
        {
            "step_id": s.get("step_id"),
            "label": s.get("label"),
            "kind": s.get("kind"),
            "chapter_index": s.get("chapter_index"),
        }
        for s in walk.get("steps") or []
    ]
    return {
        "artifact_type": "replay_publication_report_v1",
        "schema_version": "replay_publication_report_v1",
        "sweep_id": sid,
        "governance": manifest.get("governance"),
        "figures": _sweep_figure_specs(str(sid)),
        "citations": _sweep_citations(manifest),
        "chapters": chapters,
        "cross_sweep_appendix": {
            "synthesis_artifact": "cross_sweep_synthesis_v1.json",
            "included": synthesis is not None,
        },
        "interpretation_caveats": [
            "Publication packet — explanatory replay review only.",
            "Figure numbering is for documentation; not operational scoring.",
        ],
    }


def render_publication_packet_html(
    manifest: dict[str, Any],
    *,
    synthesis: dict[str, Any] | None = None,
    embed_figures: bool = False,
) -> str:
    sid = manifest.get("sweep_id")
    walk = manifest.get("presentation_walkthrough") or {}
    narrative = manifest.get("replay_narrative_summary") or {}
    sections = build_storytelling_sections(manifest=manifest)
    out_dir = _REPO / "fixtures/sa_r0/sweeps" / sid / "reports"

    steps_html = "".join(
        f"<li><strong>{s.get('label')}</strong> ({s.get('kind')}) — {s.get('copy')} "
        f"<span class=\"citation\">{replay_citation(str(sid))}</span></li>"
        for s in walk.get("steps") or []
    )
    story_html = "".join(
        f"<li><strong>{k}</strong>: {v}</li>" for k, v in sections.items()
    )
    figures = _sweep_figure_specs(str(sid))
    body = [
        f"<h1>Publication replay review packet — {sid}</h1>",
        f"<p>{narrative.get('headline', '')}</p>",
        "<h2>Walkthrough chapters</h2>",
        f"<ul>{steps_html}</ul>",
        "<h2>Storytelling</h2>",
        f"<ul>{story_html}</ul>",
        render_figures_section(figures, embed=embed_figures, html_dir=out_dir),
        render_citations_section(_sweep_citations(manifest)),
        render_cross_sweep_appendix(synthesis),
        f'<p class="no-print">Open SA viewer: <a href="{replay_viewer_url(str(sid))}">{replay_viewer_url(str(sid))}</a></p>',
    ]
    return wrap_publication_html(
        title=f"Publication replay review packet — {sid}",
        governance_notice=str(manifest.get("governance", {}).get("notice", "")),
        body_html="\n".join(body),
    )


def render_cross_sweep_publication_report(synthesis: dict[str, Any]) -> str:
    from build_cross_sweep_synthesis import render_cross_sweep_summary  # noqa: WPS433

    base = render_cross_sweep_summary(synthesis)
    lines = [
        "# Cross-sweep publication report",
        "",
        synthesis.get("governance", {}).get("notice", ""),
        "",
        base.split("\n", 1)[1] if "\n" in base else base,
        "",
        "## Cognition rollup",
        "",
    ]
    for b in (synthesis.get("cognition_rollup") or {}).get("bullets") or []:
        lines.append(f"- {b.get('observation')} — replay-local; {b.get('caveat')}")
    lines.append("")
    return "\n".join(lines)


def render_review_packet_html(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    walk = manifest.get("presentation_walkthrough") or {}
    narrative = manifest.get("replay_narrative_summary") or {}
    sections = build_storytelling_sections(manifest=manifest)
    steps_html = "".join(
        f"<li><strong>{s.get('label')}</strong> — {s.get('copy')}</li>"
        for s in walk.get("steps") or []
    )
    story_html = "".join(
        f"<li><strong>{k}</strong>: {v}</li>" for k, v in sections.items()
    )
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>Replay review packet — {sid}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; max-width: 720px; margin: 2rem auto; padding: 0 1rem; }}
    .banner {{ background: #f4f4f5; border-left: 4px solid #71717a; padding: 1rem; margin-bottom: 1.5rem; }}
  </style>
</head>
<body>
  <div class="banner">{manifest.get("governance", {}).get("notice", "")}</div>
  <h1>Replay review packet — {sid}</h1>
  <p>{narrative.get("headline", "")}</p>
  <h2>Walkthrough steps</h2>
  <ul>{steps_html}</ul>
  <h2>Storytelling</h2>
  <ul>{story_html}</ul>
  <p><em>Offline static packet — not live monitoring. Open SA viewer: ?sweep={sid}</em></p>
</body>
</html>
"""


def export_sweep_presentation(
    sweep_id: str,
    *,
    include_html: bool = True,
    publication: bool = False,
    embed_figures: bool = False,
) -> None:
    manifest = _load_sweep(sweep_id)
    errs = validate_sweep(manifest)
    if errs:
        raise SystemExit(f"invalid sweep {sweep_id}: {errs}")
    out_dir = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "reports"
    out_dir.mkdir(parents=True, exist_ok=True)

    md_files = (
        ("presentation_summary.md", render_presentation_summary),
        ("guided_walkthrough_report.md", render_guided_walkthrough_report),
        ("topology_walkthrough_report.md", render_topology_walkthrough_report),
        ("sweep_presentation_packet.md", render_sweep_presentation_packet),
    )
    for filename, renderer in md_files:
        text = renderer(manifest)
        lint = _lint_markdown(text)
        if lint:
            raise SystemExit(f"{filename}: {lint}")
        (out_dir / filename).write_text(text, encoding="utf-8")

    report = render_presentation_report_v1(manifest)
    section_lint = lint_storytelling_sections(report.get("storytelling_sections") or {})
    if section_lint:
        raise SystemExit(f"storytelling lint: {section_lint}")
    (out_dir / "replay_presentation_report_v1.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    if include_html:
        html = render_review_packet_html(manifest)
        (out_dir / "review_packet.html").write_text(html, encoding="utf-8")

    if publication:
        synthesis = _load_synthesis()
        pub_report = render_publication_report_v1(manifest, synthesis=synthesis)
        (out_dir / "replay_publication_report_v1.json").write_text(
            json.dumps(pub_report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        pub_html = render_publication_packet_html(
            manifest, synthesis=synthesis, embed_figures=embed_figures
        )
        (out_dir / "publication_packet.html").write_text(pub_html, encoding="utf-8")

    pub_dir = _REPO / "platform/sa-r0-viewer/public/demo/sweeps" / sweep_id / "reports"
    pub_dir.mkdir(parents=True, exist_ok=True)
    for f in out_dir.iterdir():
        if f.is_file():
            shutil_copy = __import__("shutil").copy2
            shutil_copy(f, pub_dir / f.name)

    export_sweep(sweep_id, formats="md,json")
    print(f"presentation export OK: {sweep_id}")


def export_cross_sweep_publication_report() -> None:
    synthesis = _load_synthesis()
    if not synthesis:
        raise SystemExit("missing cross_sweep_synthesis_v1.json — run gen_e2_research_fixtures.py")
    text = render_cross_sweep_publication_report(synthesis)
    lint = _lint_markdown(text)
    if lint:
        raise SystemExit(f"cross_sweep_publication_report lint: {lint}")
    for d in (
        _REPO / "fixtures/sa_r0/synthesis",
        _REPO / "platform/sa-r0-viewer/public/demo/synthesis",
    ):
        d.mkdir(parents=True, exist_ok=True)
        (d / "cross_sweep_publication_report.md").write_text(text, encoding="utf-8")


def export_all_presentation_packs(
    *,
    include_html: bool = True,
    publication: bool = False,
    embed_figures: bool = False,
) -> None:
    for sweep_id in SWEEP_IDS:
        export_sweep_presentation(
            sweep_id,
            include_html=include_html,
            publication=publication,
            embed_figures=embed_figures,
        )
    if publication:
        export_cross_sweep_publication_report()


def main() -> None:
    ap = argparse.ArgumentParser(description="Export E1 presentation review packs")
    ap.add_argument("--sweep", help="single sweep_id")
    ap.add_argument("--all", action="store_true", help="export all sweeps")
    ap.add_argument("--no-html", action="store_true", help="skip HTML packet")
    ap.add_argument("--publication", action="store_true", help="export publication-grade packets")
    ap.add_argument("--embed-figures", action="store_true", help="base64 embed figures in HTML")
    args = ap.parse_args()
    if args.all:
        export_all_presentation_packs(
            include_html=not args.no_html,
            publication=args.publication,
            embed_figures=args.embed_figures,
        )
    elif args.sweep:
        export_sweep_presentation(
            args.sweep,
            include_html=not args.no_html,
            publication=args.publication,
            embed_figures=args.embed_figures,
        )
    else:
        raise SystemExit("specify --sweep or --all")


if __name__ == "__main__":
    main()
