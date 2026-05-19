"""Composite static HTML assembler for replay visualization."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

from replay_viz_comprehension import CATEGORY_DISPLAY_LABELS
from replay_viz_comprehension import FIGURE_LOOK_FOR
from replay_viz_comprehension import FIGURE_REVIEWER_TITLES

NON_AUTHORITATIVE_BANNER = (
    "Non-authoritative static replay visualization. Explanatory visualization layer only. "
    "Does not replace parser-visible summaries, runtime topics, tactical authority, "
    "lifecycle semantics, or replay contracts."
)


def _cell(value: Any) -> str:
    return html.escape("" if value is None else str(value))


def _nav_link(anchor: str, label: str) -> str:
    return f'<a href="#{_cell(anchor)}">{_cell(label)}</a>'


def _figure_section(
    figure: dict[str, Any],
    *,
    base_dir: Path | None = None,
    reviewer_title: str = "",
    look_for: str = "",
) -> str:
    rel_path = figure.get("path")
    if not rel_path:
        return ""
    img_path = Path(str(rel_path))
    if base_dir is not None and img_path.is_absolute():
        try:
            img_path = img_path.relative_to(base_dir)
        except ValueError:
            pass
    src = _cell(str(img_path))
    caption = _cell(figure.get("interpretation_caveat") or "")
    category = str(figure.get("category") or "")
    title = reviewer_title or FIGURE_REVIEWER_TITLES.get(category, category.replace("_", " ").title())
    guidance = look_for or FIGURE_LOOK_FOR.get(category, "")
    figure_id = _cell(figure.get("figure_id") or "")
    guidance_html = (
        f'<p class="look-for"><strong>What to look for:</strong> {_cell(guidance)}</p>' if guidance else ""
    )
    return (
        f'<section class="figure" id="figure-{_cell(category)}">'
        f"<h3>{_cell(title)} <span class=\"figure-id\">({figure_id})</span></h3>"
        f"{guidance_html}"
        f'<img src="{src}" alt="{_cell(title)} figure" />'
        f'<p class="caveat">{caption}</p>'
        f"</section>"
    )


def _ordered_figures(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    figures = list(manifest.get("figures") or [])
    comprehension = manifest.get("comprehension") if isinstance(manifest.get("comprehension"), dict) else {}
    display_order = list(comprehension.get("figure_display_order") or [])
    order_index = {cat: idx for idx, cat in enumerate(display_order)}
    return sorted(
        figures,
        key=lambda f: (
            order_index.get(str(f.get("category") or ""), 999),
            str(f.get("figure_id") or ""),
        ),
    )


def _render_scan_guide(comprehension: dict[str, Any]) -> str:
    items = "".join(f"<li>{_cell(b)}</li>" for b in comprehension.get("scan_guide") or [])
    return f'<section id="read-first"><h2>Read this first</h2><ul class="scan-guide">{items}</ul></section>'


def _render_at_a_glance(comprehension: dict[str, Any]) -> str:
    headline = _cell(comprehension.get("headline") or "")
    cards = comprehension.get("at_a_glance") or []
    card_html = "".join(
        f'<div class="glance-card"><span class="glance-label">{_cell(c.get("label"))}</span>'
        f'<span class="glance-value">{_cell(c.get("value"))}</span></div>'
        for c in cards
    )
    return (
        f'<section id="at-a-glance"><h2>At a glance</h2>'
        f'<p class="headline">{headline}</p>'
        f'<div class="glance-grid">{card_html}</div>'
        f"</section>"
    )


def _render_incidents(comprehension: dict[str, Any]) -> str:
    groups = comprehension.get("incident_groups") or []
    if not groups:
        return '<section id="incidents"><h2>Key incidents</h2><p><em>No grouped incidents.</em></p></section>'
    blocks: list[str] = []
    for group in groups:
        category = str(group.get("category") or "")
        chip = CATEGORY_DISPLAY_LABELS.get(category, category)
        rows = group.get("rows") or []
        row_html = "".join(
            "<tr>"
            f"<td>{_cell(r.get('time_label'))}</td>"
            f"<td>{_cell(r.get('label'))}</td>"
            f"<td>{_cell(r.get('line_index'))}</td>"
            "</tr>"
            for r in rows
        )
        blocks.append(
            f'<div class="incident-group">'
            f'<h3><span class="chip">{_cell(chip)}</span></h3>'
            f"<table><thead><tr><th>When</th><th>Incident</th><th>Line</th></tr></thead>"
            f"<tbody>{row_html}</tbody></table></div>"
        )
    return f'<section id="incidents"><h2>Key incidents</h2>{"".join(blocks)}</section>'


def _render_key_windows(comprehension: dict[str, Any]) -> str:
    windows = comprehension.get("key_windows") or []
    if not windows:
        return ""
    rows = "".join(
        "<tr>"
        f"<td>{_cell(w.get('window_label'))}</td>"
        f"<td>{_cell(w.get('start_line_index'))}</td>"
        f"<td>{_cell(w.get('end_line_index'))}</td>"
        f"<td>{_cell(w.get('start_event_label'))}</td>"
        "</tr>"
        for w in windows
    )
    return (
        '<section id="windows"><h2>Localized windows</h2>'
        "<table><thead><tr><th>Window</th><th>Start line</th><th>End line</th><th>Start event</th></tr></thead>"
        f"<tbody>{rows}</tbody></table></section>"
    )


def _render_timeline_table(comprehension: dict[str, Any]) -> str:
    rows_data = comprehension.get("timeline_rows") or []
    if not rows_data:
        return ""
    rows = "".join(
        "<tr>"
        f"<td>{_cell(r.get('time_label'))}</td>"
        f"<td>{_cell(r.get('category_label'))}</td>"
        f"<td>{_cell(r.get('label'))}</td>"
        f"<td>{_cell(r.get('line_index'))}</td>"
        "</tr>"
        for r in rows_data
    )
    return (
        '<section id="timeline-table"><h2>Event timeline (table)</h2>'
        '<p class="section-note">Chronological narrative events for text-first review; not causal proof.</p>'
        "<table><thead><tr><th>When</th><th>Category</th><th>Event</th><th>Line</th></tr></thead>"
        f"<tbody>{rows}</tbody></table></section>"
    )


def render_composite_html(
    manifest: dict[str, Any],
    *,
    base_dir: Path | None = None,
) -> str:
    """Render standalone composite HTML from a visualization manifest."""

    lineage = manifest.get("lineage") if isinstance(manifest.get("lineage"), dict) else {}
    summary = manifest.get("summary") if isinstance(manifest.get("summary"), dict) else {}
    comprehension = manifest.get("comprehension") if isinstance(manifest.get("comprehension"), dict) else {}
    warnings = manifest.get("warnings") or []
    caveats = manifest.get("interpretation_caveats") or []
    skipped = manifest.get("skipped_figures") or []

    warning_items = "".join(f"<li>{_cell(w)}</li>" for w in warnings) or "<li>No warnings reported.</li>"
    caveat_items = "".join(f"<li>{_cell(c)}</li>" for c in caveats)
    skipped_items = "".join(
        f"<li><strong>{_cell(s.get('figure_category'))}</strong>: {_cell(s.get('reason'))}</li>"
        for s in skipped
    )

    figure_titles = comprehension.get("figure_reviewer_titles")
    if not isinstance(figure_titles, dict):
        figure_titles = FIGURE_REVIEWER_TITLES
    figure_look_for = comprehension.get("figure_look_for")
    if not isinstance(figure_look_for, dict):
        figure_look_for = FIGURE_LOOK_FOR

    figure_blocks = "".join(
        _figure_section(
            f,
            base_dir=base_dir,
            reviewer_title=str(figure_titles.get(str(f.get("category") or ""), "")),
            look_for=str(figure_look_for.get(str(f.get("category") or ""), "")),
        )
        for f in _ordered_figures(manifest)
    )

    summary_rows = "".join(
        f"<tr><th>{_cell(k)}</th><td>{_cell(v)}</td></tr>"
        for k, v in sorted(summary.items())
    )

    nav = " · ".join(
        [
            _nav_link("read-first", "Read first"),
            _nav_link("at-a-glance", "At a glance"),
            _nav_link("incidents", "Incidents"),
            _nav_link("figures", "Figures"),
            _nav_link("timeline-table", "Timeline"),
            _nav_link("lineage", "Lineage"),
            _nav_link("caveats", "Caveats"),
        ]
    )

    scan_section = _render_scan_guide(comprehension) if comprehension else ""
    glance_section = _render_at_a_glance(comprehension) if comprehension else ""
    incidents_section = _render_incidents(comprehension) if comprehension else ""
    windows_section = _render_key_windows(comprehension) if comprehension else ""
    timeline_section = _render_timeline_table(comprehension) if comprehension else ""

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>Replay Static Visualization Report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; max-width: 980px; margin: 2rem auto; line-height: 1.45; color: #222; }}
    .banner {{ background: #fff8e1; border: 1px solid #f0d080; padding: 0.75rem 1rem; margin-bottom: 1rem; }}
    .watermark {{ color: #555; font-size: 0.95rem; margin-bottom: 1rem; }}
    nav.report-nav {{ font-size: 0.92rem; margin-bottom: 1.5rem; padding: 0.5rem 0; border-bottom: 1px solid #e0e0e0; }}
    nav.report-nav a {{ margin-right: 0.25rem; color: #1a5fb4; text-decoration: none; }}
    nav.report-nav a:hover {{ text-decoration: underline; }}
    section {{ margin: 1.75rem 0; }}
    h1 {{ font-size: 1.5rem; }}
    h2 {{ font-size: 1.15rem; border-bottom: 1px solid #eee; padding-bottom: 0.25rem; }}
    h3 {{ font-size: 1rem; }}
    table {{ border-collapse: collapse; width: 100%; font-size: 0.92rem; }}
    th, td {{ border: 1px solid #ddd; padding: 0.4rem 0.6rem; text-align: left; vertical-align: top; }}
    th {{ background: #f6f8fa; }}
    img {{ max-width: 100%; height: auto; border: 1px solid #ddd; margin: 0.5rem 0; }}
    .figure-id {{ color: #666; font-weight: normal; font-size: 0.85rem; }}
    .caveat, .look-for, .section-note {{ color: #444; font-size: 0.92rem; }}
    .headline {{ font-size: 1.05rem; margin: 0.5rem 0 1rem; }}
    ul.scan-guide {{ padding-left: 1.25rem; }}
    .glance-grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(10rem, 1fr)); gap: 0.75rem; }}
    .glance-card {{ border: 1px solid #e0e0e0; border-radius: 4px; padding: 0.6rem 0.75rem; background: #fafbfc; }}
    .glance-label {{ display: block; font-size: 0.8rem; color: #666; text-transform: uppercase; letter-spacing: 0.02em; }}
    .glance-value {{ display: block; font-weight: 600; margin-top: 0.2rem; }}
    .incident-group {{ margin-bottom: 1.25rem; }}
    .chip {{ display: inline-block; padding: 0.15rem 0.5rem; border-radius: 3px; font-size: 0.85rem; background: #eef2f7; }}
    details.technical {{ margin-top: 1rem; }}
  </style>
</head>
<body>
  <div class="banner"><strong>{_cell(NON_AUTHORITATIVE_BANNER)}</strong></div>
  <p class="watermark">Derived evaluation artifact only; not validation, certification, or readiness evidence.</p>
  <h1>Replay Static Visualization Report</h1>
  <nav class="report-nav" aria-label="Report sections">{nav}</nav>
  {scan_section}
  {glance_section}
  {incidents_section}
  {windows_section}
  <section id="figures"><h2>Figures</h2>
    {figure_blocks or "<p><em>No figures rendered.</em></p>"}
  </section>
  {timeline_section}
  <section id="lineage"><h2>Lineage</h2>
    <table>
      <tr><th>run_id</th><td>{_cell(lineage.get("run_id"))}</td></tr>
      <tr><th>log_path</th><td>{_cell(lineage.get("log_path"))}</td></tr>
      <tr><th>meta_path</th><td>{_cell(lineage.get("meta_path"))}</td></tr>
      <tr><th>seed</th><td>{_cell(lineage.get("seed"))}</td></tr>
      <tr><th>cohort</th><td>{_cell(lineage.get("cohort"))}</td></tr>
    </table>
  </section>
  <details class="technical">
    <summary>Technical manifest summary (for authors)</summary>
    <table>{summary_rows}</table>
  </details>
  <section><h2>Warnings</h2><ul>{warning_items}</ul></section>
  <section><h2>Skipped figures</h2><ul>{skipped_items or "<li>None.</li>"}</ul></section>
  <section id="caveats"><h2>Interpretation caveats</h2><ul>{caveat_items}</ul></section>
</body>
</html>
"""
