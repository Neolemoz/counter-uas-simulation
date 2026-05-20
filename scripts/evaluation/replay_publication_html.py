#!/usr/bin/env python3
"""Print-optimized HTML helpers for E2 publication packets."""

from __future__ import annotations

import base64
from pathlib import Path
from typing import Any


PRINT_CSS = """
@media screen {
  body { font-family: Georgia, 'Times New Roman', serif; max-width: 820px; margin: 2rem auto; padding: 0 1.5rem; line-height: 1.5; color: #1a1a1a; }
  .banner { background: #f4f4f5; border-left: 4px solid #71717a; padding: 1rem; margin-bottom: 1.5rem; font-size: 0.9rem; }
  figure { margin: 1.5rem 0; }
  figure img { max-width: 100%; height: auto; border: 1px solid #ddd; }
  figcaption { font-size: 0.85rem; color: #444; margin-top: 0.5rem; }
  .citation { font-family: ui-monospace, monospace; font-size: 0.85rem; color: #555; }
  .appendix { border-top: 2px solid #ccc; margin-top: 2rem; padding-top: 1rem; }
}
@media print {
  body { font-size: 11pt; max-width: none; margin: 0; padding: 0.5in; }
  .banner { background: #eee; border-left: 3pt solid #666; page-break-inside: avoid; }
  h1, h2 { page-break-after: avoid; }
  figure { page-break-inside: avoid; }
  .no-print { display: none; }
  @page { margin: 0.75in; @bottom-center { content: "Replay review packet — explanatory only"; font-size: 8pt; color: #666; } }
}
"""


def replay_citation(sweep_id: str, member_id: str | None = None) -> str:
    if member_id:
        return f"[Replay: {sweep_id}/{member_id}]"
    return f"[Replay: {sweep_id}]"


def replay_viewer_url(sweep_id: str, member_id: str | None = None) -> str:
    if member_id:
        return f"?sweep={sweep_id}&member={member_id}"
    return f"?sweep={sweep_id}"


def figure_img_tag(
    figure_path: Path,
    *,
    figure_num: int,
    caption: str,
    embed: bool = False,
    rel_from_html: Path | None = None,
) -> str:
    if embed and figure_path.is_file():
        data = base64.b64encode(figure_path.read_bytes()).decode("ascii")
        src = f"data:image/png;base64,{data}"
    elif rel_from_html and figure_path.is_file():
        src = str(figure_path.relative_to(rel_from_html.parent)).replace("\\", "/")
    elif figure_path.is_file():
        src = figure_path.name
    else:
        src = ""
    if not src:
        return f'<figure id="fig{figure_num}"><figcaption>Figure {figure_num}. {caption} (figure not generated)</figcaption></figure>'
    return (
        f'<figure id="fig{figure_num}">'
        f'<img src="{src}" alt="{caption}"/>'
        f'<figcaption>Figure {figure_num}. {caption}</figcaption>'
        f"</figure>"
    )


def render_figures_section(
    figures: list[dict[str, Any]],
    *,
    embed: bool = False,
    html_dir: Path | None = None,
) -> str:
    parts: list[str] = ['<section class="figures"><h2>Figures</h2>']
    for i, fig in enumerate(figures, start=1):
        path = Path(fig.get("path", ""))
        parts.append(
            figure_img_tag(
                path,
                figure_num=i,
                caption=str(fig.get("caption", fig.get("label", "Replay figure"))),
                embed=embed,
                rel_from_html=html_dir,
            )
        )
    parts.append("</section>")
    return "\n".join(parts)


def render_citations_section(citations: list[dict[str, Any]]) -> str:
    if not citations:
        return ""
    items = "".join(
        f'<li class="citation"><a href="{c.get("url", "#")}">{c.get("label", "")}</a></li>'
        for c in citations
    )
    return f'<section class="citations"><h2>Replay references</h2><ul>{items}</ul></section>'


def render_cross_sweep_appendix(synthesis: dict[str, Any] | None) -> str:
    if not synthesis:
        return ""
    bullets = (synthesis.get("cognition_rollup") or {}).get("bullets") or []
    if not bullets:
        caveats = synthesis.get("interpretation_caveats") or []
        items = "".join(f"<li>{c}</li>" for c in caveats)
    else:
        items = "".join(
            f"<li>{b.get('observation')} — replay-local; {b.get('caveat')}</li>" for b in bullets
        )
    return (
        '<section class="appendix"><h2>Cross-sweep appendix</h2>'
        f"<ul>{items}</ul></section>"
    )


def wrap_publication_html(
    *,
    title: str,
    governance_notice: str,
    body_html: str,
) -> str:
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>{title}</title>
  <style>{PRINT_CSS}</style>
</head>
<body>
  <div class="banner">{governance_notice}</div>
  {body_html}
  <p class="no-print"><em>Offline static publication packet — not live monitoring.</em></p>
</body>
</html>
"""
