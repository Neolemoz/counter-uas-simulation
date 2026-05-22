"""3D heatmap export: PNG (matplotlib) or interactive HTML (Plotly) from CSV / cell list."""

from __future__ import annotations

import csv
import html
import json
import math
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


def load_heatmap_prob_csv(path: Path) -> list[tuple[float, float, float, float]]:
    """Rows: x_m, y_m, z_m, p_hit_noise_model."""
    rows: list[tuple[float, float, float, float]] = []
    with path.open(encoding='utf-8', newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(
                (
                    float(row['x_m']),
                    float(row['y_m']),
                    float(row['z_m']),
                    float(row['p_hit_noise_model']),
                ),
            )
    return rows


# Match RViz / SVG semantics: green = high P, red = low P (see _heatmap_rgba_for_score).
_HEATMAP_CMAP = LinearSegmentedColormap.from_list(
    'intercept_prob',
    [(0.92, 0.18, 0.14), (0.95, 0.82, 0.12), (0.15, 0.85, 0.22)],
)


def write_intercept_heatmap_prob_3d_png(
    cells: Sequence[tuple[float, float, float, float]],
    out_png: Path,
    *,
    title: str = 'P(hit | noise model) — 3D cells',
    subtitle: str = '',
    figsize: tuple[float, float] = (11.0, 9.0),
    dpi: int = 160,
    elev: float = 24.0,
    azim: float = -58.0,
    marker_scale: float = 1.0,
    facecolor: str = '#1a1a1e',
    grid: bool = True,
    z_exaggeration: float | None = None,
) -> None:
    """
    Scatter in 3D: position (x,y,z), color = probability. Writes PNG (axes in metres).

    ``z_exaggeration`` scales the Z dimension in ``set_box_aspect`` so shallow envelopes
    (e.g. ~300 m over ~12 km XY) stay visible. ``None`` = auto when z varies.
    If every cell shares the same z, the cloud is genuinely flat until CSV is rebuilt with
    e.g. ``intercept_heatmap_prob_threat_z_mode:=los_ramp``.
    """
    if not cells:
        raise ValueError('cells is empty')
    out_png.parent.mkdir(parents=True, exist_ok=True)

    xs = np.array([c[0] for c in cells], dtype=np.float64)
    ys = np.array([c[1] for c in cells], dtype=np.float64)
    zs = np.array([c[2] for c in cells], dtype=np.float64)
    ps = np.clip(np.array([c[3] for c in cells], dtype=np.float64), 0.0, 1.0)
    n = len(cells)

    span_xy, box_z, _z_ex, _flat_z, warn_flat = _heatmap_3d_box_params(
        xs, ys, zs, z_exaggeration=z_exaggeration,
    )
    combined_sub = (warn_flat + subtitle).strip()

    base = max(12.0, 520.0 / math.sqrt(float(n)))
    s = (base * float(marker_scale)) ** 2

    fig = plt.figure(figsize=figsize, facecolor=facecolor)
    ax = fig.add_subplot(111, projection='3d', facecolor=facecolor)
    sc = ax.scatter(
        xs,
        ys,
        zs,
        c=ps,
        cmap=_HEATMAP_CMAP,
        vmin=0.0,
        vmax=1.0,
        s=s,
        alpha=0.9,
        linewidths=0.25,
        edgecolors='#0d0d10',
        depthshade=True,
    )
    ax.set_xlabel('x (m)', color='#e8e8ec')
    ax.set_ylabel('y (m)', color='#e8e8ec')
    ax.set_zlabel('z (m)', color='#e8e8ec')
    ax.tick_params(colors='#c0c4c8')
    ax.xaxis.pane.set_facecolor('#252528')
    ax.yaxis.pane.set_facecolor('#252528')
    ax.zaxis.pane.set_facecolor('#252528')
    ax.xaxis.pane.set_edgecolor('#404048')
    ax.yaxis.pane.set_edgecolor('#404048')
    ax.zaxis.pane.set_edgecolor('#404048')
    if grid:
        ax.grid(True, color='#3a3a42', linestyle='--', linewidth=0.4, alpha=0.7)

    ax.set_box_aspect((span_xy, span_xy, box_z))

    cbar = fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.02, aspect=22)
    cbar.set_label('P(hit)', color='#e8e8ec')
    cbar.ax.yaxis.set_tick_params(color='#c0c4c8')
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='#c0c4c8')

    ax.set_title(title, color='#e8e8ec', fontsize=12, pad=12)
    if combined_sub:
        fig.text(0.5, 0.02, combined_sub, ha='center', color='#9aa0a6', fontsize=9)

    ax.view_init(elev=elev, azim=azim)
    fig.tight_layout()
    fig.savefig(out_png, dpi=dpi, facecolor=facecolor, edgecolor='none')
    plt.close(fig)


def _heatmap_3d_box_params(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    *,
    z_exaggeration: float | None,
) -> tuple[float, float, float, bool, str]:
    """span_xy, box_z, z_ex (multiplier), flat_z, warn_flat."""
    span_xy = max(float(np.ptp(xs)), float(np.ptp(ys)), 200.0)
    ptp_z = float(np.ptp(zs))
    flat_z = ptp_z < 1e-6
    span_z = 1.0 if flat_z else max(ptp_z, 1.0)

    if z_exaggeration is None:
        if flat_z:
            z_ex = 1.0
        else:
            z_ex = min(48.0, max(1.0, 0.1 * span_xy / span_z))
    else:
        z_ex = max(0.25, float(z_exaggeration))

    box_z = span_z * z_ex if not flat_z else max(span_xy * 0.06, 80.0)
    warn_flat = ''
    if flat_z:
        warn_flat = (
            'Note: z is constant for all cells in this CSV (flat plate). '
            'Re-export with intercept_heatmap_prob_threat_z_mode:=los_ramp or '
            'offline --threat-z-mode los_ramp. '
        )
    return span_xy, box_z, z_ex, flat_z, warn_flat


# Plotly.js for interactive HTML (no pip; needs network once when opening the file unless cached).
_PLOTLY_CDN_DEFAULT = 'https://cdn.plot.ly/plotly-2.35.2.min.js'


def write_intercept_heatmap_prob_3d_html(
    cells: Sequence[tuple[float, float, float, float]],
    out_html: Path,
    *,
    title: str = 'P(hit | noise model) — 3D (interactive)',
    subtitle: str = '',
    z_exaggeration: float | None = None,
    include_plotlyjs: bool | str = True,
    plotly_cdn_url: str | None = None,
) -> None:
    """
    Interactive 3D scatter in HTML (drag to rotate, scroll to zoom).

    Uses Plotly.js from a CDN — **no Python plotly package**. Opening the file needs network
    the first time (or a cached ``plotly*.min.js``). Orbit center is set to the XY centroid with
    ``z = max(z)`` so rotation is anchored high (not at the lowest cells). ``include_plotlyjs`` is
    kept for call-site compatibility; embedding the full library in-file is not implemented.
    """
    if not cells:
        raise ValueError('cells is empty')
    out_html.parent.mkdir(parents=True, exist_ok=True)

    xs = np.array([c[0] for c in cells], dtype=np.float64)
    ys = np.array([c[1] for c in cells], dtype=np.float64)
    zs = np.array([c[2] for c in cells], dtype=np.float64)
    ps = np.clip(np.array([c[3] for c in cells], dtype=np.float64), 0.0, 1.0)
    n = len(cells)

    span_xy, box_z, _z_ex, _flat_z, warn_flat = _heatmap_3d_box_params(
        xs, ys, zs, z_exaggeration=z_exaggeration,
    )
    combined_sub = (warn_flat + subtitle).strip()

    xs_list = [float(x) for x in xs.tolist()]
    ys_list = [float(y) for y in ys.tolist()]
    zs_list = [float(z) for z in zs.tolist()]
    ps_list = [float(p) for p in ps.tolist()]

    rpx = float(max(2.0, min(7.0, 220.0 / math.sqrt(float(n)))))
    ar_z = float(max(box_z / max(span_xy, 1e-9), 0.04))

    cx_data = float((xs.min() + xs.max()) / 2.0)
    cy_data = float((ys.min() + ys.max()) / 2.0)
    z_orbit = float(zs.max())

    annotations: list[dict] = []
    if combined_sub:
        annotations.append(
            {
                'text': combined_sub,
                'xref': 'paper',
                'yref': 'paper',
                'x': 0.5,
                'y': -0.02,
                'xanchor': 'center',
                'yanchor': 'top',
                'showarrow': False,
                'font': {'size': 11, 'color': '#9aa0a6'},
            },
        )

    trace = {
        'type': 'scatter3d',
        'mode': 'markers',
        'x': xs_list,
        'y': ys_list,
        'z': zs_list,
        'marker': {
            'size': rpx,
            'color': ps_list,
            'cmin': 0.0,
            'cmax': 1.0,
            'colorscale': [
                [0.0, 'rgb(235,46,36)'],
                [0.5, 'rgb(242,209,31)'],
                [1.0, 'rgb(38,217,56)'],
            ],
            'colorbar': {
                'title': {'text': 'P(hit)', 'side': 'right'},
                'tickfont': {'color': '#ccc'},
            },
            'line': {'color': 'rgb(13,13,16)', 'width': 0.15},
            'opacity': 0.92,
        },
        'hovertemplate': (
            'x=%{x:.1f} m<br>y=%{y:.1f} m<br>z=%{z:.1f} m<br>P(hit)=%{marker.color:.4f}<extra></extra>'
        ),
    }

    layout = {
        'paper_bgcolor': '#1a1a1e',
        'title': {'text': title, 'font': {'color': '#e8e8ec', 'size': 16}},
        'font': {'color': '#c0c4c8'},
        'margin': {'l': 0, 'r': 0, 't': 50, 'b': 80 if combined_sub else 20},
        'annotations': annotations,
        'scene': {
            'xaxis': {'title': {'text': 'x (m)'}, 'backgroundcolor': '#252528', 'gridcolor': '#3a3a42'},
            'yaxis': {'title': {'text': 'y (m)'}, 'backgroundcolor': '#252528', 'gridcolor': '#3a3a42'},
            'zaxis': {'title': {'text': 'z (m)'}, 'backgroundcolor': '#252528', 'gridcolor': '#3a3a42'},
            'bgcolor': '#1a1a1e',
            'aspectmode': 'manual',
            'aspectratio': {'x': 1, 'y': 1, 'z': ar_z},
            'camera': {'center': {'x': cx_data, 'y': cy_data, 'z': z_orbit}},
        },
        'template': 'plotly_dark',
    }

    cdn = (plotly_cdn_url or _PLOTLY_CDN_DEFAULT).strip()
    if include_plotlyjs is False:
        raise ValueError('include_plotlyjs=False is not supported for CDN HTML export')

    data_json = json.dumps([trace], separators=(',', ':'))
    layout_json = json.dumps(layout, separators=(',', ':'))
    config_json = json.dumps({'scrollZoom': True, 'displaylogo': False}, separators=(',', ':'))

    page = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>{html.escape(title)}</title>
  <script src="{html.escape(cdn, quote=True)}" charset="utf-8"></script>
  <style>body{{margin:0;background:#1a1a1e;}}#chart{{width:100vw;height:100vh;}}</style>
</head>
<body>
  <div id="chart"></div>
  <script>
    const data = {data_json};
    const layout = {layout_json};
    const config = {config_json};
    Plotly.newPlot('chart', data, layout, config);
  </script>
</body>
</html>
"""
    out_html.write_text(page, encoding='utf-8')
