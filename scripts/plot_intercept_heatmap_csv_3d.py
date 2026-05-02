#!/usr/bin/env python3
"""Render intercept_heatmap_prob CSV as a static 3D PNG and/or interactive HTML (Plotly)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from intercept_heatmap_viz_3d import (  # noqa: E402
    load_heatmap_prob_csv,
    write_intercept_heatmap_prob_3d_html,
    write_intercept_heatmap_prob_3d_png,
)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        'csv',
        type=Path,
        nargs='?',
        default=_REPO / 'runs' / 'intercept_heatmap_export' / 'intercept_heatmap_prob_latest.csv',
    )
    p.add_argument(
        '-o',
        '--output',
        type=Path,
        default=None,
        help='PNG path (default: same dir as CSV, intercept_heatmap_prob_latest_3d.png)',
    )
    p.add_argument('--dpi', type=int, default=160)
    p.add_argument('--elev', type=float, default=24.0)
    p.add_argument('--azim', type=float, default=-58.0)
    p.add_argument('--title', type=str, default='P(hit | noise model) — 3D heatmap')
    p.add_argument('--subtitle', type=str, default='')
    p.add_argument('--marker-scale', type=float, default=1.0)
    p.add_argument(
        '--z-exag',
        type=float,
        default=None,
        help='Stretch Z in the plot box; omit for auto (when z is not flat)',
    )
    p.add_argument(
        '--html',
        action='store_true',
        help='also write interactive HTML (drag to rotate; Plotly via CDN)',
    )
    p.add_argument(
        '--html-only',
        action='store_true',
        help='write only HTML (skip PNG); implies --html',
    )
    p.add_argument(
        '--html-out',
        type=Path,
        default=None,
        help='HTML path (default: .../intercept_heatmap_prob_latest_3d.html)',
    )
    p.add_argument(
        '--plotly-cdn-url',
        type=str,
        default=None,
        help='override Plotly.js script URL (default: cdn.plot.ly)',
    )
    args = p.parse_args()
    if args.html_only:
        args.html = True

    csv_path = args.csv.resolve()

    cells = load_heatmap_prob_csv(csv_path)

    if not args.html_only:
        out = args.output
        if out is None:
            out = csv_path.parent / 'intercept_heatmap_prob_latest_3d.png'
        else:
            out = out.resolve()
        write_intercept_heatmap_prob_3d_png(
            cells,
            out,
            title=args.title,
            subtitle=args.subtitle,
            dpi=args.dpi,
            elev=args.elev,
            azim=args.azim,
            marker_scale=args.marker_scale,
            z_exaggeration=args.z_exag,
        )
        print(f'wrote {out} ({len(cells)} cells)')

    if args.html:
        h_out = args.html_out
        if h_out is None:
            h_out = csv_path.parent / 'intercept_heatmap_prob_latest_3d.html'
        else:
            h_out = h_out.resolve()
        write_intercept_heatmap_prob_3d_html(
            cells,
            h_out,
            title=args.title + ' (interactive)',
            subtitle=args.subtitle,
            z_exaggeration=args.z_exag,
            plotly_cdn_url=args.plotly_cdn_url,
        )
        print(f'wrote {h_out} ({len(cells)} cells, open in browser to rotate)')


if __name__ == '__main__':
    main()
