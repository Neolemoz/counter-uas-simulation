#!/usr/bin/env python3
"""Export intercept P(hit) heatmap CSV+SVG without ROS/Gazebo (same math as interception_logic_node).

Use ``--png-3d`` to also write ``intercept_heatmap_prob_latest_3d.png`` (matplotlib 3D scatter).
Use ``--html-3d`` for ``intercept_heatmap_prob_latest_3d.html`` (interactive Plotly.js via CDN).
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SCRIPTS = Path(__file__).resolve().parent
if str(_REPO / 'src' / 'gazebo_target_sim') not in sys.path:
    sys.path.insert(0, str(_REPO / 'src' / 'gazebo_target_sim'))
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from intercept_heatmap_viz_3d import (  # noqa: E402
    write_intercept_heatmap_prob_3d_html,
    write_intercept_heatmap_prob_3d_png,
)

from gazebo_target_sim.interception_logic_node import (  # noqa: E402
    _build_dome_disk_xy_offsets,
    estimate_hit_probability,
    export_intercept_heatmap_prob_to_disk,
    intercept_heatmap_prob_cell_velocity,
    intercept_heatmap_prob_threat_altitude_m,
)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out-dir', type=Path, default=_REPO / 'runs' / 'intercept_heatmap_export')
    p.add_argument('--r-outer', type=float, default=6000.0)
    p.add_argument('--grid-step', type=float, default=300.0)
    p.add_argument('--dome-cx', type=float, default=0.0)
    p.add_argument('--dome-cy', type=float, default=0.0)
    p.add_argument('--dome-cz', type=float, default=0.0)
    p.add_argument('--z-offset', type=float, default=0.0)
    p.add_argument(
        '--threat-z-mode',
        type=str,
        default='los_ramp',
        help='flat | los_ramp (low at asset, high at r_ref) | los_ramp_center_high (peak at asset)',
    )
    p.add_argument('--threat-z-inner-m', type=float, default=0.0)
    p.add_argument('--threat-z-outer-m', type=float, default=300.0)
    p.add_argument(
        '--threat-z-r-ref-m',
        type=float,
        default=0.0,
        help='r_ref for los_ramp; 0 = use --r-outer',
    )
    p.add_argument('--ix', type=float, default=-5.0)
    p.add_argument('--iy', type=float, default=0.0)
    p.add_argument('--iz', type=float, default=0.0)
    p.add_argument('--v-i-max', type=float, default=35.0, help='interceptor max speed (launch guidance_vmax order)')
    p.add_argument('--t-min', type=float, default=0.5)
    p.add_argument('--t-max', type=float, default=300.0)
    p.add_argument('--hit-thresh', type=float, default=1.0)
    p.add_argument('--los-spd', type=float, default=30.0)
    p.add_argument('--vx-param', type=float, default=0.0)
    p.add_argument('--vy-param', type=float, default=0.0)
    p.add_argument('--vz-param', type=float, default=0.0)
    p.add_argument('--vx-smooth', type=float, default=0.0, help='fallback track velocity for center cell')
    p.add_argument('--vy-smooth', type=float, default=0.0)
    p.add_argument('--vz-smooth', type=float, default=0.0)
    p.add_argument('--mc-n', type=int, default=12)
    p.add_argument('--pos-sigma', type=float, default=22.0)
    p.add_argument('--vel-sigma', type=float, default=1.35)
    p.add_argument('--inter-sigma', type=float, default=3.0)
    p.add_argument('--delay-mean', type=float, default=2.0)
    p.add_argument('--delay-jitter', type=float, default=0.28)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--stamp-svg', action='store_true', help='also write intercept_heatmap_prob_stamped.*')
    p.add_argument('--png-3d', action='store_true', help='also write intercept_heatmap_prob_latest_3d.png (matplotlib)')
    p.add_argument(
        '--png-3d-path',
        type=Path,
        default=None,
        help='override output path for 3D PNG',
    )
    p.add_argument('--png-3d-elev', type=float, default=24.0)
    p.add_argument('--png-3d-azim', type=float, default=-58.0)
    p.add_argument('--png-3d-dpi', type=int, default=160)
    p.add_argument(
        '--png-3d-z-exag',
        type=float,
        default=None,
        help='Z scale in 3D PNG box (omit for auto when z varies)',
    )
    p.add_argument(
        '--html-3d',
        action='store_true',
        help='also write intercept_heatmap_prob_latest_3d.html (interactive Plotly)',
    )
    p.add_argument('--html-3d-path', type=Path, default=None, help='override path for 3D HTML')
    p.add_argument(
        '--plotly-cdn-url',
        type=str,
        default=None,
        help='override Plotly.js script URL for HTML export',
    )
    args = p.parse_args()

    rng = random.Random(args.seed)
    cx, cy = args.dome_cx, args.dome_cy
    r_ref = float(args.threat_z_r_ref_m)
    if r_ref <= 0.0:
        r_ref = float(args.r_outer)
    v_smooth = (args.vx_smooth, args.vy_smooth, args.vz_smooth)
    offsets = _build_dome_disk_xy_offsets(args.r_outer, args.grid_step)
    lbl: list[tuple[float, float, float, float]] = []
    t0 = time.monotonic()
    for gx, gy in offsets:
        tx, ty = cx + gx, cy + gy
        tz = intercept_heatmap_prob_threat_altitude_m(
            args.dome_cx,
            args.dome_cy,
            args.dome_cz,
            tx,
            ty,
            mode=args.threat_z_mode,
            z_offset_m=args.z_offset,
            z_inner_m=args.threat_z_inner_m,
            z_outer_m=args.threat_z_outer_m,
            r_ref_m=r_ref,
        )
        v_tx, v_ty, v_tz = intercept_heatmap_prob_cell_velocity(
            args.dome_cx,
            args.dome_cy,
            args.dome_cz,
            tx,
            ty,
            tz,
            use_cell_los=True,
            los_spd_m_s=args.los_spd,
            v_tx_param=args.vx_param,
            v_ty_param=args.vy_param,
            v_tz_param=args.vz_param,
            v_smooth=v_smooth,
        )
        prob = estimate_hit_probability(
            tx,
            ty,
            tz,
            v_tx,
            v_ty,
            v_tz,
            args.ix,
            args.iy,
            args.iz,
            v_i_max=args.v_i_max,
            t_min=args.t_min,
            t_max=args.t_max,
            hit_thresh_m=args.hit_thresh,
            n_samples=args.mc_n,
            pos_sigma_m=args.pos_sigma,
            vel_sigma_m_s=args.vel_sigma,
            interceptor_pos_sigma_m=args.inter_sigma,
            delay_mean_s=args.delay_mean,
            delay_jitter_s=args.delay_jitter,
            rng=rng,
            use_kinematic_rollout=True,
            rollout_dt=0.05,
            rollout_max_turn_rate_rad_s=1.5,
            rollout_max_accel_m_s2=3.0,
        )
        lbl.append((tx, ty, tz, prob))
    elapsed = time.monotonic() - t0
    args.out_dir.mkdir(parents=True, exist_ok=True)
    export_intercept_heatmap_prob_to_disk(
        args.out_dir,
        lbl,
        frame_id='map',
        ref_iid='offline',
        mc_n=args.mc_n,
        heatmap_model='kinematic_rollout_offline',
        stamp_wall=time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()),
        stamp_files=bool(args.stamp_svg),
    )
    p0 = None
    for i, (gx, gy) in enumerate(offsets):
        if abs(gx) < 1e-3 and abs(gy) < 1e-3:
            p0 = lbl[i][3]
            break
    print(f'wrote {args.out_dir / "intercept_heatmap_prob_latest.csv"} ({len(lbl)} cells, {elapsed:.1f}s)')
    if args.png_3d:
        _png = args.png_3d_path
        if _png is None:
            _png = args.out_dir / 'intercept_heatmap_prob_latest_3d.png'
        stamp_wall = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        write_intercept_heatmap_prob_3d_png(
            lbl,
            _png,
            title='P(hit | noise model) — 3D heatmap',
            subtitle=f'kinematic_rollout_offline  MC n={args.mc_n}  UTC={stamp_wall}',
            dpi=int(args.png_3d_dpi),
            elev=float(args.png_3d_elev),
            azim=float(args.png_3d_azim),
            z_exaggeration=args.png_3d_z_exag,
        )
        print(f'wrote {_png}')
    if args.html_3d:
        _html = args.html_3d_path
        if _html is None:
            _html = args.out_dir / 'intercept_heatmap_prob_latest_3d.html'
        write_intercept_heatmap_prob_3d_html(
            lbl,
            _html,
            title='P(hit | noise model) — 3D heatmap (interactive)',
            subtitle=f'kinematic_rollout_offline  MC n={args.mc_n}  UTC={stamp_wall}',
            z_exaggeration=args.png_3d_z_exag,
            plotly_cdn_url=args.plotly_cdn_url,
        )
        print(f'wrote {_html} (open in browser to rotate)')
    if p0 is not None:
        print(f'center cell P(hit) ≈ {p0:.4f}')
    probs = [x[3] for x in lbl]
    print(f'min {min(probs):.4f}  max {max(probs):.4f}  mean {sum(probs)/len(probs):.4f}')


if __name__ == '__main__':
    main()
