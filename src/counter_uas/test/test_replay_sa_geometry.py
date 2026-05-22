from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_geometry():
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'replay_sa_geometry.py'
    spec = importlib.util.spec_from_file_location('replay_sa_geometry', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _square_overlay(kind: str, overlay_id: str, x0: float, y0: float, size: float) -> dict:
    return {
        'overlay_id': overlay_id,
        'kind': kind,
        'geometry': {
            'type': 'polygon',
            'vertices_enu_m': [
                [x0, y0, 0],
                [x0 + size, y0, 0],
                [x0 + size, y0 + size, 0],
                [x0, y0 + size, 0],
            ],
        },
    }


def test_classify_point_visibility_clear() -> None:
    geo = _load_geometry()
    overlays = [_square_overlay('los_blocked', 'blk', -500, -500, 100)]
    assert geo.classify_point_visibility((0, 0, 0), overlays) == 'clear'


def test_classify_point_visibility_blocked() -> None:
    geo = _load_geometry()
    overlays = [_square_overlay('los_blocked', 'blk', -50, -50, 200)]
    assert geo.classify_point_visibility((0, 0, 0), overlays) == 'blocked'


def test_approximate_los_status_visible_vs_blocked() -> None:
    geo = _load_geometry()
    overlays = [_square_overlay('los_blocked', 'blk', -150, -150, 120)]
    origin_clear = (400.0, 400.0, 10.0)
    target_clear = (800.0, 600.0, 50.0)
    origin_blocked = (-100.0, -100.0, 10.0)
    target_blocked = (-110.0, -110.0, 50.0)
    assert geo.approximate_los_status(origin_clear, target_clear, overlays) == 'visible'
    assert geo.approximate_los_status(origin_blocked, target_blocked, overlays) == 'terrain_blocked'


def test_build_los_segments_multi_threat() -> None:
    geo = _load_geometry()
    entities = [
        {
            'entity_id': 'radar_01',
            'kind': 'radar',
            'position_enu_m': [0, 0, 12],
            'label': 'r',
        },
    ]
    tracks = [
        {
            'track_id': 'threat_uav_0',
            'role': 'threat',
            'samples': [
                {'t': 2, 'x_m': -1200, 'y_m': 50, 'z_m': 300},
                {'t': 8, 'x_m': -1000, 'y_m': 40, 'z_m': 280},
            ],
        },
        {
            'track_id': 'threat_uav_1',
            'role': 'threat',
            'samples': [
                {'t': 3, 'x_m': -1100, 'y_m': -60, 'z_m': 290},
                {'t': 9, 'x_m': -900, 'y_m': -50, 'z_m': 270},
            ],
        },
    ]
    segments = geo.build_los_segments(entities, tracks, [], max_segments=12)
    to_ids = {s['to_track_id'] for s in segments}
    assert 'threat_uav_0' in to_ids
    assert 'threat_uav_1' in to_ids
    assert len(segments) <= 12


def test_build_los_segments_respects_cap() -> None:
    geo = _load_geometry()
    entities = [
        {
            'entity_id': 'radar_01',
            'kind': 'radar',
            'position_enu_m': [0, 0, 12],
            'label': 'r',
        },
        {
            'entity_id': 'eoir_01',
            'kind': 'eoir',
            'position_enu_m': [200, 100, 15],
            'label': 'e',
        },
    ]
    tracks = [
        {
            'track_id': 'threat_uav_0',
            'role': 'threat',
            'samples': [
                {'t': 2, 'x_m': -1200, 'y_m': 50, 'z_m': 300},
                {'t': 5, 'x_m': -1100, 'y_m': 45, 'z_m': 290},
                {'t': 8, 'x_m': -1000, 'y_m': 40, 'z_m': 280},
                {'t': 10, 'x_m': -900, 'y_m': 35, 'z_m': 270},
            ],
        },
    ]
    segments = geo.build_los_segments(entities, tracks, [], max_segments=12)
    assert segments
    assert all(s.get('caveat') for s in segments)
    assert len(segments) <= 12
