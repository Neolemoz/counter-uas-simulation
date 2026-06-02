"""Load sensor realism overlay YAML packs (docs/scenarios/realism)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
_REALISM_DIR = _REPO_ROOT / 'docs' / 'scenarios' / 'realism'

OVERLAY_RANGE_DEPENDENT = 'range_dependent_sensing'
OVERLAY_SENSOR_DECIMATION = 'sensor_decimation_latency'
COMBINED_OVERLAY_IDS = (OVERLAY_RANGE_DEPENDENT, OVERLAY_SENSOR_DECIMATION)


def overlay_yaml_path(overlay_id: str) -> Path:
    path = _REALISM_DIR / f'{overlay_id}.yaml'
    if not path.is_file():
        raise FileNotFoundError(f'missing sensor realism overlay: {path}')
    return path


def load_overlay_document(overlay_id: str) -> dict[str, Any]:
    with overlay_yaml_path(overlay_id).open(encoding='utf-8') as handle:
        doc = yaml.safe_load(handle)
    if not isinstance(doc, dict):
        raise ValueError(f'overlay {overlay_id!r} must be a mapping')
    return doc


def _parse_override_line(line: str) -> tuple[str, Any]:
    key, _, raw = line.partition(':')
    key = key.strip()
    if not key:
        raise ValueError(f'invalid override line: {line!r}')
    value = yaml.safe_load(raw.strip())
    return key, value


def _normalize_override_lines(lines: list[Any]) -> list[str]:
    """YAML list entries may be strings or single-key mappings."""
    normalized: list[str] = []
    for item in lines:
        if isinstance(item, str):
            stripped = item.strip()
            if stripped:
                normalized.append(stripped)
        elif isinstance(item, dict):
            for key, value in item.items():
                normalized.append(f'{key}: {value}')
    return normalized


def nested_from_dotted_overrides(lines: list[Any]) -> dict[str, Any]:
    """Turn ``radar.publish_every_n: 2`` lines into nested dicts for ROS parameters."""
    out: dict[str, Any] = {}
    for line in _normalize_override_lines(lines):
        if not str(line).strip():
            continue
        key, value = _parse_override_line(str(line))
        cursor = out
        parts = key.split('.')
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return out


def load_combined_sensor_overrides(
    overlay_ids: tuple[str, ...] = COMBINED_OVERLAY_IDS,
) -> dict[str, dict[str, Any]]:
    """Merge ``parameter_overrides`` sections from multiple overlay docs by node name."""
    merged: dict[str, dict[str, Any]] = {}
    for overlay_id in overlay_ids:
        doc = load_overlay_document(overlay_id)
        overrides = doc.get('parameter_overrides') or {}
        if not isinstance(overrides, dict):
            raise ValueError(f'overlay {overlay_id!r} parameter_overrides must be a mapping')
        for node_name, lines in overrides.items():
            if not isinstance(lines, list):
                raise ValueError(f'overlay {overlay_id!r} {node_name} overrides must be a list')
            node_params = nested_from_dotted_overrides(lines)
            base = merged.setdefault(str(node_name), {})
            for key, value in node_params.items():
                if isinstance(value, dict) and isinstance(base.get(key), dict):
                    base[key] = {**base[key], **value}
                else:
                    base[key] = value
    return merged


def radar_camera_overlay_sections() -> tuple[dict[str, Any], dict[str, Any]]:
    merged = load_combined_sensor_overrides()
    return merged.get('radar_sim_node', {}), merged.get('camera_sim_node', {})
