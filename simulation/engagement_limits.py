"""Shared engagement limit tuple for RT sandbox and offline simulation (Step 4 parity)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_YAML = _REPO_ROOT / 'src/rt_sandbox_gz/config/rt_engagement_limits.yaml'


@dataclass(frozen=True)
class EngagementLimits:
    max_speed_mps: float
    max_accel_mps2: float
    max_turn_rate_rad_s: float
    max_climb_mps: float
    drag_decel_per_mps: float
    wind_x_mps: float
    wind_y_mps: float
    wind_z_mps: float
    max_turn_rate_deg_s: float | None = None

    @property
    def turn_rate_deg_s(self) -> float:
        if self.max_turn_rate_deg_s is not None:
            return float(self.max_turn_rate_deg_s)
        return math.degrees(self.max_turn_rate_rad_s)

    def as_dict(self) -> dict[str, float]:
        return {
            'max_speed_mps': self.max_speed_mps,
            'max_accel_mps2': self.max_accel_mps2,
            'max_turn_rate_rad_s': self.max_turn_rate_rad_s,
            'max_turn_rate_deg_s': self.turn_rate_deg_s,
            'max_climb_mps': self.max_climb_mps,
            'drag_decel_per_mps': self.drag_decel_per_mps,
            'wind_x_mps': self.wind_x_mps,
            'wind_y_mps': self.wind_y_mps,
            'wind_z_mps': self.wind_z_mps,
        }

    def kinematic_limits_kwargs(self) -> dict[str, float]:
        return {
            'max_speed_mps': self.max_speed_mps,
            'max_accel_mps2': self.max_accel_mps2,
            'max_turn_rate_rad_s': self.max_turn_rate_rad_s,
            'max_climb_mps': self.max_climb_mps,
        }

    def aero_kwargs(self) -> dict[str, float]:
        return {
            'drag_decel_per_mps': self.drag_decel_per_mps,
            'wind_x_mps': self.wind_x_mps,
            'wind_y_mps': self.wind_y_mps,
            'wind_z_mps': self.wind_z_mps,
        }


def _parse_engagement_section(data: dict[str, Any]) -> EngagementLimits:
    section = data.get('engagement_limits') if isinstance(data, dict) else None
    if not isinstance(section, dict):
        raise ValueError('engagement_limits section missing in YAML')
    deg = section.get('max_turn_rate_deg_s')
    rad = section.get('max_turn_rate_rad_s')
    if rad is None and deg is not None:
        rad = math.radians(float(deg))
    if rad is None:
        rad = math.radians(16.0)
    return EngagementLimits(
        max_speed_mps=float(section.get('max_speed_mps', 25.0)),
        max_accel_mps2=float(section.get('max_accel_mps2', 30.0)),
        max_turn_rate_rad_s=float(rad),
        max_climb_mps=float(section.get('max_climb_mps', 8.0)),
        drag_decel_per_mps=float(section.get('drag_decel_per_mps', 0.0)),
        wind_x_mps=float(section.get('wind_x_mps', 0.0)),
        wind_y_mps=float(section.get('wind_y_mps', 0.0)),
        wind_z_mps=float(section.get('wind_z_mps', 0.0)),
        max_turn_rate_deg_s=float(deg) if deg is not None else None,
    )


def load_engagement_limits(path: Path | str | None = None) -> EngagementLimits:
    """Load canonical limits from ``rt_engagement_limits.yaml``."""
    yaml_path = Path(path) if path is not None else _DEFAULT_YAML
    raw = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    return _parse_engagement_section(raw or {})


def default_engagement_limits() -> EngagementLimits:
    """Cached-friendly accessor for tests and offline sim."""
    return load_engagement_limits()
