"""Load shared engagement limits YAML for RT sandbox (Step 4 parity)."""

from __future__ import annotations

import sys
from pathlib import Path

_SIM = Path(__file__).resolve().parents[3] / 'simulation'
if str(_SIM) not in sys.path:
    sys.path.insert(0, str(_SIM))

from engagement_limits import (  # noqa: E402
    EngagementLimits,
    default_engagement_limits,
    load_engagement_limits,
)

from rt_sandbox_gz.kinematic_plant import AeroEnvironment, KinematicLimits  # noqa: E402


def default_kinematic_limits() -> KinematicLimits:
    return KinematicLimits(**default_engagement_limits().kinematic_limits_kwargs())


def default_aero_environment() -> AeroEnvironment:
    return AeroEnvironment(**default_engagement_limits().aero_kwargs())


__all__ = [
    'AeroEnvironment',
    'EngagementLimits',
    'KinematicLimits',
    'default_aero_environment',
    'default_engagement_limits',
    'default_kinematic_limits',
    'load_engagement_limits',
]
