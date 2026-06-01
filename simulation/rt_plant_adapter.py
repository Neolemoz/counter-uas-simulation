"""Use RT sandbox kinematic integrator from offline simulation (Step 4 parity)."""

from __future__ import annotations

import sys
from pathlib import Path

from engagement_limits import EngagementLimits, default_engagement_limits

_GZ_PKG = Path(__file__).resolve().parents[1] / 'src' / 'rt_sandbox_gz'
if str(_GZ_PKG) not in sys.path:
    sys.path.insert(0, str(_GZ_PKG))

from rt_sandbox_gz.kinematic_plant import (  # noqa: E402
    AeroEnvironment,
    KinematicLimits,
    PlantState,
    integrate_toward_pose,
)


def limits_from_engagement(el: EngagementLimits | None = None) -> KinematicLimits:
    eng = el or default_engagement_limits()
    return KinematicLimits(**eng.kinematic_limits_kwargs())


def aero_from_engagement(el: EngagementLimits | None = None) -> AeroEnvironment:
    eng = el or default_engagement_limits()
    return AeroEnvironment(**eng.aero_kwargs())


def integrate_commanded_pose(
    state: PlantState,
    commanded: dict[str, float],
    dt: float,
    el: EngagementLimits | None = None,
) -> PlantState:
    """Single RT plant step with shared engagement tuple."""
    return integrate_toward_pose(
        state,
        commanded,
        dt,
        limits_from_engagement(el),
        aero_from_engagement(el),
    )
