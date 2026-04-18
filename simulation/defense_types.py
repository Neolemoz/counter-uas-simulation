"""Explicit types and thin CV-intercept wrappers for the offline defense pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from core.intercept import compute_intercept, solve_intercept_time

ManeuverAxis = Literal["y", "z"]
GuidanceKind = Literal["naive", "predictive", "pn"]


@dataclass(frozen=True)
class TargetParams:
    """Maneuvering target parameters (matches ``realtime_sim.target_position_velocity`` inputs)."""

    p0: np.ndarray
    v_lin: np.ndarray
    A: float
    omega: float
    axis: ManeuverAxis


@dataclass(frozen=True)
class InterceptorSpec:
    """One interceptor launch configuration (guidance label for bookkeeping; selection not applied yet)."""

    id: str
    position0: np.ndarray
    speed: float
    guidance: GuidanceKind


def feasibility_cv(
    p_t: np.ndarray,
    v_t: np.ndarray,
    p_i: np.ndarray,
    s_i: float,
) -> bool:
    """Constant-velocity intercept exists (positive time, closed-form geometry)."""
    return compute_intercept(p_t, v_t, p_i, s_i) is not None


def time_to_intercept_cv(
    p_t: np.ndarray,
    v_t: np.ndarray,
    p_i: np.ndarray,
    s_i: float,
) -> float | None:
    """Positive CV intercept time, or ``None`` if infeasible."""
    return solve_intercept_time(p_t, v_t, p_i, s_i)
