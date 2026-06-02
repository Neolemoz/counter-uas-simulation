"""Pure helpers for optional range-dependent PD and measurement noise (additive realism)."""

from __future__ import annotations


def effective_detection_probability(
    *,
    base_p: float,
    distance_m: float,
    max_range_m: float,
    decay_with_range: float,
    min_detection_probability: float,
) -> float:
    """Linear PD falloff from ``base_p`` at range 0 toward ``min_detection_probability`` at max range.

    When ``decay_with_range`` <= 0, returns ``base_p`` unchanged (legacy flat PD).
    """
    base_p = min(1.0, max(0.0, float(base_p)))
    min_p = min(base_p, min(1.0, max(0.0, float(min_detection_probability))))
    decay = float(decay_with_range)
    if decay <= 0.0:
        return base_p
    if max_range_m <= 1e-9:
        return base_p
    frac = min(max(float(distance_m) / float(max_range_m), 0.0), 1.0)
    p = base_p * (1.0 - decay * frac)
    return max(min_p, min(1.0, p))


def effective_measurement_std(
    base_std: float,
    distance_m: float,
    max_range_m: float,
    std_scale_with_range: float,
) -> float:
    """Scale isotropic measurement sigma by ``1 + std_scale_with_range * (d / r_max)`` when scale > 0."""
    base = max(0.0, float(base_std))
    scale = float(std_scale_with_range)
    if scale <= 0.0 or max_range_m <= 1e-9:
        return base
    frac = min(max(float(distance_m) / float(max_range_m), 0.0), 1.0)
    return base * (1.0 + scale * frac)
