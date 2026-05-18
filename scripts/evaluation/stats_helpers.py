#!/usr/bin/env python3
"""Deterministic statistical helpers for Layer C validation artifacts."""

from __future__ import annotations

import math
import random
from collections.abc import Callable, Sequence
from statistics import NormalDist


DEFAULT_CONFIDENCE = 0.95
DEFAULT_BOOTSTRAP_RESAMPLES = 2000


def cohort_tier(n: int) -> str:
    """Return the evidentiary tier for a cohort size."""
    if n >= 40:
        return "validation"
    if n >= 10:
        return "pilot"
    return "smoke"


def percentile(values: Sequence[float], p: float) -> float:
    """Linear-interpolated percentile matching the existing MC harness."""
    xs = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not xs:
        return float("nan")
    if len(xs) == 1:
        return xs[0]
    idx = (len(xs) - 1) * float(p) / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return xs[lo]
    return xs[lo] + (xs[hi] - xs[lo]) * (idx - lo)


def _z(confidence: float) -> float:
    alpha = 1.0 - float(confidence)
    return NormalDist().inv_cdf(1.0 - alpha / 2.0)


def wilson_ci(successes: int, n: int, *, confidence: float = DEFAULT_CONFIDENCE) -> dict[str, float | int | str]:
    """Wilson score interval for a binomial proportion."""
    n_i = int(n)
    k_i = int(successes)
    if n_i <= 0:
        return {
            "method": "wilson",
            "confidence": float(confidence),
            "n": n_i,
            "successes": k_i,
            "estimate": float("nan"),
            "lower": float("nan"),
            "upper": float("nan"),
        }
    if k_i < 0 or k_i > n_i:
        raise ValueError("successes must be in [0, n]")
    z = _z(confidence)
    phat = k_i / n_i
    denom = 1.0 + z * z / n_i
    centre = phat + z * z / (2.0 * n_i)
    radius = z * math.sqrt((phat * (1.0 - phat) + z * z / (4.0 * n_i)) / n_i)
    return {
        "method": "wilson",
        "confidence": float(confidence),
        "n": n_i,
        "successes": k_i,
        "estimate": phat,
        "lower": max(0.0, (centre - radius) / denom),
        "upper": min(1.0, (centre + radius) / denom),
    }


def _bootstrap_samples(
    values: Sequence[float],
    *,
    n_resamples: int,
    seed: int,
    statistic: Callable[[list[float]], float],
) -> list[float]:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return []
    rng = random.Random(int(seed))
    n = len(clean)
    out: list[float] = []
    for _ in range(int(n_resamples)):
        sample = [clean[rng.randrange(n)] for _ in range(n)]
        stat = float(statistic(sample))
        if math.isfinite(stat):
            out.append(stat)
    return out


def bootstrap_ci(
    values: Sequence[float],
    statistic: Callable[[list[float]], float],
    *,
    estimate: float | None = None,
    confidence: float = DEFAULT_CONFIDENCE,
    n_resamples: int = DEFAULT_BOOTSTRAP_RESAMPLES,
    seed: int = 1,
    method_name: str = "bootstrap",
) -> dict[str, float | int | str]:
    """Percentile bootstrap confidence interval for one-sample statistics."""
    clean = [float(v) for v in values if math.isfinite(float(v))]
    est = float(statistic(clean)) if estimate is None and clean else float("nan")
    if estimate is not None:
        est = float(estimate)
    samples = _bootstrap_samples(clean, n_resamples=n_resamples, seed=seed, statistic=statistic)
    alpha = 1.0 - float(confidence)
    return {
        "method": method_name,
        "confidence": float(confidence),
        "n": len(clean),
        "resamples": int(n_resamples),
        "seed": int(seed),
        "estimate": est,
        "lower": percentile(samples, 100.0 * alpha / 2.0),
        "upper": percentile(samples, 100.0 * (1.0 - alpha / 2.0)),
    }


def bootstrap_quantile_ci(
    values: Sequence[float],
    q: float,
    *,
    confidence: float = DEFAULT_CONFIDENCE,
    n_resamples: int = DEFAULT_BOOTSTRAP_RESAMPLES,
    seed: int = 1,
) -> dict[str, float | int | str]:
    """Percentile bootstrap CI for a percentile/quantile statistic."""
    q_f = float(q)
    return bootstrap_ci(
        values,
        lambda sample: percentile(sample, q_f),
        estimate=percentile(values, q_f),
        confidence=confidence,
        n_resamples=n_resamples,
        seed=seed,
        method_name=f"bootstrap_p{q_f:g}",
    )


def _binomial_cdf(k: int, n: int, p: float = 0.5) -> float:
    if k < 0:
        return 0.0
    if k >= n:
        return 1.0
    return sum(math.comb(n, i) * (p**i) * ((1.0 - p) ** (n - i)) for i in range(k + 1))


def exact_paired_binary_pvalue(baseline_only: int, candidate_only: int) -> float:
    """Two-sided exact McNemar/binomial-sign p-value for discordant pairs."""
    b = int(baseline_only)
    c = int(candidate_only)
    discordant = b + c
    if discordant <= 0:
        return 1.0
    return min(1.0, 2.0 * _binomial_cdf(min(b, c), discordant, 0.5))


def paired_binary_summary(
    baseline_success: Sequence[bool],
    candidate_success: Sequence[bool],
    *,
    confidence: float = DEFAULT_CONFIDENCE,
    n_resamples: int = DEFAULT_BOOTSTRAP_RESAMPLES,
    seed: int = 1,
) -> dict[str, object]:
    """Summarize paired binary outcomes, including paired delta uncertainty."""
    if len(baseline_success) != len(candidate_success):
        raise ValueError("paired binary sequences must have equal length")
    pairs = [(bool(b), bool(c)) for b, c in zip(baseline_success, candidate_success)]
    n = len(pairs)
    base_only = sum(1 for b, c in pairs if b and not c)
    cand_only = sum(1 for b, c in pairs if c and not b)
    both_success = sum(1 for b, c in pairs if b and c)
    both_fail = sum(1 for b, c in pairs if not b and not c)
    base_rate = sum(1 for b, _ in pairs if b) / n if n else float("nan")
    cand_rate = sum(1 for _, c in pairs if c) / n if n else float("nan")
    deltas = [(1.0 if c else 0.0) - (1.0 if b else 0.0) for b, c in pairs]
    delta_ci = bootstrap_ci(
        deltas,
        lambda sample: sum(sample) / len(sample) if sample else float("nan"),
        estimate=(cand_rate - base_rate) if n else float("nan"),
        confidence=confidence,
        n_resamples=n_resamples,
        seed=seed,
        method_name="paired_bootstrap_mean_delta",
    )
    return {
        "n_pairs": n,
        "baseline_success_rate": base_rate,
        "candidate_success_rate": cand_rate,
        "paired_success_delta": (cand_rate - base_rate) if n else float("nan"),
        "paired_success_delta_ci95": delta_ci,
        "both_success": both_success,
        "both_fail": both_fail,
        "baseline_only_success": base_only,
        "candidate_only_success": cand_only,
        "discordant_pairs": base_only + cand_only,
        "exact_mcnemar_p_two_sided": exact_paired_binary_pvalue(base_only, cand_only),
    }


def paired_bootstrap_delta_ci(
    baseline_values: Sequence[float],
    candidate_values: Sequence[float],
    statistic: Callable[[list[float]], float],
    *,
    statistic_name: str,
    confidence: float = DEFAULT_CONFIDENCE,
    n_resamples: int = DEFAULT_BOOTSTRAP_RESAMPLES,
    seed: int = 1,
) -> dict[str, float | int | str]:
    """Bootstrap CI for candidate-minus-baseline paired statistic deltas."""
    if len(baseline_values) != len(candidate_values):
        raise ValueError("paired value sequences must have equal length")
    pairs = [
        (float(b), float(c))
        for b, c in zip(baseline_values, candidate_values)
        if math.isfinite(float(b)) and math.isfinite(float(c))
    ]
    if not pairs:
        return {
            "method": f"paired_bootstrap_{statistic_name}_delta",
            "confidence": float(confidence),
            "n": 0,
            "resamples": int(n_resamples),
            "seed": int(seed),
            "estimate": float("nan"),
            "lower": float("nan"),
            "upper": float("nan"),
        }
    b_vals = [b for b, _ in pairs]
    c_vals = [c for _, c in pairs]
    estimate = float(statistic(c_vals) - statistic(b_vals))
    rng = random.Random(int(seed))
    samples: list[float] = []
    n = len(pairs)
    for _ in range(int(n_resamples)):
        sample = [pairs[rng.randrange(n)] for _ in range(n)]
        sb = [b for b, _ in sample]
        sc = [c for _, c in sample]
        stat = float(statistic(sc) - statistic(sb))
        if math.isfinite(stat):
            samples.append(stat)
    alpha = 1.0 - float(confidence)
    return {
        "method": f"paired_bootstrap_{statistic_name}_delta",
        "confidence": float(confidence),
        "n": n,
        "resamples": int(n_resamples),
        "seed": int(seed),
        "estimate": estimate,
        "lower": percentile(samples, 100.0 * alpha / 2.0),
        "upper": percentile(samples, 100.0 * (1.0 - alpha / 2.0)),
    }


def summarize_binary(successes: int, n: int, *, confidence: float = DEFAULT_CONFIDENCE) -> dict[str, object]:
    """Return point estimate plus Wilson CI for a binary metric."""
    ci = wilson_ci(successes, n, confidence=confidence)
    return {
        "n": int(n),
        "successes": int(successes),
        "rate": ci["estimate"],
        "ci95": ci,
    }
