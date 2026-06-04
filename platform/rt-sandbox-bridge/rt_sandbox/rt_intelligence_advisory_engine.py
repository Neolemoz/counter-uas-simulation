"""Pure RT intelligence advisory derivation.

The engine is intentionally side-effect free: it consumes precomputed geometry
and feasibility inputs and returns rt_intelligence_advisory_v1 dictionaries.
"""

from __future__ import annotations

import math
from typing import Any


SCHEMA = "rt_intelligence_advisory_v1"
INPUT_SCHEMA = "rt_intelligence_advisory_input_v1"
AUTHORITY = "intelligence_advisory_explanatory"
GOVERNANCE_BANNER = (
    "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, "
    "or weapon authority"
)

WEIGHT_DISTANCE = 30.0
WEIGHT_TTI = 30.0
WEIGHT_DESCENT = 15.0
WEIGHT_CRITICAL = 25.0

DEFAULT_CONSTANTS = {
    "warning_ring_radius_m": 5000.0,
    "critical_radius_m": 14.0,
    "tti_reference_s": 120.0,
    "descent_reference_mps": 5.0,
    "descending_fast_threshold_mps": 0.35,
    "tti_tie_tolerance_s": 0.001,
}


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return min(max(value, lo), hi)


def _constants(data: dict[str, Any]) -> dict[str, float]:
    raw = data.get("constants") if isinstance(data.get("constants"), dict) else {}
    out: dict[str, float] = {}
    for key, default in DEFAULT_CONSTANTS.items():
        parsed = _finite_number(raw.get(key))
        out[key] = parsed if parsed is not None and parsed > 0.0 else default
    return out


def _attacker_id(attacker: dict[str, Any]) -> str:
    return str(attacker.get("attacker_id") or "")


def _defender_id(defender: dict[str, Any]) -> str:
    return str(defender.get("defender_id") or "")


def _tti_rows_for_attacker(data: dict[str, Any], attacker_id: str) -> list[dict[str, Any]]:
    rows = data.get("feasible_tti")
    if not isinstance(rows, list):
        return []
    out: list[dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        if str(row.get("attacker_id") or "") != attacker_id:
            continue
        defender_id = str(row.get("defender_id") or "")
        if not defender_id:
            continue
        tti_s = _finite_number(row.get("tti_s"))
        feasible = bool(row.get("feasible")) and tti_s is not None
        out.append(
            {
                "defender_id": defender_id,
                "feasible": feasible,
                "tti_s": tti_s if feasible else None,
                "reason": str(row.get("reason") or ("feasible" if feasible else "infeasible")),
            }
        )
    return out


def rank_defenders(
    data: dict[str, Any],
    attacker_id: str,
    *,
    tti_tie_tolerance_s: float | None = None,
) -> list[dict[str, Any]]:
    """Rank defender rows for one attacker using feasibility, TTI, then id."""
    constants = _constants(data)
    tie_tol = (
        float(tti_tie_tolerance_s)
        if tti_tie_tolerance_s is not None and tti_tie_tolerance_s >= 0.0
        else constants["tti_tie_tolerance_s"]
    )
    tti_by_id = {_defender_id(row): row for row in _tti_rows_for_attacker(data, attacker_id)}
    defenders = data.get("defender_candidates") if isinstance(data.get("defender_candidates"), list) else []

    rows: list[dict[str, Any]] = []
    for defender in defenders:
        if not isinstance(defender, dict):
            continue
        defender_id = _defender_id(defender)
        if not defender_id:
            continue
        tti_row = tti_by_id.get(defender_id)
        feasible = bool(tti_row and tti_row.get("feasible"))
        tti_s = _finite_number(tti_row.get("tti_s")) if tti_row else None
        reason = str(tti_row.get("reason") if tti_row else "no_solution")
        rows.append(
            {
                "defender_id": defender_id,
                "feasible": feasible,
                "tti_s": tti_s if feasible else None,
                "reason": reason,
            }
        )

    def sort_key(row: dict[str, Any]) -> tuple[int, float, str]:
        feasible_rank = 0 if row["feasible"] else 1
        tti = row["tti_s"] if row["tti_s"] is not None else float("inf")
        return (feasible_rank, float(tti), str(row["defender_id"]))

    rows.sort(key=sort_key)
    feasible_ttis = [
        float(row["tti_s"]) for row in rows if row["feasible"] and row["tti_s"] is not None
    ]
    best_tti = min(feasible_ttis) if feasible_ttis else None
    feasible_count = len(feasible_ttis)
    tie_at_best = (
        best_tti is not None
        and sum(abs(float(row["tti_s"]) - best_tti) <= tie_tol for row in rows if row["feasible"]) > 1
    )

    ranked: list[dict[str, Any]] = []
    for index, row in enumerate(rows, start=1):
        reason_codes: list[str] = []
        if row["feasible"] and row["tti_s"] is not None and best_tti is not None:
            if abs(float(row["tti_s"]) - best_tti) <= tie_tol:
                reason_codes.append("shortest_tti")
                if feasible_count == 1:
                    reason_codes.append("only_feasible")
                if tie_at_best:
                    reason_codes.append("tti_tie_break")
        ranked.append(
            {
                "defender_id": row["defender_id"],
                "rank": index,
                "feasible": row["feasible"],
                "tti_s": row["tti_s"],
                "reason_codes": reason_codes,
            }
        )
    return ranked


def _best_feasible_tti(ranked_defenders: list[dict[str, Any]]) -> float | None:
    for row in ranked_defenders:
        if row.get("feasible") and row.get("tti_s") is not None:
            return float(row["tti_s"])
    return None


def compute_threat_score(
    attacker: dict[str, Any],
    best_feasible_tti_s: float | None,
    constants: dict[str, float],
) -> tuple[float | None, dict[str, Any]]:
    """Return bounded score and rt_intelligence_advisory_v1 components."""
    distance = _finite_number(attacker.get("distance_to_protected_center_m"))
    descent = _finite_number(attacker.get("descent_rate_mps"))
    warning_radius = constants["warning_ring_radius_m"]
    critical_radius = constants["critical_radius_m"]
    tti_reference = constants["tti_reference_s"]
    descent_reference = constants["descent_reference_mps"]

    if distance is None:
        distance_norm = None
        critical_active = False
        critical_norm = 0.0
        score = None
    else:
        distance_norm = _clamp(1.0 - (distance / warning_radius))
        critical_active = distance <= critical_radius
        critical_norm = 1.0 if critical_active else 0.0
        tti_norm = (
            _clamp(1.0 - (best_feasible_tti_s / tti_reference))
            if best_feasible_tti_s is not None
            else 0.0
        )
        descent_norm = (
            _clamp(abs(min(descent, 0.0)) / descent_reference)
            if descent is not None
            else 0.0
        )
        score = _clamp(
            (
                WEIGHT_DISTANCE * distance_norm
                + WEIGHT_TTI * tti_norm
                + WEIGHT_DESCENT * descent_norm
                + WEIGHT_CRITICAL * critical_norm
            )
            / 100.0,
            0.0,
            1.0,
        ) * 100.0

    tti_norm = (
        _clamp(1.0 - (best_feasible_tti_s / tti_reference))
        if best_feasible_tti_s is not None
        else 0.0
    )
    descent_norm = (
        _clamp(abs(min(descent, 0.0)) / descent_reference)
        if descent is not None
        else None
    )
    components = {
        "distance_to_protected_center": {
            "value_m": distance,
            "normalized": distance_norm,
            "weight": WEIGHT_DISTANCE,
        },
        "best_feasible_tti": {
            "value_s": best_feasible_tti_s,
            "normalized": tti_norm,
            "weight": WEIGHT_TTI,
        },
        "descent_factor": {
            "value_mps": descent,
            "normalized": descent_norm,
            "weight": WEIGHT_DESCENT,
        },
        "critical_zone_factor": {
            "active": bool(distance is not None and distance <= critical_radius),
            "normalized": 1.0 if distance is not None and distance <= critical_radius else 0.0,
            "weight": WEIGHT_CRITICAL,
        },
    }
    return (round(score, 6) if score is not None else None), components


def _attacker_sort_key(row: dict[str, Any]) -> tuple[float, float, float, int, str]:
    attacker = row["attacker"]
    score = row["score"]
    distance = _finite_number(attacker.get("distance_to_protected_center_m"))
    best_tti = row["best_tti"]
    descent = _finite_number(attacker.get("descent_rate_mps"))
    return (
        -(score if score is not None else -1.0),
        distance if distance is not None else float("inf"),
        best_tti if best_tti is not None else float("inf"),
        0 if descent is not None and descent < 0.0 else 1,
        _attacker_id(attacker),
    )


def rank_attackers(data: dict[str, Any]) -> dict[str, int | None]:
    """Rank attackers by advisory threat score and deterministic tie breaks."""
    constants = _constants(data)
    rows: list[dict[str, Any]] = []
    attackers = data.get("attackers") if isinstance(data.get("attackers"), list) else []
    for attacker in attackers:
        if not isinstance(attacker, dict):
            continue
        attacker_id = _attacker_id(attacker)
        if not attacker_id:
            continue
        ranked_defenders = rank_defenders(data, attacker_id)
        best_tti = _best_feasible_tti(ranked_defenders)
        score, _components = compute_threat_score(attacker, best_tti, constants)
        rows.append(
            {
                "attacker": attacker,
                "score": score,
                "best_tti": best_tti,
            }
        )
    rows.sort(key=_attacker_sort_key)
    ranks: dict[str, int | None] = {}
    for index, row in enumerate(rows, start=1):
        attacker_id = _attacker_id(row["attacker"])
        ranks[attacker_id] = index if row["score"] is not None else None
    return ranks


def derive_reason_codes(
    attacker: dict[str, Any],
    ranked_defenders: list[dict[str, Any]],
    constants: dict[str, float],
) -> list[str]:
    codes: list[str] = []
    distance = _finite_number(attacker.get("distance_to_protected_center_m"))
    descent = _finite_number(attacker.get("descent_rate_mps"))
    feasible = [row for row in ranked_defenders if row.get("feasible")]

    if distance is None:
        codes.append("insufficient_inputs")
    else:
        if distance <= constants["critical_radius_m"]:
            codes.append("critical_target")
        if distance <= constants["warning_ring_radius_m"]:
            codes.append("inside_warning_ring")

    if descent is not None and descent <= -constants["descending_fast_threshold_mps"]:
        codes.append("descending_fast")

    if not feasible:
        codes.append("no_solution")
    else:
        codes.append("feasible_pair_available")
        codes.append("shortest_tti")
        if len(feasible) == 1:
            codes.append("only_feasible")
        if ranked_defenders and "tti_tie_break" in ranked_defenders[0].get("reason_codes", []):
            codes.append("tti_tie_break")

    return codes


def derive_heuristic_confidence(
    attacker: dict[str, Any],
    ranked_defenders: list[dict[str, Any]],
) -> dict[str, Any]:
    basis: list[str] = []
    distance = _finite_number(attacker.get("distance_to_protected_center_m"))
    descent = _finite_number(attacker.get("descent_rate_mps"))
    attacker_id = _attacker_id(attacker)
    feasible = [row for row in ranked_defenders if row.get("feasible")]
    best_tti = _best_feasible_tti(ranked_defenders)

    if attacker_id:
        basis.append("complete_attacker_identity")
    if distance is not None:
        basis.append("distance_available")
    if descent is not None:
        basis.append("descent_available")
    if ranked_defenders:
        basis.append("defender_candidates_available")
    if feasible and best_tti is not None:
        basis.append("feasible_tti_available")
        basis.append("deterministic_defender_ranking")

    if not attacker_id or distance is None or not feasible or best_tti is None:
        return {"score": 0.35, "level": "low", "basis": basis}
    if descent is None:
        return {"score": 0.6, "level": "medium", "basis": basis}
    return {"score": 0.85, "level": "high", "basis": basis}


def _recommendation(ranked_defenders: list[dict[str, Any]]) -> dict[str, Any]:
    for row in ranked_defenders:
        if row.get("feasible"):
            return {
                "defender_id": row["defender_id"],
                "feasibility": {"feasible": True, "reason": "feasible"},
                "tti_s": row["tti_s"],
            }
    return {
        "defender_id": None,
        "feasibility": {"feasible": False, "reason": "no_solution"},
        "tti_s": None,
    }


def _explanation(codes: list[str], attacker_id: str, recommended: dict[str, Any]) -> str:
    defender_id = recommended.get("defender_id")
    if "no_solution" in codes:
        return f"No feasible defender solution is available for attacker {attacker_id}."
    if defender_id:
        details: list[str] = []
        if "critical_target" in codes:
            details.append("inside the critical zone")
        if "descending_fast" in codes:
            details.append("descending fast")
        if details:
            return (
                f"Attacker {attacker_id} is "
                + ", ".join(details)
                + f"; defender {defender_id} has the shortest feasible TTI."
            )
        return (
            f"Defender {defender_id} has the shortest feasible TTI for "
            f"attacker {attacker_id}."
        )
    return f"Insufficient advisory inputs for attacker {attacker_id}."


def build_intelligence_advisories(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Build rt_intelligence_advisory_v1 payloads for all attackers."""
    constants = _constants(data)
    ranks = rank_attackers(data)
    advisory_utc = str(data.get("advisory_utc") or "")
    attackers = data.get("attackers") if isinstance(data.get("attackers"), list) else []
    advisories: list[dict[str, Any]] = []
    for attacker in attackers:
        if not isinstance(attacker, dict):
            continue
        attacker_id = _attacker_id(attacker)
        if not attacker_id:
            continue
        ranked_defenders = rank_defenders(data, attacker_id)
        best_tti = _best_feasible_tti(ranked_defenders)
        score, components = compute_threat_score(attacker, best_tti, constants)
        recommended = _recommendation(ranked_defenders)
        codes = derive_reason_codes(attacker, ranked_defenders, constants)
        confidence = derive_heuristic_confidence(attacker, ranked_defenders)
        advisories.append(
            {
                "schema": SCHEMA,
                "identity": {
                    "advisory_id": f"adv-{attacker_id}",
                    "attacker_id": attacker_id,
                    "advisory_utc": advisory_utc,
                },
                "threat_evaluation": {
                    "threat_score": score,
                    "threat_rank": ranks.get(attacker_id),
                    "threat_components": components,
                },
                "recommended_defender": recommended,
                "defender_ranking": {"ranked_defenders": ranked_defenders},
                "reasoning": {
                    "reason_codes": codes,
                    "explanation": _explanation(codes, attacker_id, recommended),
                },
                "confidence": {"heuristic_confidence": confidence},
                "governance": {
                    "authority": AUTHORITY,
                    "governance_banner": GOVERNANCE_BANNER,
                },
            }
        )
    advisories.sort(
        key=lambda item: (
            item["threat_evaluation"]["threat_rank"]
            if item["threat_evaluation"]["threat_rank"] is not None
            else 10**9,
            item["identity"]["attacker_id"],
        )
    )
    return advisories


def build_intelligence_advisory(data: dict[str, Any], attacker_id: str) -> dict[str, Any]:
    """Build one rt_intelligence_advisory_v1 payload for attacker_id."""
    for advisory in build_intelligence_advisories(data):
        if advisory["identity"]["attacker_id"] == attacker_id:
            return advisory
    raise KeyError(f"attacker_id not found: {attacker_id}")
