#!/usr/bin/env python3
"""
Parse interception_logic stdout blocks ``=== Interceptor Selection ===``.

Oracle (single-threat audit): among ``interceptor_X: feasible=True, tti=<float>`` rows,
oracle id(s) minimise tti. Compared to ``selected:`` within the same block.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

_STRIP_LAUNCH_PREFIX = re.compile(r"^\[[^\]]+\]\s+")

_SELECTION_HEADER = "=== Interceptor Selection ==="

_LINE_FEASIBLE = re.compile(
    r"^(interceptor_\w+)\s*:\s*feasible\s*=\s*(True|False)\s*,\s*tti\s*=\s*(.+)$",
    re.IGNORECASE,
)
_LINE_SELECTED = re.compile(r"^selected\s*:\s*(\S+)", re.IGNORECASE)


def _strip_ros_launch_prefix(line: str) -> str:
    return _STRIP_LAUNCH_PREFIX.sub("", line.strip())


def _parse_tti_token(tt_raw: str) -> float | None:
    tt = tt_raw.strip().lower()
    if tt in ("", "none", "n/a", "na"):
        return None
    try:
        v = float(tt)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


@dataclass
class SelectionBlock:
    oracle_ids: tuple[str, ...]
    oracle_tti_min: float | None
    selected: str | None
    oracle_match: bool | None


def extract_selection_blocks(text: str) -> list[dict[str, object]]:
    lines = text.splitlines()
    out: list[dict[str, object]] = []

    i = 0
    max_scan_lines = 64
    while i < len(lines):
        headline = _strip_ros_launch_prefix(lines[i])
        if _SELECTION_HEADER not in headline:
            i += 1
            continue

        feas: dict[str, float | None] = {}
        sel: str | None = None
        j = i + 1
        end = min(len(lines), i + 1 + max_scan_lines)
        while j < end:
            nxt_strip = _strip_ros_launch_prefix(lines[j])
            line_stripped = nxt_strip.strip()
            # Next header starts a fresh block handled by outer loop at index j.
            if _SELECTION_HEADER in line_stripped:
                break

            m_fe = _LINE_FEASIBLE.match(line_stripped)
            if m_fe:
                feat_id = m_fe.group(1)
                feasible = m_fe.group(2).lower() == "true"
                if feasible:
                    feas[feat_id] = _parse_tti_token(m_fe.group(3))
                # infeasible ids are ignored for oracle (do not accumulate)

            m_sel = _LINE_SELECTED.match(line_stripped)
            if m_sel:
                cand = m_sel.group(1).strip().strip("'\"")
                if cand.lower() in ("none", "null"):
                    cand = ""
                sel = cand or None
            j += 1

        oracle_ids_lst, tmin = _oracle_from_feasibility(feas)
        match: bool | None = None if (sel is None or not oracle_ids_lst) else (sel in oracle_ids_lst)

        out.append(
            {
                "oracle_ids": oracle_ids_lst,
                "oracle_tti_min": tmin,
                "selected": sel,
                "oracle_match": match,
            },
        )
        # Continue from first line following this block without re-consuming the header.
        i = j if j > i else i + 1
    return out


def _oracle_from_feasibility(feas: dict[str, float | None]) -> tuple[list[str], float | None]:
    eligible: list[tuple[str, float]] = []
    for k, tt in feas.items():
        if tt is not None:
            eligible.append((k, float(tt)))
    if not eligible:
        return [], None
    t_best = min(t for _, t in eligible)
    return ([k for k, t in eligible if abs(t - t_best) < 1e-6], t_best)


def audit_selection_log(text: str) -> SelectionBlock | None:
    blk = extract_selection_blocks(text)
    if not blk:
        return None
    last_raw = blk[-1]
    oids_raw = last_raw.get("oracle_ids")
    oracle_ids_tuple = tuple(oids_raw) if isinstance(oids_raw, list) else ()
    oracle_tti = last_raw.get("oracle_tti_min")
    selected_raw = last_raw.get("selected")
    selected_val = selected_raw if isinstance(selected_raw, str) else None
    om = last_raw.get("oracle_match")
    om_bool = om if isinstance(om, bool) else None
    return SelectionBlock(
        oracle_ids=oracle_ids_tuple,
        oracle_tti_min=float(oracle_tti) if oracle_tti is not None else None,
        selected=selected_val,
        oracle_match=om_bool,
    )


def selection_audit_summary(text: str) -> dict:
    blocks = extract_selection_blocks(text)
    if not blocks:
        return {
            "n_selection_blocks": 0,
            "selection_oracle_match_rate": None,
            "last_selected": None,
            "last_oracle_ids": [],
            "last_oracle_tti_min": None,
            "last_oracle_match": None,
        }
    matches = sum(1 for b in blocks if b.get("oracle_match") is True)
    aud = sum(1 for b in blocks if b.get("oracle_match") is not None)
    lb = blocks[-1]
    oids = lb["oracle_ids"] if isinstance(lb.get("oracle_ids"), list) else []
    rate = matches / aud if aud else None
    return {
        "n_selection_blocks": len(blocks),
        "selection_oracle_match_rate": rate,
        "last_selected": lb.get("selected"),
        "last_oracle_ids": list(oids),
        "last_oracle_tti_min": lb.get("oracle_tti_min"),
        "last_oracle_match": lb.get("oracle_match"),
    }
