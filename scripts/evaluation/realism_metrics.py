#!/usr/bin/env python3
"""Additive realism-metric extraction from runtime logs.

This helper is intentionally separate from the stable parser-visible summary surfaces.
It extracts optional runtime-realism counters from additive log tags and existing
tracking lifecycle logs without changing the canonical success/miss/intercept-time
contracts.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

_DELAYED_RE = re.compile(r"\[REALISM_EVENT\]\s+delayed_detection\b")
_STALE_RE = re.compile(r"\[REALISM_EVENT\]\s+stale_detection\b")
_BURST_RE = re.compile(r"\[REALISM_EVENT\]\s+dropout_burst_start\b")
_GHOST_RE = re.compile(r"\[REALISM_EVENT\]\s+ghost_detection\b")
_FRAG_START_RE = re.compile(r"\[REALISM_EVENT\]\s+fragmented_gap_start\b")
_FRAG_END_RE = re.compile(r"\[REALISM_EVENT\]\s+fragmented_gap_end\b")
_LIFECYCLE_OBSERVER_SUMMARY_RE = re.compile(
    r"\[LIFECYCLE_OBSERVER\]\s+event=summary\b.*?"
    r"tracks_state_msgs_window=(?P<msgs>\d+).*?"
    r"unique_track_ids_window=(?P<uniq>\d+).*?"
    r"track_persistence_events_window=(?P<persist>\d+)",
)
_TRACK_CONTINUITY_GAP_RE = re.compile(r"\[TRACK_CONTINUITY\]\s+event=track_gap\b")
_TRACK_CONTINUITY_CHANGE_RE = re.compile(r"\[TRACK_CONTINUITY\]\s+event=track_id_change\b")
_LIFECYCLE_OBSERVER_SELECTED_RE = re.compile(r"\[LIFECYCLE_OBSERVER\]\s+event=selected_id\b")
_SELECTION_PROXY_RE = re.compile(r"\[SELECTION_PROXY\]\s+event=track_persistence_window\b")
_TRACK_PERSISTENCE_RE = re.compile(r"\[TRACK_PERSISTENCE\]\s+event=track_persistence_boundary\b")
_TRACK_MISS_RE = re.compile(r"Track\s+(?P<tid>\d+)\s+NOT updated \(missed_frames=(?P<mf>\d+)\)")
_TRACK_UPDATE_RE = re.compile(r"Track\s+(?P<tid>\d+)\s+updated \(valid match\)")
_CANDIDATE_DETECTED_RE = re.compile(r"Candidate detected:")
_CANDIDATE_DISCARDED_RE = re.compile(r"Candidate discarded:")
_TRACK_MERGE_RE = re.compile(r"Merging track\s+\d+\s+and track\s+\d+")


def realism_metrics_summary(text: str) -> dict[str, int]:
    delayed_detection_count = 0
    stale_detection_count = 0
    dropout_burst_count = 0
    ghost_detection_count = 0
    fragmented_gap_count = 0
    fragmented_gap_end_count = 0
    lifecycle_observer_summary_count = 0
    tracks_state_observation_count = 0
    unique_track_ids_observed_count = 0
    track_continuity_gap_count = 0
    track_continuity_change_count = 0
    selection_proxy_window_count = 0
    selection_proxy_event_count = 0
    track_persistence_boundary_count = 0
    lifecycle_observer_selected_id_count = 0
    candidate_spawn_count = 0
    candidate_discard_count = 0
    duplicate_track_merge_count = 0
    track_coast_count = 0
    track_recovery_count = 0
    max_track_missed_frames = 0
    recovery_thrash_count = 0

    track_was_coasting: dict[str, bool] = {}
    track_recovery_cycles: dict[str, int] = {}

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if _DELAYED_RE.search(line):
            delayed_detection_count += 1
        if _STALE_RE.search(line):
            stale_detection_count += 1
        if _BURST_RE.search(line):
            dropout_burst_count += 1
        if _GHOST_RE.search(line):
            ghost_detection_count += 1
        if _FRAG_START_RE.search(line):
            fragmented_gap_count += 1
        if _FRAG_END_RE.search(line):
            fragmented_gap_end_count += 1
        ms = _LIFECYCLE_OBSERVER_SUMMARY_RE.search(line)
        if ms:
            lifecycle_observer_summary_count += 1
            tracks_state_observation_count += int(ms.group("msgs"))
            unique_track_ids_observed_count += int(ms.group("uniq"))
            selection_proxy_window_count += int(ms.group("persist"))
        if _TRACK_CONTINUITY_GAP_RE.search(line):
            track_continuity_gap_count += 1
        if _TRACK_CONTINUITY_CHANGE_RE.search(line):
            track_continuity_change_count += 1
        if _LIFECYCLE_OBSERVER_SELECTED_RE.search(line):
            lifecycle_observer_selected_id_count += 1
        if _SELECTION_PROXY_RE.search(line):
            selection_proxy_event_count += 1
        if _TRACK_PERSISTENCE_RE.search(line):
            track_persistence_boundary_count += 1
        if _CANDIDATE_DETECTED_RE.search(line):
            candidate_spawn_count += 1
        if _CANDIDATE_DISCARDED_RE.search(line):
            candidate_discard_count += 1
        if _TRACK_MERGE_RE.search(line):
            duplicate_track_merge_count += 1

        mm = _TRACK_MISS_RE.search(line)
        if mm:
            track_coast_count += 1
            tid = mm.group("tid")
            mf = int(mm.group("mf"))
            max_track_missed_frames = max(max_track_missed_frames, mf)
            track_was_coasting[tid] = True
            continue

        mu = _TRACK_UPDATE_RE.search(line)
        if mu:
            tid = mu.group("tid")
            if track_was_coasting.get(tid, False):
                track_recovery_count += 1
                track_recovery_cycles[tid] = track_recovery_cycles.get(tid, 0) + 1
                if track_recovery_cycles[tid] > 1:
                    recovery_thrash_count += 1
            track_was_coasting[tid] = False

    unrecovered_coast_track_count = sum(1 for was_coasting in track_was_coasting.values() if was_coasting)
    candidate_churn_pressure_count = candidate_spawn_count + candidate_discard_count
    coast_recovery_cycle_count = track_recovery_count
    lifecycle_thrash_score = (
        candidate_discard_count
        + duplicate_track_merge_count
        + recovery_thrash_count
        + unrecovered_coast_track_count
    )

    return {
        "delayed_detection_count": delayed_detection_count,
        "stale_detection_count": stale_detection_count,
        "dropout_burst_count": dropout_burst_count,
        "ghost_detection_count": ghost_detection_count,
        "fragmented_gap_count": fragmented_gap_count,
        "fragmented_gap_end_count": fragmented_gap_end_count,
        "lifecycle_observer_summary_count": lifecycle_observer_summary_count,
        "tracks_state_observation_count": tracks_state_observation_count,
        "unique_track_ids_observed_count": unique_track_ids_observed_count,
        "track_continuity_gap_count": track_continuity_gap_count,
        "track_continuity_change_count": track_continuity_change_count,
        "selection_proxy_window_count": selection_proxy_window_count,
        "selection_proxy_event_count": selection_proxy_event_count,
        "track_persistence_boundary_count": track_persistence_boundary_count,
        "lifecycle_observer_selected_id_count": lifecycle_observer_selected_id_count,
        "candidate_spawn_count": candidate_spawn_count,
        "candidate_discard_count": candidate_discard_count,
        "duplicate_track_merge_count": duplicate_track_merge_count,
        "track_coast_count": track_coast_count,
        "track_recovery_count": track_recovery_count,
        "max_track_missed_frames": max_track_missed_frames,
        "coast_recovery_cycle_count": coast_recovery_cycle_count,
        "recovery_thrash_count": recovery_thrash_count,
        "unrecovered_coast_track_count": unrecovered_coast_track_count,
        "candidate_churn_pressure_count": candidate_churn_pressure_count,
        "lifecycle_thrash_score": lifecycle_thrash_score,
    }


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: realism_metrics.py <log_path>", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    text = path.read_text(encoding="utf-8", errors="replace")
    print(json.dumps(realism_metrics_summary(text), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
