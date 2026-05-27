#!/usr/bin/env python3
"""Read-only RT capture staging inspection (dev/maintainer only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.capture import list_staged_capture_ids, validate_capture_candidate  # noqa: E402
from rt_sandbox.capture_handoff_mirror import capture_belongs_to_session  # noqa: E402
from rt_sandbox.capture_normalize import validate_normalized_capture  # noqa: E402
from rt_sandbox.tactical_capture_annex import validate_tactical_annex  # noqa: E402
from rt_sandbox.isolation import repo_root_from, rt_sandbox_captures_dir  # noqa: E402
from rt_sandbox.sa_handoff import handoff_status_summary  # noqa: E402


def cmd_list(repo_root: Path, session_id: str | None = None) -> int:
    ids = list_staged_capture_ids(repo_root)
    if not ids:
        print("No staged captures.")
        return 0
    shown = 0
    for cid in ids:
        cand_path = rt_sandbox_captures_dir(repo_root) / cid / "candidate.json"
        data = json.loads(cand_path.read_text(encoding="utf-8"))
        if session_id and not capture_belongs_to_session(data, session_id):
            continue
        print(
            f"{cid}\tapproval={data.get('approval_status')}\t"
            f"session={data.get('ephemeral_session_ref', data.get('session_id'))}"
        )
        shown += 1
    if session_id and shown == 0:
        print(f"No staged captures for session_id={session_id}.")
    return 0


def cmd_show(repo_root: Path, capture_id: str) -> int:
    staging = rt_sandbox_captures_dir(repo_root) / capture_id
    cand = staging / "candidate.json"
    if not cand.exists():
        print(f"Capture not found: {capture_id}", file=sys.stderr)
        return 1
    print(json.dumps(json.loads(cand.read_text(encoding="utf-8")), indent=2))
    return 0


def cmd_normalization_status(repo_root: Path, capture_id: str | None) -> int:
    root = rt_sandbox_captures_dir(repo_root)
    targets = [capture_id] if capture_id else list_staged_capture_ids(repo_root)
    if not targets:
        print("No captures to inspect.")
        return 0
    for cid in targets:
        staging = root / cid
        cand_path = staging / "candidate.json"
        if not cand_path.exists():
            print(f"{cid}: missing candidate")
            continue
        cand = json.loads(cand_path.read_text(encoding="utf-8"))
        norm_status = cand.get("normalization_status", "unknown")
        val_errors = validate_normalized_capture(staging)
        prov = {}
        prov_path = staging / "provenance.json"
        if prov_path.exists():
            prov = json.loads(prov_path.read_text(encoding="utf-8"))
        annex_flag = "yes" if (staging / "tactical_annex.json").is_file() else "no"
        print(
            f"{cid}\tnormalization={norm_status}\t"
            f"revision={cand.get('conversion_revision', '-')}\t"
            f"valid={not val_errors}\t"
            f"tactical_annex={annex_flag}"
        )
        if val_errors:
            print(f"  errors: {', '.join(val_errors)}")
        if prov:
            print(
                f"  adapter_attached={prov.get('adapter_attached')} "
                f"mode={prov.get('adapter_mode')}"
            )
    return 0


def cmd_validate(repo_root: Path, capture_id: str | None) -> int:
    root = rt_sandbox_captures_dir(repo_root)
    targets = [capture_id] if capture_id else list_staged_capture_ids(repo_root)
    if not targets:
        print("No captures to validate.")
        return 0
    failed = 0
    for cid in targets:
        errors = validate_capture_candidate(root / cid / "candidate.json")
        if errors:
            print(f"{cid}: FAIL — {', '.join(errors)}")
            failed += 1
        else:
            print(f"{cid}: OK")
    return 1 if failed else 0


def cmd_tactical_continuity(repo_root: Path, capture_id: str | None) -> int:
    root = rt_sandbox_captures_dir(repo_root)
    targets = [capture_id] if capture_id else list_staged_capture_ids(repo_root)
    if not targets:
        print("No captures to inspect.")
        return 0
    for cid in targets:
        staging = root / cid
        annex_path = staging / "tactical_annex.json"
        norm_path = staging / "normalized_manifest.json"
        print(f"=== {cid} ===")
        if not annex_path.exists():
            print("  tactical_annex: none")
            if norm_path.exists():
                norm = json.loads(norm_path.read_text(encoding="utf-8"))
                if norm.get("tactical_annex"):
                    print("  warning: embedded tactical_annex in manifest without sidecar")
            continue
        annex = json.loads(annex_path.read_text(encoding="utf-8"))
        err = validate_tactical_annex(annex)
        print(f"  schema: {annex.get('schema')}")
        print(f"  final_tactical_mode: {annex.get('final_tactical_mode')}")
        print(f"  selected_id: {annex.get('selected_id')}")
        print(f"  assigned_target: {annex.get('assigned_target')}")
        for key in (
            "selected_timeline",
            "assignment_timeline",
            "tti_timeline",
            "recommendation_timeline",
            "mode_switches",
            "pause_resume_transitions",
            "assignment_lock_events",
            "target_switch_events",
        ):
            print(f"  {key}: {len(annex.get(key) or [])}")
        print(f"  validation: {'OK' if err is None else err}")
        if norm_path.exists():
            norm = json.loads(norm_path.read_text(encoding="utf-8"))
            embedded = norm.get("tactical_annex") is not None
            print(f"  normalized_manifest_embedded: {embedded}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect RT capture staging (read-only)")
    sub = parser.add_subparsers(dest="command", required=True)
    list_p = sub.add_parser("list", help="List staged capture candidate IDs")
    list_p.add_argument(
        "--session-id",
        default=None,
        help="Filter to captures for this session_id / ephemeral_session_ref",
    )
    show_p = sub.add_parser("show", help="Show candidate.json for one capture")
    show_p.add_argument("capture_id")
    val_p = sub.add_parser("validate", help="Validate candidate schema(s)")
    val_p.add_argument("capture_id", nargs="?", default=None)
    norm_p = sub.add_parser(
        "normalization-status",
        help="Show normalization status and validation for capture(s)",
    )
    norm_p.add_argument("capture_id", nargs="?", default=None)
    handoff_p = sub.add_parser(
        "handoff-status",
        help="Show handoff review, manifest, steps, and export events",
    )
    handoff_p.add_argument("capture_id")
    tac_p = sub.add_parser(
        "tactical-continuity",
        help="Summarize tactical capture annex timelines (read-only)",
    )
    tac_p.add_argument("capture_id", nargs="?", default=None)
    args = parser.parse_args()
    repo_root = repo_root_from()
    if args.command == "list":
        return cmd_list(repo_root, getattr(args, "session_id", None))
    if args.command == "show":
        return cmd_show(repo_root, args.capture_id)
    if args.command == "validate":
        return cmd_validate(repo_root, args.capture_id)
    if args.command == "normalization-status":
        return cmd_normalization_status(repo_root, args.capture_id)
    if args.command == "handoff-status":
        print(json.dumps(handoff_status_summary(repo_root, args.capture_id), indent=2))
        return 0
    if args.command == "tactical-continuity":
        return cmd_tactical_continuity(repo_root, args.capture_id)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
