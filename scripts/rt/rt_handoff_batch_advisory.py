#!/usr/bin/env python3
"""Batch RT→SA handoff advisory maintainer CLI (PLAT-RT-F6 P2)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.batch_advisory import (  # noqa: E402
    AnnotateReviewResult,
    BatchFilters,
    annotate_handoff_review_note,
    build_batch_review_document,
    corpus_preview_for_capture,
    filter_rows,
    repo_root_or_default,
    scan_staged,
)

def _add_filters(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--capture-id",
        action="append",
        dest="capture_ids",
        default=None,
        help="Limit to capture id (repeatable)",
    )
    parser.add_argument(
        "--ids-file",
        type=Path,
        help="Newline-separated capture ids",
    )
    parser.add_argument(
        "--state",
        choices=[
            "capture_ready",
            "review_complete",
            "approval_ready",
            "handoff_ready",
            "import_ready",
            "blocked",
            "committed",
            "not_ready",
            "error",
        ],
        help="Filter by advisory state key",
    )
    parser.add_argument("--blocked-only", action="store_true")
    parser.add_argument("--reject-only", action="store_true")
    parser.add_argument("--defer-only", action="store_true")
    parser.add_argument("--terminal", action="store_true", dest="terminal_committed")


def _resolve_ids(args: argparse.Namespace) -> list[str] | None:
    ids: list[str] = list(args.capture_ids or [])
    if args.ids_file:
        text = args.ids_file.read_text(encoding="utf-8")
        ids.extend(line.strip() for line in text.splitlines() if line.strip())
    return ids if ids else None


def _filters_from_args(args: argparse.Namespace) -> BatchFilters:
    return BatchFilters(
        state=args.state,
        blocked_only=args.blocked_only,
        reject_only=args.reject_only,
        defer_only=args.defer_only,
        terminal_committed=args.terminal_committed,
    )


def cmd_scan(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = scan_staged(repo_root, _resolve_ids(args))
    rows = filter_rows(rows, _filters_from_args(args))
    if args.json:
        print(json.dumps([r.to_dict() for r in rows], indent=2, sort_keys=True))
    else:
        for row in rows:
            if row.error:
                print(f"{row.capture_candidate_id}\terror={row.error}")
                continue
            adv = row.advisory
            print(
                f"{row.capture_candidate_id}\t"
                f"state={adv.get('advisory_state')}\t"
                f"blocked={adv.get('blocked')}\t"
                f"phase={row.workflow_phase or ''}"
            )
            if row.next_cli:
                print(f"  next: {row.next_cli}")
    return 0


def cmd_report(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = scan_staged(repo_root, _resolve_ids(args))
    rows = filter_rows(rows, _filters_from_args(args))
    doc = build_batch_review_document(repo_root, rows, dry_run=args.dry_run)
    if args.json:
        print(json.dumps(doc, indent=2, sort_keys=True))
    else:
        print(doc["governance_banner"])
        print(f"total={doc['summary']['total']} dry_run={doc['dry_run']}")
        for state, count in sorted(doc["summary"]["by_advisory_state"].items()):
            print(f"  {state}: {count}")
        if doc["summary"]["block_reason_rollup"]:
            print("blockers:")
            for reason, count in sorted(doc["summary"]["block_reason_rollup"].items()):
                print(f"  {reason}: {count}")
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = scan_staged(repo_root, _resolve_ids(args))
    rows = filter_rows(rows, _filters_from_args(args))
    doc = build_batch_review_document(repo_root, rows, dry_run=args.dry_run)
    text = json.dumps(doc, indent=2, sort_keys=True) + "\n"
    if args.out == Path("-"):
        sys.stdout.write(text)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        print(f"wrote {args.out}", file=sys.stderr)
    return 0


def cmd_corpus_preview(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    preview = corpus_preview_for_capture(
        repo_root,
        args.capture_id,
        corpus_dest=args.corpus_dest,
    )
    if args.json:
        print(json.dumps(preview, indent=2, sort_keys=True))
    else:
        print(preview["governance_banner"])
        for label in ("would_add", "would_conflict", "missing_staging_ref"):
            items = preview.get(label) or []
            if items:
                print(f"{label}:")
                for item in items:
                    print(f"  - {item}")
        for note in preview.get("notes") or []:
            print(f"note: {note}")
    return 0


def cmd_annotate_review(args: argparse.Namespace) -> int:
    if args.dry_run:
        print("annotate-review requires --no-dry-run", file=sys.stderr)
        return 1
    if not args.allow_write:
        print("annotate-review requires --allow-write", file=sys.stderr)
        return 1
    if args.confirm_capture_id != args.capture_id:
        print("--confirm-capture-id must match capture_id", file=sys.stderr)
        return 1
    repo_root = repo_root_or_default(args.repo_root)
    result: AnnotateReviewResult = annotate_handoff_review_note(
        repo_root,
        args.capture_id,
        note=args.note,
        reviewer=args.reviewer,
    )
    if args.json:
        print(json.dumps({"ok": result.ok, "message": result.message}, indent=2))
    else:
        print(result.message)
    return 0 if result.ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Batch RT→SA handoff advisory (maintainer only; advisory ≠ authority).",
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--repo-root", type=Path, default=None)
    common.add_argument("--json", action="store_true")
    common.add_argument(
        "--dry-run",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Default on; read-only subcommands ignore writes",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    scan_p = sub.add_parser(
        "scan",
        parents=[common],
        help="List advisory state per capture",
    )
    _add_filters(scan_p)
    scan_p.set_defaults(func=cmd_scan)

    report_p = sub.add_parser(
        "report",
        parents=[common],
        help="Aggregate advisory summary",
    )
    _add_filters(report_p)
    report_p.set_defaults(func=cmd_report)

    exp = sub.add_parser(
        "export",
        parents=[common],
        help="Write rt_handoff_batch_review_v1 JSON",
    )
    _add_filters(exp)
    exp.add_argument(
        "--out",
        type=Path,
        default=Path("-"),
        help="Output path (- for stdout)",
    )
    exp.set_defaults(func=cmd_export)

    cp = sub.add_parser(
        "corpus-preview",
        parents=[common],
        help="Read-only corpus path preview",
    )
    cp.add_argument("capture_id")
    cp.add_argument(
        "--corpus-dest",
        type=Path,
        default=None,
        help="Proposed fixtures/sa_r0 destination",
    )
    cp.set_defaults(func=cmd_corpus_preview)

    ann = sub.add_parser(
        "annotate-review",
        parents=[common],
        help="Append note to handoff_review.json (explicit write gates)",
    )
    ann.add_argument("capture_id")
    ann.add_argument("--note", required=True)
    ann.add_argument("--confirm-capture-id", required=True)
    ann.add_argument("--allow-write", action="store_true")
    ann.add_argument("--reviewer", default="batch_advisory_cli")
    ann.set_defaults(func=cmd_annotate_review)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
