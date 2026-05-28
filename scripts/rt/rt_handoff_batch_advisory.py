#!/usr/bin/env python3
"""Batch RT→SA handoff advisory maintainer CLI (PLAT-RT-F6 P2 + F7 P0–P2)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
_SCRIPTS_RT = _REPO / "scripts" / "rt"
for p in (_BRIDGE_PKG, _SCRIPTS_RT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rt_sandbox.advisory_queue import (  # noqa: E402
    ADVISORY_BATCH_SUMMARY_SCHEMA,
    ADVISORY_BATCH_SUMMARY_V2_SCHEMA,
    FILTER_PRESET_IDS,
    TEMPLATE_PACK_IDS,
)
from rt_sandbox.batch_advisory import (  # noqa: E402
    ADVISORY_BATCH_REVIEW_V2_SCHEMA,
    AnnotateReviewResult,
    BatchFilters,
    annotate_handoff_review_note,
    build_advisory_batch_review_v2_document,
    build_advisory_batch_summary_document,
    build_advisory_batch_summary_v2_document,
    build_batch_review_document,
    build_dry_run_review_document,
    corpus_preview_for_capture,
    filter_rows,
    filter_rows_by_group,
    render_template_pack,
    repo_root_or_default,
    scan_staged,
    validate_advisory_batch_review_v2,
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


def _add_f7_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--sort",
        choices=["queue", "capture_id"],
        default="queue",
        help="Capture sort order (default: queue priority)",
    )
    parser.add_argument(
        "--group-by",
        dest="group_by",
        choices=[
            "normalization",
            "review_attestation",
            "approval_gate",
            "packaging",
            "lineage",
            "experiment_warn",
            "terminal_block",
        ],
        help="Filter rows matching blocker group",
    )
    parser.add_argument(
        "--manifest-ref",
        type=Path,
        default=None,
        help="Experiment manifest for warn-only rollup in summary",
    )
    parser.add_argument(
        "--schema",
        choices=["f7", "f6", "v2", "f8"],
        default="f7",
        help="Report/export schema (f7=summary v1; v2=stand-up review; f8=summary v2)",
    )


def _add_f8_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--preset",
        choices=list(FILTER_PRESET_IDS),
        default=None,
        help="Named filter preset (filter/sort only; does not run CLIs)",
    )
    parser.add_argument(
        "--focus-captures",
        default=None,
        help="Comma-separated capture ids for focus set (cognition only)",
    )
    parser.add_argument(
        "--cohort-index-ref",
        type=Path,
        default=None,
        help="X2 cohort index path for experiment_handoff_rollup (read-only)",
    )
    parser.add_argument(
        "--template-pack",
        choices=list(TEMPLATE_PACK_IDS),
        default=None,
        help="Render-only stand-up template pack (f8 export)",
    )


def _add_export_out(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("-"),
        help="Output path (- for stdout)",
    )


def _resolve_focus_ids(args: argparse.Namespace) -> list[str] | None:
    raw = getattr(args, "focus_captures", None)
    if not raw:
        return None
    return [p.strip() for p in str(raw).split(",") if p.strip()]


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


def _load_rows(args: argparse.Namespace, repo_root: Path):
    rows = scan_staged(repo_root, _resolve_ids(args))
    rows = filter_rows(rows, _filters_from_args(args))
    if getattr(args, "group_by", None):
        rows = filter_rows_by_group(rows, args.group_by, repo_root=repo_root)
    return rows


def _write_json_doc(doc: dict[str, Any], out: Path, *, schema_hint: str) -> None:
    text = json.dumps(doc, indent=2, sort_keys=True) + "\n"
    if out == Path("-"):
        sys.stdout.write(text)
    else:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
        print(f"wrote {out} ({schema_hint})", file=sys.stderr)


def cmd_scan(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = _load_rows(args, repo_root)
    dicts = [r.to_dict(repo_root=repo_root) for r in rows]
    if args.json:
        print(json.dumps(dicts, indent=2, sort_keys=True))
    else:
        for d in dicts:
            if d.get("error"):
                print(f"{d['capture_candidate_id']}\terror={d['error']}")
                continue
            adv = d.get("advisory") or {}
            qp = d.get("queue_priority") or {}
            print(
                f"{d['capture_candidate_id']}\t"
                f"state={adv.get('advisory_state')}\t"
                f"band={qp.get('band', '')}\t"
                f"cohort={d.get('readiness_cohort', '')}\t"
                f"blocked={adv.get('blocked')}"
            )
            groups = d.get("blocker_groups") or []
            if groups:
                print(f"  groups: {', '.join(groups)}")
            if d.get("next_cli"):
                print(f"  next: {d['next_cli']}")
    return 0


def _build_doc(args: argparse.Namespace, repo_root: Path, rows):
    if args.schema == "f6":
        return build_batch_review_document(repo_root, rows, dry_run=args.dry_run)
    if args.schema == "f8":
        doc = build_advisory_batch_summary_v2_document(
            repo_root,
            rows,
            dry_run=args.dry_run,
            manifest_ref=args.manifest_ref,
            sort_key=args.sort,
            preset_applied=getattr(args, "preset", None),
            focus_capture_ids=_resolve_focus_ids(args),
            cohort_index_ref=getattr(args, "cohort_index_ref", None),
        )
        pack = getattr(args, "template_pack", None)
        if pack:
            doc["template_render"] = render_template_pack(doc, pack)
        return doc
    if args.schema == "v2":
        doc = build_advisory_batch_review_v2_document(
            repo_root,
            rows,
            dry_run=True,
            manifest_ref=args.manifest_ref,
            sort_key=args.sort,
        )
        errors = validate_advisory_batch_review_v2(doc)
        if errors:
            raise ValueError("invalid v2 document: " + "; ".join(errors))
        return doc
    return build_advisory_batch_summary_document(
        repo_root,
        rows,
        dry_run=args.dry_run,
        manifest_ref=args.manifest_ref,
        sort_key=args.sort,
    )


def cmd_report(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = _load_rows(args, repo_root)
    try:
        doc = _build_doc(args, repo_root, rows)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(doc, indent=2, sort_keys=True))
    else:
        print(doc["governance_banner"])
        print(f"schema={doc['schema']} total={doc['summary']['total']} dry_run={doc['dry_run']}")
        for state, count in sorted(doc["summary"]["by_advisory_state"].items()):
            print(f"  {state}: {count}")
        groups = doc["summary"].get("blocker_groups") or {}
        if groups:
            print("blocker_groups:")
            for gid, info in sorted(groups.items(), key=lambda x: -x[1].get("count", 0)):
                print(f"  {gid}: {info.get('count', 0)}")
        cohorts = doc["summary"].get("readiness_cohorts") or {}
        if cohorts:
            print("readiness_cohorts:")
            for cid, count in sorted(cohorts.items()):
                print(f"  {cid}: {count}")
        if doc["summary"].get("block_reason_rollup"):
            print("block_reason_rollup:")
            for reason, count in sorted(doc["summary"]["block_reason_rollup"].items()):
                print(f"  {reason}: {count}")
        exp = doc["summary"].get("experiment_rollup")
        if exp:
            print(f"experiment_rollup: {exp.get('handoff_eligibility')} — {exp.get('note', '')}")
        cohorts_v2 = doc["summary"].get("readiness_cohorts_v2") or {}
        if cohorts_v2:
            print("readiness_cohorts_v2:")
            for cid, count in sorted(cohorts_v2.items()):
                print(f"  {cid}: {count}")
        hr = doc["summary"].get("handoff_rollup") or {}
        if hr.get("by_stage"):
            print("handoff_rollup.by_stage:")
            for stage, count in sorted(hr["by_stage"].items()):
                print(f"  {stage}: {count}")
        eh = doc["summary"].get("experiment_handoff_rollup")
        if eh:
            print(f"experiment_handoff_rollup: {eh.get('handoff_eligibility')} — {eh.get('note', '')}")
        if doc.get("preset_applied"):
            print(f"preset_applied: {doc['preset_applied']}")
        if doc.get("focus_capture_ids"):
            print(f"focus_capture_ids: {', '.join(doc['focus_capture_ids'])}")
        tr = doc.get("template_render")
        if tr and not args.json:
            print(f"template_pack: {tr.get('pack_id')} ({tr.get('format')})")
        standup = doc.get("standup")
        if standup:
            pri = standup.get("priority_capture_ids") or []
            if pri:
                print(f"standup priority captures: {', '.join(pri)}")
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = _load_rows(args, repo_root)
    try:
        doc = _build_doc(args, repo_root, rows)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    _write_json_doc(doc, args.out, schema_hint=doc.get("schema", ADVISORY_BATCH_SUMMARY_SCHEMA))
    return 0


def cmd_standup_export(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = _load_rows(args, repo_root)
    doc = build_advisory_batch_review_v2_document(
        repo_root,
        rows,
        dry_run=True,
        manifest_ref=args.manifest_ref,
        sort_key=args.sort,
    )
    errors = validate_advisory_batch_review_v2(doc)
    if errors:
        print("validation failed: " + "; ".join(errors), file=sys.stderr)
        return 1
    _write_json_doc(doc, args.out, schema_hint=ADVISORY_BATCH_REVIEW_V2_SCHEMA)
    if args.out != Path("-"):
        print(
            f"stand-up export: {doc['summary']['total']} capture(s)",
            file=sys.stderr,
        )
    return 0


def cmd_grouped_export(args: argparse.Namespace) -> int:
    return cmd_standup_export(args)


def cmd_dry_run_review(args: argparse.Namespace) -> int:
    repo_root = repo_root_or_default(args.repo_root)
    rows = _load_rows(args, repo_root)
    state_filter = frozenset(args.states) if args.states else None
    doc = build_dry_run_review_document(
        repo_root,
        rows,
        max_captures=args.max_captures,
        states=state_filter,
        write_preview=args.write_preview,
    )
    if args.json or args.out == Path("-"):
        text = json.dumps(doc, indent=2, sort_keys=True) + "\n"
        if args.out == Path("-"):
            sys.stdout.write(text)
        else:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(text, encoding="utf-8")
            print(f"wrote {args.out}", file=sys.stderr)
    else:
        _write_json_doc(doc, args.out, schema_hint=doc["schema"])
    if not args.json and args.out == Path("-"):
        ran_ok = sum(1 for c in doc["captures"] if c.get("status") == "ran" and c.get("rc") == 0)
        ran_fail = sum(1 for c in doc["captures"] if c.get("status") == "ran" and c.get("rc") not in (0, None))
        skipped = sum(1 for c in doc["captures"] if c.get("status") == "skipped")
        errors = sum(1 for c in doc["captures"] if c.get("status") == "error")
        print(
            "dry-run-review (preview only): "
            f"ran_ok={ran_ok} ran_fail={ran_fail} skipped={skipped} error={errors}",
            file=sys.stderr,
        )
        print("commit: forbidden — use rt_sa_import commit --corpus-dest explicitly", file=sys.stderr)
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
        if preview.get("dest_policy"):
            print(f"dest_policy: {preview.get('dest_policy')}")
        if preview.get("proposed_dest_rel"):
            print(f"proposed_dest_rel: {preview.get('proposed_dest_rel')}")
        if preview.get("dest_valid") is False:
            print("dest_valid: false (preview only; dest rejected by policy)")
        if preview.get("bundle_index_present") is False:
            print("bundle_index_present: false")
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
    _add_f7_options(report_p)
    _add_f8_options(report_p)
    report_p.set_defaults(func=cmd_report)

    exp = sub.add_parser(
        "export",
        parents=[common],
        help="Write batch advisory JSON (default f7; --schema v2 for stand-up export)",
    )
    _add_filters(exp)
    _add_f7_options(exp)
    _add_f8_options(exp)
    _add_export_out(exp)
    exp.set_defaults(func=cmd_export)

    standup_p = sub.add_parser(
        "standup-export",
        parents=[common],
        help="Write rt_advisory_batch_review_v2 stand-up JSON",
    )
    _add_filters(standup_p)
    _add_f7_options(standup_p)
    _add_export_out(standup_p)
    standup_p.set_defaults(func=cmd_standup_export, schema="v2")

    grouped_p = sub.add_parser(
        "grouped-export",
        parents=[common],
        help="Write rt_advisory_batch_review_v2 with grouped indexes",
    )
    _add_filters(grouped_p)
    _add_f7_options(grouped_p)
    _add_export_out(grouped_p)
    grouped_p.set_defaults(func=cmd_grouped_export, schema="v2")

    dry_p = sub.add_parser(
        "dry-run-review",
        parents=[common],
        help="Batch import pipeline dry-run preview (read-only; no commit)",
    )
    _add_filters(dry_p)
    _add_export_out(dry_p)
    dry_p.add_argument(
        "--max-captures",
        type=int,
        default=10,
        help="Max captures to run dry-run against (default 10)",
    )
    dry_p.add_argument(
        "--states",
        nargs="+",
        choices=["import_ready", "handoff_ready"],
        default=None,
        help="Advisory states eligible for dry-run (default import_ready handoff_ready)",
    )
    dry_p.add_argument(
        "--write-preview",
        action="store_true",
        help="Write staging dry_run_preview.json per capture (default off)",
    )
    dry_p.set_defaults(func=cmd_dry_run_review)

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
