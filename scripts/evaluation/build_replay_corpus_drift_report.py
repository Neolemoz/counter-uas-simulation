#!/usr/bin/env python3
"""Build deterministic replay corpus drift report (replay_corpus_drift_report_v1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_replay_corpus_index import build_corpus_index  # noqa: E402
from replay_corpus_lineage import (  # noqa: E402
    CORPUS_ID,
    DRIFT_GENERATION_REVISION,
    DRIFT_GOVERNANCE,
    collect_drift_findings,
    summarize_findings,
    write_viewer_mirror,
)

_REPO = Path(__file__).resolve().parents[2]
_AUDITS = _REPO / "fixtures/sa_r0/corpus_audits"
_DEFAULT_INDEX = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"


def build_drift_report(index: dict[str, Any], *, expected_index: dict[str, Any] | None = None) -> dict[str, Any]:
    findings = collect_drift_findings(index, repo_root=_REPO, expected_index=expected_index)
    return {
        "artifact_type": "replay_corpus_drift_report_v1",
        "schema_version": "replay_corpus_drift_report_v1",
        "corpus_id": index.get("corpus_id") or CORPUS_ID,
        "index_revision": index.get("index_revision") or "",
        "generation_revision": DRIFT_GENERATION_REVISION,
        "governance": DRIFT_GOVERNANCE,
        "summary": summarize_findings(findings),
        "findings": findings,
    }


def render_drift_summary(report: dict[str, Any]) -> str:
    lines = [
        "# Corpus drift summary",
        "",
        report.get("governance", {}).get("notice", ""),
        "",
        f"Corpus: `{report.get('corpus_id')}` · index revision `{report.get('index_revision', '')[:12]}…`",
        "",
        f"Total findings: **{report.get('summary', {}).get('total', 0)}**",
        "",
    ]
    by_kind = report.get("summary", {}).get("by_kind") or {}
    if by_kind:
        lines.append("## By kind")
        lines.append("")
        for kind, count in by_kind.items():
            lines.append(f"- `{kind}`: {count}")
        lines.append("")

    for f in (report.get("findings") or [])[:30]:
        lines.append(f"- **[{f.get('severity')}]** `{f.get('kind')}` — {f.get('message')}")
    if len(report.get("findings") or []) > 30:
        lines.append(f"- … and {len(report['findings']) - 30} more (see JSON report)")
    lines.extend(["", "## Interpretation", "", "- Drift is fixture integrity only, not operational failure.", ""])
    return "\n".join(lines)


def write_drift_report(report: dict[str, Any]) -> None:
    _AUDITS.mkdir(parents=True, exist_ok=True)
    json_text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    drift_json = _AUDITS / "replay_corpus_drift_report_v1.json"
    drift_json.write_text(json_text, encoding="utf-8")
    (_AUDITS / "corpus_drift_summary.md").write_text(render_drift_summary(report), encoding="utf-8")
    mirror = _REPO / "platform/sa-r0-viewer/public/demo/corpus_audits/replay_corpus_drift_report_v1.json"
    write_viewer_mirror(drift_json, mirror)


def check_drift_report() -> None:
    index = json.loads(_DEFAULT_INDEX.read_text(encoding="utf-8"))
    expected = build_drift_report(index, expected_index=build_corpus_index())
    path = _AUDITS / "replay_corpus_drift_report_v1.json"
    if not path.is_file():
        raise SystemExit(f"missing drift report: {path}")
    actual = json.loads(path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit("replay_corpus_drift_report_v1.json is stale — run build_replay_corpus_drift_report.py")


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay corpus drift report")
    ap.add_argument("--check", action="store_true", help="verify committed report")
    ap.add_argument("--json", action="store_true", help="print report JSON to stdout")
    args = ap.parse_args()

    if args.check:
        check_drift_report()
        print("corpus drift report check OK")
        return

    index = json.loads(_DEFAULT_INDEX.read_text(encoding="utf-8"))
    report = build_drift_report(index, expected_index=build_corpus_index())
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
        return

    write_drift_report(report)
    print(f"corpus drift report OK ({report['summary']['total']} findings)")


if __name__ == "__main__":
    main()
