#!/usr/bin/env python3
"""Central SA platform integrity auditor (PLAT-SA-STAB)."""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import sa_integrity_lib as lib  # noqa: E402
from aggregate_spatial_analytics import load_bundle  # noqa: E402
from build_cross_sweep_synthesis import SWEEP_IDS  # noqa: E402
from classify_replay_pattern import PATTERN_PRIORITY, classify_member_patterns  # noqa: E402
from governance_lint_sa import batch_lint_sa_fixtures  # noqa: E402
from replay_narrative_intelligence import _first_detection_t  # noqa: E402

_REPO = lib.repo_root()
FIXTURES_SA = lib.FIXTURES_SA
PUBLIC_DEMO = lib.PUBLIC_DEMO
SCENARIOS = lib.SCENARIOS

DUPLICATE_BULLET_MIN_COUNT = 3
DUPLICATE_CAVEAT_MIN_COUNT = 4

CHECK_IDS = (
    "authoring_integrity",
    "orchestration_integrity",
    "orchestration_async_integrity",
    "orchestration_recovery_integrity",
    "catalog_sync",
    "fixture_pub_parity",
    "synthesis_stale",
    "linkage_stale",
    "linkage_graph",
    "storyboard_urls",
    "pattern_taxonomy",
    "narrative_duplicates",
    "sweep_reports",
    "research_bundle",
    "corpus_index_stale",
    "corpus_lineage",
    "corpus_release_stale",
    "corpus_drift_stale",
    "corpus_provenance",
    "corpus_release_diff",
    "corpus_viewer_audit_mirror",
    "corpus_evolution_stale",
    "corpus_publication_stale",
    "corpus_release_export",
    "federation_index_stale",
    "federation_integrity",
    "federation_viewer_mirror",
    "federation_recovery_continuity",
    "bundle_catalog",
    "governance_batch",
)


def _errors(check_id: str, messages: list[str]) -> list[str]:
    return [f"[{check_id}] {m}" for m in messages]


def check_authoring_integrity() -> list[str]:
    import replay_sa_authoring_integrity as auth_int  # noqa: E402

    result = auth_int.run_integrity_audit(strict=True)
    issues = list(result.get("errors") or [])
    return _errors("authoring_integrity", issues)


def check_orchestration_integrity() -> list[str]:
    import replay_sa_orchestration_integrity as orch_int  # noqa: E402

    result = orch_int.run_integrity_audit(strict=True)
    issues = list(result.get("errors") or [])
    return _errors("orchestration_integrity", issues)


def check_orchestration_async_integrity() -> list[str]:
    import replay_sa_orchestration_async as orch_async  # noqa: E402

    result = orch_async.run_async_integrity_audit(strict=True)
    issues = [
        f"{i.get('kind')}: {i.get('manifest_id')}: {i.get('message')}"
        for i in (result.get("issues") or [])
    ]
    return _errors("orchestration_async_integrity", issues)


def check_orchestration_recovery_integrity() -> list[str]:
    import replay_sa_orchestration_recovery as orch_recovery  # noqa: E402

    result = orch_recovery.run_recovery_integrity_audit(strict=True)
    issues = [
        f"{i.get('kind')}: {i.get('manifest_id')}: {i.get('message')}"
        for i in (result.get("issues") or [])
    ]
    return _errors("orchestration_recovery_integrity", issues)


def check_catalog_sync() -> list[str]:
    issues: list[str] = []
    pairs = (
        (SCENARIOS / "compare_pairs_v1.json", PUBLIC_DEMO / "compare_pairs.json", "compare_pairs"),
        (SCENARIOS / "index.json", PUBLIC_DEMO / "catalog.json", "catalog"),
        (SCENARIOS / "sweeps_index_v1.json", PUBLIC_DEMO / "sweeps_index.json", "sweeps_index"),
    )
    for a, b, label in pairs:
        if a.is_file():
            err = lib.assert_json_files_equal(a, b, label)
            if err:
                issues.append(err)
    return _errors("catalog_sync", issues)


def check_fixture_pub_parity() -> list[str]:
    issues: list[str] = []
    fix_synth = FIXTURES_SA / "synthesis"
    pub_synth = PUBLIC_DEMO / "synthesis"
    for rel in lib.SYNTHESIS_PARITY_FILES:
        err = lib.parity_file(fix_synth, pub_synth, rel, json_file=rel.endswith(".json"))
        if err:
            issues.append(err)

    for fix_path, pub_path in lib.iter_presentation_json_pairs():
        err = lib.assert_json_files_equal(fix_path, pub_path, fix_path.name)
        if err:
            issues.append(err)

    for sweep_id in SWEEP_IDS:
        fix_reports = FIXTURES_SA / "sweeps" / sweep_id / "reports"
        pub_reports = PUBLIC_DEMO / "sweeps" / sweep_id / "reports"
        if not fix_reports.is_dir():
            issues.append(f"missing sweep reports: {fix_reports}")
            continue
        for suffix in lib.SWEEP_REPORT_PARITY_SUFFIXES:
            err = lib.parity_file(fix_reports, pub_reports, suffix, json_file=suffix.endswith(".json"))
            if err:
                issues.append(err)

    return _errors("fixture_pub_parity", issues)


def _run_script_check(script: str, *args: str) -> list[str]:
    cmd = [sys.executable, str(_EVAL / script), *args]
    proc = subprocess.run(cmd, cwd=_REPO, capture_output=True, text=True)
    if proc.returncode != 0:
        msg = (proc.stderr or proc.stdout or "").strip() or f"exit {proc.returncode}"
        return [f"{script} {' '.join(args)}: {msg}"]
    return []


def check_synthesis_stale() -> list[str]:
    return _errors("synthesis_stale", _run_script_check("build_cross_sweep_synthesis.py", "--check"))


def check_linkage_stale() -> list[str]:
    return _errors("linkage_stale", _run_script_check("build_replay_linkage.py", "--check"))


def check_linkage_graph() -> list[str]:
    path = FIXTURES_SA / "synthesis/replay_linkage_index_v1.json"
    if not path.is_file():
        return _errors("linkage_graph", [f"missing {path}"])
    linkage = lib.load_json(path)
    node_ids = {str(n["sweep_id"]) for n in linkage.get("nodes") or [] if n.get("sweep_id")}
    issues: list[str] = []
    for edge in linkage.get("edges") or []:
        src = edge.get("source")
        tgt = edge.get("target")
        if src not in node_ids:
            issues.append(f"edge source not in nodes: {src}")
        if tgt not in node_ids:
            issues.append(f"edge target not in nodes: {tgt}")
        if not edge.get("edge_id"):
            issues.append(f"edge missing edge_id: {edge}")
        if not edge.get("link_kind"):
            issues.append(f"edge missing link_kind: {edge.get('edge_id')}")
    return _errors("linkage_graph", issues)


def _parse_target_url(url: str) -> dict[str, str]:
    if "?" not in url:
        return {}
    query = url.split("?", 1)[1]
    parsed = parse_qs(query, keep_blank_values=True)
    return {k: (v[0] if v else "") for k, v in parsed.items()}


def check_storyboard_urls() -> list[str]:
    issues: list[str] = []
    pack_ids = lib.catalog_pack_ids()
    sweep_ids = lib.sweep_ids_from_index() | set(SWEEP_IDS)
    sb_ids = lib.storyboard_ids()

    for fix_path, _pub_path in lib.iter_presentation_json_pairs():
        if fix_path.name == "index.json":
            continue
        file_id = fix_path.stem
        if file_id not in sb_ids:
            issues.append(f"storyboard file {fix_path.name} not listed in presentations/index.json")

    for sb_id in sb_ids:
        if not (lib.FIXTURES_SA / "presentations" / f"{sb_id}.json").is_file():
            issues.append(f"presentations/index.json lists missing storyboard file {sb_id}.json")

    for fix_path, _pub_path in lib.iter_presentation_json_pairs():
        if fix_path.name == "index.json":
            continue
        sb = lib.load_json(fix_path)
        for scene in sb.get("scenes") or []:
            url = str(scene.get("target_url") or "")
            params = _parse_target_url(url)
            demo = params.get("demo")
            if demo and demo not in pack_ids:
                issues.append(f"{fix_path.name}: unknown demo pack_id '{demo}' in {url}")
            sweep = params.get("sweep")
            if sweep and sweep not in sweep_ids:
                issues.append(f"{fix_path.name}: unknown sweep_id '{sweep}' in {url}")
            presentation = params.get("presentation")
            if presentation and presentation not in sb_ids:
                issues.append(f"{fix_path.name}: unknown presentation '{presentation}' in {url}")

    return _errors("storyboard_urls", issues)


def check_pattern_taxonomy() -> list[str]:
    issues: list[str] = []
    warnings: list[str] = []
    allowed = set(PATTERN_PRIORITY)

    for sweep_json in sorted((FIXTURES_SA / "sweeps").glob("*/sweep.json")):
        manifest = lib.load_json(sweep_json)
        sid = manifest.get("sweep_id", sweep_json.parent.name)
        baseline = str(manifest.get("baseline_topology_key") or "")
        members = manifest.get("members") or []
        bundle_paths = [
            sweep_json.parent / "members" / str(m["member_id"]) / "index.json"
            for m in members
        ]
        bundles = []
        for bp in bundle_paths:
            if not bp.is_file():
                issues.append(f"{sid}: missing member bundle {bp}")
                continue
            bundles.append(load_bundle(bp))

        if len(bundles) != len(members):
            continue

        fds = [_first_detection_t(b) for b in bundles]
        valid_fds = [f for f in fds if f is not None]
        median_fd = float(statistics.median(valid_fds)) if valid_fds else None
        sens = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "topology_sensitivity", {}
        )
        has_sensitivity = bool(sens.get("counts") and max(sens["counts"]) > 0)

        for m, b in zip(members, bundles):
            tags = m.get("replay_pattern_tags") or []
            for tag in tags:
                if tag not in allowed:
                    issues.append(f"{sid}/{m.get('member_id')}: unknown pattern_id '{tag}'")
            expected = classify_member_patterns(
                b,
                member=m,
                baseline_topology_key=baseline,
                sweep_median_fd=median_fd,
                has_sensitivity=has_sensitivity,
            )
            if set(tags) != set(expected):
                warnings.append(
                    f"{sid}/{m.get('member_id')}: tags {tags} != recompute {expected}"
                )

    out = _errors("pattern_taxonomy", issues)
    for w in warnings:
        print(f"audit_sa_platform_integrity: warning [pattern_taxonomy] {w}", file=sys.stderr)
    return out


def check_narrative_duplicates() -> list[str]:
    """Warn on repeated template bullets/caveats (stderr); fail only on empty summaries."""
    issues: list[str] = []
    bullet_counter: Counter[str] = Counter()
    caveat_counter: Counter[str] = Counter()
    sweep_count = 0

    for sweep_json in sorted((FIXTURES_SA / "sweeps").glob("*/sweep.json")):
        sweep_count += 1
        manifest = lib.load_json(sweep_json)
        summary = manifest.get("replay_narrative_summary") or {}
        bullets = summary.get("bullets") or []
        if not bullets:
            issues.append(f"{manifest.get('sweep_id')}: empty replay_narrative_summary.bullets")
        for bullet in bullets:
            text = str(bullet).strip()
            if text:
                bullet_counter[text] += 1
        for caveat in manifest.get("interpretation_caveats") or []:
            text = str(caveat).strip()
            if text:
                caveat_counter[text] += 1
        gov = manifest.get("governance") or {}
        notice = str(gov.get("notice") or "").strip()
        if notice:
            caveat_counter[notice] += 1

    for text, count in bullet_counter.items():
        if count >= DUPLICATE_BULLET_MIN_COUNT:
            print(
                f"audit_sa_platform_integrity: warning [narrative_duplicates] "
                f"shared bullet ({count}/{sweep_count} sweeps): {text[:80]}...",
                file=sys.stderr,
            )
    for text, count in caveat_counter.items():
        if count >= DUPLICATE_CAVEAT_MIN_COUNT:
            print(
                f"audit_sa_platform_integrity: warning [narrative_duplicates] "
                f"repeated caveat ({count}x): {text[:80]}...",
                file=sys.stderr,
            )

    return _errors("narrative_duplicates", issues)


def check_sweep_reports() -> list[str]:
    issues: list[str] = []
    for sweep_id in SWEEP_IDS:
        issues.extend(
            _run_script_check(
                "export_replay_analytics_report.py",
                "--sweep",
                sweep_id,
                "--check",
            )
        )
    return _errors("sweep_reports", issues)


def check_research_bundle() -> list[str]:
    return _errors(
        "research_bundle",
        _run_script_check("export_research_bundle.py", "--check"),
    )


def check_corpus_index_stale() -> list[str]:
    return _errors(
        "corpus_index_stale",
        _run_script_check("build_replay_corpus_index.py", "--check"),
    )


def check_corpus_lineage() -> list[str]:
    return _errors(
        "corpus_lineage",
        _run_script_check("audit_replay_lineage.py"),
    )


def check_corpus_release_stale() -> list[str]:
    return _errors(
        "corpus_release_stale",
        _run_script_check("build_replay_corpus_release.py", "--check"),
    )


def check_corpus_drift_stale() -> list[str]:
    return _errors(
        "corpus_drift_stale",
        _run_script_check("build_replay_corpus_drift_report.py", "--check"),
    )


def check_corpus_provenance() -> list[str]:
    return _errors(
        "corpus_provenance",
        _run_script_check("audit_replay_corpus_provenance.py"),
    )


def check_corpus_release_diff() -> list[str]:
    return _errors(
        "corpus_release_diff",
        _run_script_check("diff_replay_corpus_releases.py", "--check"),
    )


def check_corpus_viewer_audit_mirror() -> list[str]:
    issues: list[str] = []
    pairs = [
        (
            FIXTURES_SA / "corpus_audits/replay_corpus_drift_report_v1.json",
            PUBLIC_DEMO / "corpus_audits/replay_corpus_drift_report_v1.json",
        ),
        (
            FIXTURES_SA / "corpus_releases/sa_r0_corpus_r1_r1/replay_corpus_release_manifest_v1.json",
            PUBLIC_DEMO / "corpus_releases/sa_r0_corpus_r1_r1/replay_corpus_release_manifest_v1.json",
        ),
        (
            FIXTURES_SA / "synthesis/replay_corpus_evolution_manifest_v1.json",
            PUBLIC_DEMO / "synthesis/replay_corpus_evolution_manifest_v1.json",
        ),
        (
            FIXTURES_SA / "synthesis/replay_corpus_evolution_summary_v1.json",
            PUBLIC_DEMO / "synthesis/replay_corpus_evolution_summary_v1.json",
        ),
        (
            FIXTURES_SA / "synthesis/replay_corpus_publication_packet_v1.json",
            PUBLIC_DEMO / "synthesis/replay_corpus_publication_packet_v1.json",
        ),
    ]
    for canonical, mirror in pairs:
        if not canonical.is_file():
            issues.append(f"missing canonical: {canonical}")
            continue
        if not mirror.is_file():
            issues.append(f"missing viewer mirror: {mirror}")
            continue
        if canonical.read_bytes() != mirror.read_bytes():
            issues.append(f"viewer mirror stale: {mirror.relative_to(_REPO)}")
    return _errors("corpus_viewer_audit_mirror", issues)


def check_corpus_evolution_stale() -> list[str]:
    return _errors(
        "corpus_evolution_stale",
        _run_script_check("build_replay_corpus_evolution.py", "--check"),
    )


def check_corpus_publication_stale() -> list[str]:
    return _errors(
        "corpus_publication_stale",
        _run_script_check("build_replay_corpus_publication.py", "--check"),
    )


def check_corpus_release_export() -> list[str]:
    return _errors(
        "corpus_release_export",
        _run_script_check("export_replay_corpus_release.py", "--check"),
    )


def check_federation_index_stale() -> list[str]:
    return _errors(
        "federation_index_stale",
        _run_script_check("build_replay_federation_index.py", "--check"),
    )


def check_federation_integrity() -> list[str]:
    return _errors(
        "federation_integrity",
        _run_script_check("audit_replay_federation_integrity.py", "--check", "--strict"),
    )


def check_federation_viewer_mirror() -> list[str]:
    issues: list[str] = []
    fed = FIXTURES_SA / "federation"
    demo = PUBLIC_DEMO / "federation"
    names = (
        "replay_federation_manifest_v1.json",
        "replay_federation_index_v1.json",
        "replay_federation_lineage_graph_v1.json",
        "replay_federation_continuity_index_v1.json",
        "replay_federation_publication_collection_v1.json",
        "replay_federation_replay_summary_v1.json",
        "replay_federation_reproducibility_v1.json",
        "replay_federation_snapshot_v1.json",
    )
    for name in names:
        canonical = fed / name
        mirror = demo / name
        if not canonical.is_file():
            issues.append(f"missing canonical: {canonical}")
            continue
        if not mirror.is_file():
            issues.append(f"missing viewer mirror: {mirror}")
            continue
        if canonical.read_bytes() != mirror.read_bytes():
            issues.append(f"viewer mirror stale: {mirror.relative_to(_REPO)}")
    audit_pairs = (
        (
            fed / "audits/replay_federation_integrity_report_v1.json",
            demo / "audits/replay_federation_integrity_report_v1.json",
        ),
        (
            fed / "audits/orchestration_federation_recovery_continuity_v1.json",
            demo / "audits/orchestration_federation_recovery_continuity_v1.json",
        ),
    )
    for canonical, mirror in audit_pairs:
        if not canonical.is_file():
            issues.append(f"missing canonical: {canonical}")
            continue
        if not mirror.is_file():
            issues.append(f"missing viewer mirror: {mirror}")
            continue
        if canonical.read_bytes() != mirror.read_bytes():
            issues.append(f"viewer mirror stale: {mirror.relative_to(_REPO)}")
    return _errors("federation_viewer_mirror", issues)


def check_federation_recovery_continuity() -> list[str]:
    return _errors(
        "federation_recovery_continuity",
        _run_script_check("audit_federation_recovery_continuity.py", "--check", "--strict"),
    )


def check_bundle_catalog() -> list[str]:
    issues: list[str] = []
    catalog = lib.load_json(SCENARIOS / "index.json")
    for pack in catalog.get("packs") or []:
        pack_id = pack.get("pack_id")
        if not pack_id:
            continue
        pack_dir = SCENARIOS / str(pack_id)
        if not pack_dir.is_dir():
            issues.append(f"catalog pack_id missing scenario dir: {pack_dir}")
        demo_rel = pack.get("demo_bundle")
        if demo_rel:
            demo_path = _REPO / str(demo_rel)
            if not demo_path.is_file():
                issues.append(f"catalog demo_bundle missing: {demo_path}")

    for demo_dir in sorted(FIXTURES_SA.glob("demo_*")):
        index = demo_dir / "index.json"
        if not index.is_file():
            issues.append(f"missing demo bundle index: {index}")

    for sweep_json in sorted((FIXTURES_SA / "sweeps").glob("*/sweep.json")):
        manifest = lib.load_json(sweep_json)
        for m in manifest.get("members") or []:
            mid = m.get("member_id")
            if not mid:
                continue
            member_index = sweep_json.parent / "members" / str(mid) / "index.json"
            if not member_index.is_file():
                issues.append(f"{manifest.get('sweep_id')}: missing {member_index}")

    return _errors("bundle_catalog", issues)


def check_governance_batch() -> list[str]:
    return _errors("governance_batch", batch_lint_sa_fixtures())


CHECK_FUNCS = {
    "authoring_integrity": check_authoring_integrity,
    "orchestration_integrity": check_orchestration_integrity,
    "orchestration_async_integrity": check_orchestration_async_integrity,
    "orchestration_recovery_integrity": check_orchestration_recovery_integrity,
    "catalog_sync": check_catalog_sync,
    "fixture_pub_parity": check_fixture_pub_parity,
    "synthesis_stale": check_synthesis_stale,
    "linkage_stale": check_linkage_stale,
    "linkage_graph": check_linkage_graph,
    "storyboard_urls": check_storyboard_urls,
    "pattern_taxonomy": check_pattern_taxonomy,
    "narrative_duplicates": check_narrative_duplicates,
    "sweep_reports": check_sweep_reports,
    "research_bundle": check_research_bundle,
    "corpus_index_stale": check_corpus_index_stale,
    "corpus_lineage": check_corpus_lineage,
    "corpus_release_stale": check_corpus_release_stale,
    "corpus_drift_stale": check_corpus_drift_stale,
    "corpus_provenance": check_corpus_provenance,
    "corpus_release_diff": check_corpus_release_diff,
    "corpus_viewer_audit_mirror": check_corpus_viewer_audit_mirror,
    "corpus_evolution_stale": check_corpus_evolution_stale,
    "corpus_publication_stale": check_corpus_publication_stale,
    "corpus_release_export": check_corpus_release_export,
    "federation_index_stale": check_federation_index_stale,
    "federation_integrity": check_federation_integrity,
    "federation_viewer_mirror": check_federation_viewer_mirror,
    "federation_recovery_continuity": check_federation_recovery_continuity,
    "bundle_catalog": check_bundle_catalog,
    "governance_batch": check_governance_batch,
}


def run_checks(selected: tuple[str, ...]) -> list[str]:
    all_errors: list[str] = []
    for check_id in selected:
        fn = CHECK_FUNCS.get(check_id)
        if fn is None:
            all_errors.append(f"[unknown] {check_id}")
            continue
        all_errors.extend(fn())
    return all_errors


def main() -> None:
    ap = argparse.ArgumentParser(description="Audit SA platform fixture integrity")
    ap.add_argument("--all", action="store_true", help="run all checks")
    for check_id in CHECK_IDS:
        ap.add_argument(
            f"--{check_id.replace('_', '-')}",
            action="store_true",
            dest=check_id,
            help=f"run {check_id} check",
        )
    args = ap.parse_args()

    selected = list(CHECK_IDS) if args.all else [cid for cid in CHECK_IDS if getattr(args, cid)]

    if not selected:
        ap.print_help()
        raise SystemExit("specify --all or at least one --<check-id>")

    errors = run_checks(tuple(selected))
    if errors:
        print("audit_sa_platform_integrity FAILED:", file=sys.stderr)
        for err in errors:
            print(f"  {err}", file=sys.stderr)
        print(
            "Hints: sync_sa_catalog.py, gen_e1_presentation_fixtures.py, "
            "gen_e2_research_fixtures.py, gen_f1_corpus_fixtures.py, "
            "run_replay_corpus_regen.py, gen_d3_sweep_enrichment.py",
            file=sys.stderr,
        )
        raise SystemExit(1)
    print(f"audit_sa_platform_integrity OK ({len(selected)} checks)")


if __name__ == "__main__":
    main()
