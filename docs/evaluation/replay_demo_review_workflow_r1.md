# Replay Demo & Review Workflow R1

Phase name: **Replay Demo & Review Workflow R1**

Build recommendation: **documentation and curation only**. This wave does not authorize runtime, launch, topic, schema, parser-contract, or replay-builder changes.

`AGENTS.md` remains the primary governance authority. Frozen replay tooling contracts are defined in scoped freeze audits; see [freeze_registry_r1.md](freeze_registry_r1.md).

## Purpose

Answer: **“How should a human safely and correctly use this replay-analysis platform?”**

This workflow is for mentors, reviewers, and demo presenters. It does **not** define a battle-management UI, live operator surface, or readiness certification path.

## Frozen replay toolchain

Do not reopen Comprehension R1 or observability builder contracts for demos. Use the existing pipeline:

```
raw log + .meta.json
  → replay_observability.py (single-run-report, narrative, optional paired-comparison)
  → replay_static_visualization.py composite
  → replay_static_visualization.html + PNG figures + manifest
```

| Step | Command | Primary artifact | Reviewer focus |
|------|---------|------------------|----------------|
| 1 | `single-run-report` | `*.replay_observability.json` | Provenance, divergence trace, lifecycle timeline |
| 2 | `narrative` | `*.replay_narrative.json` | Sequence, incidents (non-causal) |
| 3 | `composite` | `replay_viz/` directory | 30-second scan via comprehension digest |
| 4 (optional) | `paired-comparison` | matched-seed JSON | A vs B localization only |
| 5 (before external share) | `governance-lint` | lint JSON | Forbidden-claim check |
| 6 (optional) | `replay_sa_bundle.py pack` + SA-R0 viewer | `replay_sa_bundle_v1` / spatial replay | Mentor spatial orientation only — not primary surface |

### Example commands

From the repository root, with `LOG` and `META` pointing at a captured run:

```bash
python3 scripts/evaluation/replay_observability.py single-run-report \
  "$LOG" \
  --meta "$META" \
  --out-json runs/evaluation/demo/RUN.replay_observability.json

python3 scripts/evaluation/replay_observability.py narrative \
  --single-run-json runs/evaluation/demo/RUN.replay_observability.json \
  --out-json runs/evaluation/demo/RUN.replay_narrative.json

python3 scripts/evaluation/replay_static_visualization.py composite \
  --narrative-json runs/evaluation/demo/RUN.replay_narrative.json \
  --observability-json runs/evaluation/demo/RUN.replay_observability.json \
  --out-dir runs/evaluation/demo/RUN.replay_viz/
```

Optional lint before sharing:

```bash
python3 scripts/evaluation/replay_observability.py governance-lint \
  runs/evaluation/demo/RUN.replay_observability.json
```

Open the primary demo surface locally:

```bash
xdg-open runs/evaluation/demo/RUN.replay_viz/replay_static_visualization.html
```

Optional SA-R0 spatial replay (read-only; not a replacement for comprehension HTML):

```bash
python3 scripts/evaluation/replay_sa_bundle.py pack \
  --narrative-json runs/evaluation/demo/RUN.replay_narrative.json \
  --observability-json runs/evaluation/demo/RUN.replay_observability.json \
  --viz-manifest runs/evaluation/demo/RUN.replay_viz/replay_static_visualization.json \
  --scenario-pack fixtures/scenarios/ridge_defense \
  --out-dir runs/evaluation/demo/RUN.sa_bundle/

python3 scripts/evaluation/replay_sa_bundle.py export-portable \
  --bundle-dir runs/evaluation/demo/RUN.sa_bundle/ \
  --out-zip runs/evaluation/demo/RUN.sa_bundle.zip

cd platform/sa-r0-viewer && npm run dev
# open http://localhost:5173 — use scenario catalog picker or ?demo=<pack_id>
# sync catalog + demo bundles: python3 scripts/evaluation/sync_sa_catalog.py
```

## Recommended replay reading order

Read artifacts in **evidence-layer order**, not HTML layout order alone:

| Order | Artifact / section | Layer | Why read it |
|-------|-------------------|-------|-------------|
| 1 | Sidecar `.meta.json` | raw / provenance | Seed, cohort, launch args, git state |
| 2 | `*.replay_observability.json` → `bundle.lineage`, warnings | derived | Completeness before interpretation |
| 3 | Parser-visible summary inside observability | parser-visible | Stable outcome fields (`success`, `min_miss`, etc.) |
| 4 | `*.replay_narrative.json` → events, windows | derived, non-causal | Sequence context only |
| 5 | `replay_static_visualization.html` → banner, scan guide | explanatory viz | Framing before figures |
| 6 | HTML → at_a_glance, incident_groups | explanatory viz | Where to look in the log |
| 7 | Figures: comprehension_panel → timeline_band → divergence_overlay → lifecycle_strip | explanatory viz | Spatial/temporal localization |
| 8 | HTML → lineage appendix | provenance | Final shareability check |
| 9 | Optional: `governance-lint` JSON | lint | Wording presence, not approval |

Layer definitions: [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md).

## Thirty-second replay review script

Use when opening `replay_static_visualization.html` for mentor walkthroughs or quick peer review:

1. Read the **banner** and **scan_guide** — derived replay summary only; not authority.
2. Scan **at_a_glance** — outcome labels, divergence class, ambiguity class (taxonomy only).
3. Read **incident_groups** — where to look in the log (block indices / windows).
4. Open **comprehension_panel** — stacked timeline, divergence overlay, lifecycle strip.
5. Check **lineage** — log path, seed, `profile_id`, git state, warnings.
6. Close with explicit negation: not readiness, not causal proof, not a unified replay state.

## Fifteen-minute reviewer onboarding path

Single-session path for a new reviewer with no prior replay tooling context:

| Minutes | Activity | Document / artifact |
|---------|----------|-------------------|
| 0–3 | Learn evidence layers | [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) § Layer Map |
| 3–5 | Learn forbidden wording | Same guide § Causal-Language Boundary + § Reviewer Checklist |
| 5–8 | Walk thirty-second script on `clean_hit` | [demo_cases/clean_hit.md](demo_cases/clean_hit.md) + generated HTML |
| 8–11 | Contrast with `selection_oracle_divergence` or `governance_warnings` | Matching [demo_cases/](demo_cases/) brief |
| 11–13 | Run `governance-lint` on observability JSON | This doc § External sharing checklist |
| 13–15 | Practice one forbidden-conclusion correction aloud | This doc § Forbidden phrases |

Extended onboarding (three sessions, ~45 minutes): repeat with `paired_seed_delta`, `ambiguity_pressure`, and `fragmentation_phase_pocket` using the mentor sequence below.

## Fifteen-to-twenty-minute mentor presentation sequence

| Minutes | Section | Action |
|---------|---------|--------|
| 0–2 | Layers | Point reviewers to [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) layer table |
| 2–5 | Clean hit case | Walk 30-second script on a baseline success run |
| 5–9 | Divergence or ambiguity case | Show localization-only language; lifecycle dormancy caveat if counters are zero |
| 9–12 | Near-miss or phase-pocket case | Threshold / cadence sensitivity without “failure drama” |
| 12–15 | Optional comparison | Same-seed paired profile; what not to conclude |
| 15–18 | Governance | Run `governance-lint`; read warnings panel as prompts, not severity |
| 18–20 | Q&A anchors | Re-state forbidden claims list below |

## SA-R0 presentation mode (PLAT-SA-E1)

Machine-readable storyboard decks live under `fixtures/sa_r0/presentations/` (synced to viewer `public/demo/presentations/`).

Example viewer URLs (static demo server):

- `?demo=valley_ingress&walkthrough=1&chapter=0` — bundle chapter walkthrough
- `?presentation=walkthrough_valley_ingress_long&demo=valley_ingress&chapter=0` — storyboard deck
- `?sweep=ridge_overlap_sweep&walkthrough=1` — sweep member with presentation chapters

Storyboard index: `/demo/presentations/index.json`. Review exports: `fixtures/sa_r0/sweeps/<id>/reports/sweep_presentation_packet.md`.

See [sa_e1_research_presentation_plan.md](sa_e1_research_presentation_plan.md) and [replay_storyboard_v1.md](replay_storyboard_v1.md).

## Cross-sweep synthesis and publication (PLAT-SA-E2)

Corpus rollups live under `fixtures/sa_r0/synthesis/` (synced to viewer `public/demo/synthesis/`).

Regenerate:

```bash
python3 scripts/evaluation/gen_e2_research_fixtures.py
```

Key artifacts:

- `cross_sweep_synthesis_v1.json` — pattern/ambiguity/topology/LOS/divergence rollups
- `replay_linkage_index_v1.json` — rule-based sweep linkage
- `cognition_rollup_summary.md` — reviewer cognition bullets
- `fixtures/sa_r0/sweeps/<id>/reports/publication_packet.html` — print-ready review packet
- `fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/` — portable research bundle

Storyboard: `showcase_cross_sweep_synthesis` in `/demo/presentations/index.json`.

See [sa_e2_replay_knowledge_synthesis_plan.md](sa_e2_replay_knowledge_synthesis_plan.md) and [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md).

## Replay comparison dos and don'ts

### Same seed, different profile

Use `paired-comparison` when two runs share explicit `noise_seed` lineage.

- **Do:** describe divergence timing, lifecycle churn, or outcome label differences.
- **Do:** confirm seed and profile id appear in lineage for both sides.
- **Don't:** declare a winner, rank profiles, or infer robustness proof.

### Same profile, different seed

Use topology-index or cohort summaries to describe spread.

- **Do:** describe label distribution across seeds under one profile.
- **Don't:** rank seeds as “better system” behavior or certify an operating region.

### Never in comparisons

- Side-by-side “winner” framing
- Hit-rate league tables as robustness proof
- Tactical performance or field-effectiveness claims
- Causal mechanism language (“X caused the miss”)

Case briefs: [demo_cases/paired_seed_delta.md](demo_cases/paired_seed_delta.md), [demo_cases/topology_spread.md](demo_cases/topology_spread.md).

## Artifact packaging convention

Store demo bundles under `runs/evaluation/demo/<case_id>/`. Bulk logs remain gitignored per repository hygiene.

```
runs/evaluation/demo/<case_id>/
  <run_id>.log                          # optional local copy; prefer runs/logs/ reference
  <run_id>.meta.json
  <run_id>.replay_observability.json
  <run_id>.replay_narrative.json
  <run_id>.replay_viz/
    replay_static_visualization.html    # primary demo surface
    replay_static_visualization.json    # manifest + comprehension
    comprehension_panel.png
    timeline_band.png
    divergence_overlay.png
    lifecycle_strip.png
    (optional) engagement_series.png
    (optional) sparse_topdown.png
  README_case.md                        # copy from docs/evaluation/demo_cases/
```

### Naming conventions

| Pattern | Meaning |
|---------|---------|
| `<run_id>.log` / `<run_id>.meta.json` | Capture sidecar pair from `run_capture.py` |
| `<run_id>.replay_observability.json` | Output of `single-run-report` |
| `<run_id>.replay_narrative.json` | Output of `narrative` |
| `<run_id>.replay_viz/` | Output of `composite` |
| `demo/<case_id>/` | Curated mentor bundle keyed to showcase case |

Use the capture `run_id` as the stem. Do not rename parser-visible fields inside JSON.

### Lineage expectations

Every shareable bundle must preserve or reference:

- `log_path`, `meta_path` (or explicit “unavailable” warnings)
- seed source and cohort when present in meta notes
- git commit and dirty flag when present
- launch args / profile id for profile-driven runs
- warnings array — never strip to “clean up” a demo

Missing lineage → stop interpretation; use [demo_cases/governance_warnings.md](demo_cases/governance_warnings.md).

### Replay artifact sharing guidance

- Share HTML + manifest JSON + brief `README_case.md`; avoid shipping raw logs externally unless policy allows.
- Redact local usernames from paths if required.
- Include link to [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md).
- State explicitly: derived, non-authoritative, not operational readiness.
- Run `governance-lint` before external send.

## Replay review ergonomics

Practices that reduce misinterpretation during live review:

- **One screen, one layer:** keep parser-visible summary visible separately from HTML when debating outcomes.
- **Lineage first:** open meta sidecar before figures when seed or profile disputes arise.
- **Say the negation:** end every demo segment with what the artifact is *not*.
- **Dormancy explicit:** if lifecycle counters are zero, say it aloud — do not skip the strip.
- **No live refresh:** static HTML is the demo surface; do not imply real-time authority.
- **Compare with matched seed only:** if comparing two runs, confirm seed in lineage before showing divergence deltas.
- **Lint, don't score:** treat `governance-lint ok: true` as wording check passed, not approval.

## Showcase case catalog

| Case ID | Teaches | Brief |
|---------|---------|-------|
| `clean_hit` | Parser-visible success path | [clean_hit.md](demo_cases/clean_hit.md) |
| `near_miss` | Threshold sensitivity | [near_miss.md](demo_cases/near_miss.md) |
| `ambiguity_pressure` | Sensing/fusion stress | [ambiguity_pressure.md](demo_cases/ambiguity_pressure.md) |
| `selection_oracle_divergence` | Replay-local disagreement | [selection_oracle_divergence.md](demo_cases/selection_oracle_divergence.md) |
| `fragmentation_phase_pocket` | Cadence/phase sensitivity | [fragmentation_phase_pocket.md](demo_cases/fragmentation_phase_pocket.md) |
| `paired_seed_delta` | Matched-seed comparability | [paired_seed_delta.md](demo_cases/paired_seed_delta.md) |
| `topology_spread` | Profile/geometry seed spread | [topology_spread.md](demo_cases/topology_spread.md) |
| `governance_warnings` | Provenance and lint prompts | [governance_warnings.md](demo_cases/governance_warnings.md) |

Index: [demo_cases/README.md](demo_cases/README.md).

## Governance checklist for external sharing

Before sending HTML, JSON, or dashboard exports outside the team:

- [ ] Confirm artifact type and evidence layer with the recipient
- [ ] Run `governance-lint` on observability or narrative JSON
- [ ] Verify log path, meta path, seed, cohort, git state, and warnings are visible in lineage
- [ ] Remove or redact paths that leak local usernames if required by policy
- [ ] Include link to [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)
- [ ] State explicitly: not operational readiness, not certification, not HITL approval
- [ ] Avoid hit-rate summaries without matched-seed context and dormancy caveats

### Forbidden phrases in demo narration

Do not use: “validated,” “certified,” “field-ready,” “proven robust,” “the system failed because,” “operator should,” “engage now,” “battle manager,” “readiness score,” “winner,” “best profile.”

Prefer: “associated with,” “localized near,” “replay-local,” “derived summary,” “parser-visible,” “explanatory evidence.”

## Interpretation anchors

- Comprehension copy: frozen `scan_guide` in manifest (`render_profile: static_viz_comprehension_r1_v1`)
- Wave 7 divergence fields: [README.md](../../README.md) frozen field list
- Lifecycle dormancy: [scripts/evaluation/README.md](../../scripts/evaluation/README.md) Wave 3+ cautions
- Layer definitions: [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)

## Scope boundaries

### Allowed

- Demo runbook and case catalog documentation
- Local demo bundles under `runs/evaluation/demo/`
- Mentor scripts and onboarding outlines
- Pointers to existing fixture CSVs and evaluation commands

### Forbidden

- New CLIs, live UI, CI publishing of HTML as gates
- Rosbag trajectory overlays, runtime automation for demos
- Operational framing, readiness claims, HITL semantics
- Edits to frozen `replay_observability.py` narrative builders
- Composite robustness or readiness scoring

## SA-R0 sweep review (PLAT-SA-D3, additive)

For Monte Carlo sweep families under `fixtures/sa_r0/sweeps/`, use the SA-R0 viewer workstation and static exports:

```bash
python3 scripts/evaluation/gen_d3_sweep_enrichment.py
python3 scripts/evaluation/export_replay_analytics_report.py --sweep ridge_overlap_sweep --format md,json
```

Viewer URLs: `?sweep=<id>&member=<n>`, `?sweep=<id>&filmstrip=0,1,2,3`, or `?sweep=<id>&cohort=<cohort_id>`.

Review artifacts in `fixtures/sa_r0/sweeps/<id>/reports/` include `sweep_narrative_summary.md`, `replay_cluster_report.md`, and `replay_review_report_v1.json`. Explanatory replay reasoning only — not operational guidance.

## Platform fixture maintenance (PLAT-SA-STAB)

Before sharing SA-R0 demo fixtures or merging platform changes, run the integrity gate:

```bash
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

If parity or staleness checks fail, regenerate with `sync_sa_catalog.py`, `gen_d3_sweep_enrichment.py`, `gen_e1_presentation_fixtures.py`, or `gen_e2_research_fixtures.py` as indicated in audit stderr. See [sa_stabilization_plan.md](sa_stabilization_plan.md).

## Related documents

- [freeze_registry_r1.md](freeze_registry_r1.md) — frozen layer index
- [sa_d3_replay_narrative_intelligence_plan.md](sa_d3_replay_narrative_intelligence_plan.md) — D3 workstation wave
- [replay_static_visualization_comprehension_r1_freeze_audit.md](replay_static_visualization_comprehension_r1_freeze_audit.md)
- [replay_observability_freeze_audit.md](replay_observability_freeze_audit.md)
- [situational_awareness_ui_planning_r1.md](situational_awareness_ui_planning_r1.md) — SA UI architecture plan (docs frozen; not demo implementation)

## Freeze status

Replay Demo & Review Workflow R1 is **documentation-only** and does not introduce a new runtime or JSON schema freeze. Tooling behavior remains governed by prior replay observability and static visualization freeze audits.

**Freeze-readiness recommendation:** freeze as docs-only (EVAL-DEMO-R1) after markdown validation passes and registry row is confirmed in [freeze_registry_r1.md](freeze_registry_r1.md).
