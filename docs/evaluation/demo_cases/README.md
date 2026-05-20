# Replay demo showcase cases

Curated case briefs for [replay_demo_review_workflow_r1.md](../replay_demo_review_workflow_r1.md). These are **interpretation prompts only** — not runtime scenarios, not certified test vectors, and not authority definitions.

Each brief follows the same structure: what to demonstrate, what to look for, what not to conclude, recommended HTML sections/figures, and governance caveats.

When preparing a live demo, copy the case brief (or its `README_case.md` block) into `runs/evaluation/demo/<case_id>/README_case.md` after generating artifacts.

| Case ID | Brief file | Primary teaching goal |
|---------|------------|------------------------|
| `clean_hit` | [clean_hit.md](clean_hit.md) | Parser-visible success path |
| `near_miss` | [near_miss.md](near_miss.md) | Threshold sensitivity |
| `ambiguity_pressure` | [ambiguity_pressure.md](ambiguity_pressure.md) | Sensing/fusion stress |
| `selection_oracle_divergence` | [selection_oracle_divergence.md](selection_oracle_divergence.md) | Replay-local disagreement |
| `fragmentation_phase_pocket` | [fragmentation_phase_pocket.md](fragmentation_phase_pocket.md) | Cadence/phase sensitivity |
| `paired_seed_delta` | [paired_seed_delta.md](paired_seed_delta.md) | Matched-seed comparison |
| `topology_spread` | [topology_spread.md](topology_spread.md) | Seed spread under one profile |
| `governance_warnings` | [governance_warnings.md](governance_warnings.md) | Provenance and lint prompts |

## Generating artifacts

See the command block in [replay_demo_review_workflow_r1.md](../replay_demo_review_workflow_r1.md#example-commands). Fixture hints are listed per case file.

## Standard figure set

Most demos use the UX Refinement R2 composite output (`static_viz_ux_refinement_r2_v1`; see [replay_ux_refinement_r2_freeze_audit.md](../replay_ux_refinement_r2_freeze_audit.md)):

| Figure / section | Role |
|------------------|------|
| HTML banner + scan_guide | Framing |
| at_a_glance + summary cards | Outcome, near-miss margin, D-class, selection summary |
| incident_groups | Log localization (selection blocks collapsed; mismatches elevated) |
| comprehension_panel | Stacked overview |
| timeline_band | Event/window bands |
| divergence_overlay | Selection/oracle blocks |
| lifecycle_strip | Churn/gap localization |
| lineage appendix | Provenance |

| timeline (salient) | Prioritized events; overflow note for long logs |

Optional when `log_path` is readable: `engagement_series.png` (threshold line when log-evidenced), `sparse_topdown.png`.

Pass `--observability-json` to composite for richest selection mismatch counts in summary cards.
