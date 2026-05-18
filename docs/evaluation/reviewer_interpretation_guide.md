# Reviewer Interpretation Guide

This guide is a reviewer-facing map for replay analysis, evaluation artifacts, and static reports. It is documentation only. It does not redefine runtime topics, parser-visible fields, tactical authority, lifecycle semantics, or freeze boundaries.

`AGENTS.md` remains the primary governance authority for repository direction. This guide only clarifies how to read existing evidence-side artifacts without collapsing them into a single authority layer.

## Layer Map

| Layer | Role | Examples | Correct Reading | Common Misread |
|---|---|---|---|---|
| Authoritative state | Runtime or parser surface that a scoped tool is allowed to rely on for a specific purpose. | `/interceptor/selected_id` for tactical replay tooling; `parse_run_to_result` fields for aggregate parser summaries. | Authority is local to the documented consumer and scope. | Treating all replay outputs as authoritative state. |
| Mirrored state | A copied or summarized view of another source. | Committed snapshots of generated aggregates; report cards that repeat lineage or summary values. | Mirrors improve reviewability but do not become the source of truth. | Treating a mirror as stronger evidence than the source artifact. |
| Explanatory evidence | Raw or lightly structured evidence that helps reviewers understand a run. | Logs, sidecar `.meta.json`, `[TACTICAL_*]`, `[REALISM_EVENT]`, lifecycle/churn log lines. | Evidence can explain context while remaining non-authoritative. | Treating log ordering, absence, or cadence as tactical truth or causal proof. |
| Derived interpretation | Evaluation-side analysis layered on top of existing evidence. | Evaluation rows, divergence taxonomies, matched-seed buckets, lifecycle/churn summaries, topology indexes. | Derived outputs classify or localize replay evidence. | Treating labels as runtime state, rankings, readiness, or robustness proof. |
| Replay artifacts | Static reviewer conveniences generated from existing evidence and derived interpretation. | Replay observability bundles, static reports, governance lint JSON, static dashboards, Phase 3 replay narrative validation review notes. | Artifacts preserve lineage and make review easier. | Treating a report, dashboard, validation note, or lint `ok` as approval, certification, or a unified replay state. |

## Artifact Reading Rules

- Raw logs and sidecar metadata preserve provenance and context; they are not parser contracts.
- `parse_run_to_result` summaries are the stable parser-visible surface for aggregate statistics; they do not absorb every evaluation-side derived field.
- Evaluation rows and replay observability JSON are derived artifacts; they may include useful classifications but do not create new runtime authority.
- Static reports and dashboards are explanatory visualization layers; they are reviewer conveniences, not live dashboards, operational displays, or readiness evidence.
- Governance lint checks caveat and anti-claim presence; `ok: true` does not certify correctness, validity, comparability, or approval.

## Glossary Of Loaded Terms

- `authority`: A scoped source that a specific consumer may rely on for a documented purpose. Authority does not transfer automatically to mirrors, dashboards, or derived artifacts.
- `truth`: Use only with a qualifier, such as Gazebo source truth or parser-local `[HIT]` truth. Do not use it to imply operational truth, tactical certainty, or causal proof from replay artifacts.
- `parser-visible`: Part of the stable parser summary surface. Parser-visible fields must not be renamed or reinterpreted by reviewer hardening.
- `derived`: Computed after capture by evaluation-side tooling. Derived outputs can support review but cannot replace runtime or parser authority.
- `explanatory`: Helpful for understanding evidence, not authoritative by itself.
- `provenance` / `lineage`: Source paths, seed source, cohort, git state, launch arguments, and warnings that help reviewers trace evidence. Lineage does not certify validity or comparability.
- `validation`: Use for bounded regression evidence or surrogate agreement only when the assumptions are visible. In Phase 3 replay narrative validation, use the term only for reviewer-interpretation validation of static narratives. Do not translate validation wording into certification, readiness, runtime correctness, authority, or field performance claims.
- `dashboard`: A static explanatory visualization layer unless explicitly governed otherwise. In this repository it is not live, operational, or authoritative.
- `robustness`: A research goal and evaluation theme, not proof from a single replay artifact, lifecycle counter, or dashboard table.
- `readiness` / `certification`: Out of scope for reviewer artifacts. Replay analysis must not imply operational readiness, hardware readiness, HITL approval, or certification.

## Causal-Language Boundary

Replay analysis may show temporal adjacency, co-occurrence, localization, or association. Prefer wording such as:

- `associated with`
- `co-occurs with`
- `localized near`
- `adjacent to`
- `consistent with`

Avoid wording that implies causality, mechanism, operational truth, engagement certainty, or proof unless a separate governed analysis and authoritative evidence contract explicitly supports it.

## Reviewer Checklist

Before sharing or interpreting reviewer artifacts:

- Confirm the artifact type and layer before reading conclusions.
- Confirm log path, metadata path, seed source, cohort, git state, launch arguments, and warnings.
- Keep parser-visible summaries separate from evaluation rows and replay observability artifacts.
- Read divergence labels as replay-local taxonomy, not tactical authority or causal mechanism.
- Read matched-seed buckets as descriptive comparability aids, not statistical superiority or rankings.
- Read lifecycle/churn counters as explanatory overlays over raw log lines, not tracker lifecycle truth or robustness proof.
- Read topology/profile labels as lineage shorthand, not certified operating regions.
- Treat warning panels as interpretation prompts, not severity, readiness, governance approval, or operational status.
- Read Phase 3 replay narrative validation findings as qualitative reviewer interpretation notes, not scores, approvals, runtime findings, or causal explanations.
