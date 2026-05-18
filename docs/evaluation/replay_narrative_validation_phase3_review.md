# PHASE 3 Replay Narrative Validation Review

## Scope

`AGENTS.md` remains the primary authority. This review implements the Phase 3 documentation waves for static replay narrative validation as reviewer-interpretation validation only.

This review is replay-side, static, deterministic, additive, and non-authoritative. It prefers review and freeze stabilization over feature expansion. Narrative summaries remain sequence-oriented and evidence-oriented, not causal-assertive.

This review does not authorize runtime changes, parser/schema changes, live dashboards, operator semantics, readiness/scoring semantics, causal AI explanation systems, governance automation expansion, interactive frontend architecture, live replay systems, or operational visualization flows.

Reference boundaries:

- [PHASE 3 Replay Narrative Validation Plan](replay_narrative_validation_phase3_plan.md)
- [Reviewer Interpretation Guide](reviewer_interpretation_guide.md)
- [PHASE 2 Replay Narrative UX Plan](replay_narrative_ux_phase2_plan.md)
- [Replay Narrative UX Freeze Audit](replay_narrative_ux_freeze_audit.md)
- [Replay Observability Freeze Audit](replay_observability_freeze_audit.md)

## Wave 1: Static Case Matrix

The Phase 3 case matrix is a reviewer-coverage aid for interpretation failure modes. It is not coverage evidence, capability evidence, a campaign plan, or a readiness claim.

| Case ID | Replay Pattern | Static Source Slot | Reviewer Interpretation Focus | Selection Note |
|---|---|---|---|---|
| `P3-C1-low-ambiguity` | Low-ambiguity replay | Existing Phase 2 replay narrative JSON or single-run replay observability report | Baseline sequence readability and caveat visibility | Use when event order and source references are clear enough to test baseline comprehension. |
| `P3-C2-lifecycle-heavy` | Lifecycle/churn-heavy replay | Existing lifecycle timeline or replay narrative with lifecycle overlays | Whether lifecycle labels remain explanatory and non-authoritative | Use when lifecycle or churn labels are present without treating them as tracker lifecycle truth. |
| `P3-C3-divergence-window` | Divergence replay with mismatch windows | Existing divergence trace or replay narrative with mismatch events | Whether divergence is read as localization, not causality | Use when selected/oracle mismatch windows can be reviewed as replay-side association only. |
| `P3-C4-visibility-limited` | Visibility-limited replay | Existing D5 or warning-heavy replay observability artifact | Whether uncertainty remains visible without implying severity | Use when warnings or visibility limits need prominence without becoming operational status. |
| `P3-C5-missing-provenance` | Missing or partial lineage replay | Existing artifact with missing meta, seed, cohort, or git-state warning | Whether provenance gaps remain visible and bounded | Use when missing lineage is preserved as a limitation instead of smoothed into a clean narrative. |
| `P3-C6-fragmentation-ambiguity` | Fragmentation or ambiguity replay | Existing narrative with fragmented-gap, ghost, dropout, stale, or delay evidence | Whether ambiguity wording stays descriptive and non-causal | Use when adjacency and uncertainty need careful non-causal phrasing. |
| `P3-C7-maintenance-sentinel` | Repeated-label or repeated-caveat replay | Any existing narrative that stresses repeated taxonomy labels or caveats | Whether labels and caveats remain maintainable without drift | Use when repeated wording should point back to central guidance rather than duplicating caveats. |

Selection rules:

- Prefer existing Phase 2 narrative JSON, single-run replay observability reports, divergence traces, lifecycle timelines, and warning-heavy artifacts.
- Do not run new captures solely to fill the matrix.
- Preserve source artifact paths and warning text when concrete replay artifacts are selected later.
- Keep the initial reviewed set small enough for manual interpretation review.
- Do not infer that absent cases are unsupported, invalid, ready, certified, robust, or operationally acceptable.

## Wave 2: Reviewer Rubric

Reviewer findings are qualitative notes about interpretability. They are not scores, approvals, severity labels, readiness bands, certification claims, or runtime correctness findings.

| Dimension | Reviewer Question | Allowed Finding Language | Misread To Avoid |
|---|---|---|---|
| Sequence readability | Can the reviewer reconstruct replay sequence without reading every raw log line? | `clear sequence`, `sequence gap`, `ordering ambiguous` | Narrative order as causal proof. |
| Event grouping clarity | Are related events grouped without hiding source evidence? | `grouping clear`, `grouping overloaded`, `needs source split` | Grouped events as unified replay state. |
| Taxonomy consistency | Are event categories and labels stable across similar evidence? | `label consistent`, `label overloaded`, `taxonomy drift risk` | Labels as parser fields or runtime states. |
| Provenance visibility | Are source paths, warnings, seed/cohort lineage, and missing metadata visible? | `provenance clear`, `lineage incomplete`, `warning needs prominence` | Lineage as certification or comparability proof. |
| Caveat visibility | Are interpretation caveats visible without dominating the report? | `caveat visible`, `caveat buried`, `caveat duplicated` | Caveats as approval, severity, or operational status. |
| Ambiguity handling | Are ambiguity and visibility limits presented as uncertainty, not diagnosis? | `ambiguity bounded`, `uncertainty unclear`, `visibility limit hidden` | Ambiguity as causal diagnosis. |
| Authority-overread risk | Does wording preserve derived and non-authoritative status? | `boundary clear`, `authority overread risk`, `wording too strong` | Findings as tactical authority. |
| Maintainability | Can maintainers update labels without duplicating governance text? | `maintainable`, `repetition drift risk`, `needs central reference` | Maintainability feedback as runtime redesign mandate. |

Allowed reviewer-note examples:

- `The sequence is clear for reviewer reconstruction, with source references visible.`
- `The label is overloaded across cases and should be clarified before tooling expansion.`
- `The warning is visible but could be misread as severity; wording should emphasize interpretation prompt.`
- `The divergence description is localized in time but should avoid causal phrasing.`
- `The case is provenance-limited; the limitation is visible and should remain in the report.`

Forbidden reviewer-note patterns:

- `validated`, `approved`, `certified`, `ready`, `robust`, or `operationally acceptable`;
- numeric, color-coded, or ranked readiness scores;
- causal conclusions such as `fragmentation caused the miss`;
- parser or runtime prescriptions such as `promote this label to a parser field`;
- operator or HITL language such as `reviewer accepted the decision`.

## Wave 3: Static Validation Report Shape

If a later scoped wave implements generated Phase 3 reports, the report should remain a static explanatory reviewer artifact. This documented shape is not a schema commitment, renderer commitment, validator, parser contract, or approval record.

Candidate report sections:

- `purpose`: reviewer-interpretation validation only, with explicit non-authoritative scope.
- `source_narrative_artifacts`: static paths, artifact types, lineage status, warnings, and caveats copied from existing artifacts.
- `case_matrix`: selected Wave 1 case ids, replay pattern, source artifact, and selection rationale.
- `reviewer_interpretation_findings`: qualitative Wave 2 findings, not scores or approvals.
- `taxonomy_coherence_findings`: label consistency, overloaded labels, missing context, and maintainability notes.
- `semantic_drift_findings`: wording that risks causal, tactical, operational, readiness, or authority overread.
- `ambiguity_findings`: uncertainty, visibility limits, fragmentation adjacency, and provenance gaps.
- `maintainability_findings`: repeated caveats, duplicated language, or unclear terminology ownership.
- `warnings_and_caveats`: source warnings plus report-level interpretation caveats.

Required properties for any future static report:

- Use static file input and output only.
- Sort deterministically by case id, finding category, source artifact, and finding text.
- Avoid wall-clock timestamps, generated run ids, and live process links.
- Preserve source artifact paths, warnings, and missing-provenance notes.
- Keep findings textual and bounded; do not add readiness, certification, severity, approval, capability, or robustness fields.
- Do not link to runtime processes, websocket streams, ROS topics, operator actions, or interactive frontend flows.

## Wave 4: Taxonomy And Drift Review

This documentation-only review compares the Phase 3 case slots against the Phase 2 replay narrative boundaries. It identifies wording and maintenance risks only; it does not prescribe runtime behavior, parser changes, schema changes, or realism expansion.

Taxonomy findings:

- `detection`, `selection`, `ambiguity`, `lifecycle`, `divergence`, `outcome`, and `provenance_warning` remain suitable reviewer-facing categories when tied to source evidence and caveats.
- `commitment` remains acceptable only as reviewer shorthand backed by selected-id evidence; it must not imply operator, tactical, or runtime authority.
- Divergence-window wording should continue to use `localized near`, `adjacent to`, `associated with`, or `consistent with` instead of causal phrasing.
- Lifecycle and churn labels remain overlays over raw log lines; they are not tracker lifecycle truth or robustness evidence.
- Warning-heavy and missing-provenance cases should keep limitations visible instead of cleaning the narrative for readability.

Semantic-drift findings:

- `validation` in Phase 3 means reviewer-interpretation validation only.
- `clear`, `ambiguous`, `overloaded`, and similar rubric terms describe reviewer readability, not runtime correctness.
- The case matrix remains a review aid, not campaign coverage evidence.
- A static validation report, if later implemented, remains explanatory and non-authoritative.
- Interactive frontend architecture, live replay systems, and operational visualization flows remain out of scope for this phase.

Ambiguity and provenance findings:

- Fragmentation, ghost, dropout, stale, or delay evidence should stay descriptive and sequence-oriented.
- Visibility-limited classifications should remain inconclusive rather than negative, severe, or operational.
- Missing metadata, seed lineage, cohort, or git state should remain visible as provenance limitations.
- Source warnings should be preserved close to the reviewer finding they inform.

Maintainability findings:

- Repeated caveats should link back to the reviewer interpretation guide or Phase 2 boundaries where possible.
- Future wording changes should be recorded as reviewer-interpretation wording, not parser or runtime changes.
- Report sections should centralize common caveats to reduce drift and reviewer fatigue.
