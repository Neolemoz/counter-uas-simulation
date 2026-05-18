# PHASE 3 Replay Narrative Validation Plan

## Scope

`AGENTS.md` remains the primary authority. In this phase, `validation` means reviewer-interpretation validation only: checking whether static replay narrative artifacts are understandable, semantically bounded, taxonomy-coherent, and maintainable across replay cases.

This phase does not validate operational readiness, certification, runtime correctness, tactical authority, lifecycle truth, system capability, field performance, or robustness. It does not authorize runtime changes, parser-contract changes, live dashboards, HITL/operator workflows, causal explanation systems, readiness scoring, or governance automation expansion.

Reference boundaries:

- [Reviewer Interpretation Guide](reviewer_interpretation_guide.md)
- [PHASE 2 Replay Narrative UX Plan](replay_narrative_ux_phase2_plan.md)
- [Replay Narrative UX Freeze Audit](replay_narrative_ux_freeze_audit.md)
- [Replay Observability Freeze Audit](replay_observability_freeze_audit.md)

## Goal

Evaluate whether replay narratives help reviewers understand sequence, ambiguity, taxonomy, provenance, and caveats without collapsing derived artifacts into authority or implying causal, operational, or readiness conclusions.

## Waves

### Wave 0: Documentation Scope

Create this plan and freeze the interpretation-only meaning of `validation` before any tooling or schema work.

### Wave 1: Case Matrix Definition

Define a small static replay-case matrix from existing artifacts where possible. The matrix should exercise reviewer interpretation failure modes, not runtime capability.

Initial case slots:

| Case ID | Replay Pattern | Source Preference | Reviewer Focus |
|---|---|---|---|
| `P3-C1-low-ambiguity` | Low-ambiguity replay | Existing Phase 2 narrative JSON or single-run replay observability report | Baseline sequence readability and caveat visibility |
| `P3-C2-lifecycle-heavy` | Lifecycle/churn-heavy replay | Existing lifecycle timeline or replay narrative with lifecycle overlays | Whether lifecycle labels remain explanatory and non-authoritative |
| `P3-C3-divergence-window` | Divergence replay with mismatch windows | Existing divergence trace or replay narrative with mismatch events | Whether divergence is read as localization, not causality |
| `P3-C4-visibility-limited` | Visibility-limited replay | Existing D5 or warning-heavy replay observability artifact | Whether uncertainty remains visible without implying severity |
| `P3-C5-missing-provenance` | Missing or partial lineage replay | Existing artifact with missing meta, seed, cohort, or git state warning | Whether provenance gaps remain visible and bounded |
| `P3-C6-fragmentation-ambiguity` | Fragmentation or ambiguity replay | Existing narrative with fragmented-gap, ghost, dropout, stale, or delay evidence | Whether ambiguity wording stays descriptive and non-causal |
| `P3-C7-maintenance-sentinel` | Repeated-label or repeated-caveat replay | Any existing narrative that stresses repeated taxonomy labels or caveats | Whether labels and caveats remain maintainable without drift |

Selection criteria:

- use existing Phase 2 narrative artifacts or existing replay observability inputs first;
- prefer cases with clear lineage, known caveats, and stable deterministic outputs;
- include at least one warning-heavy or provenance-limited case so caveat visibility is reviewed;
- keep the initial case set small enough for manual review.
- record why each selected case is useful for reviewer-interpretation validation;
- preserve source artifact paths and warning text when a concrete case is selected later.

Exclusions:

- no new runtime captures solely to fill the matrix;
- no campaign-wide coverage claims;
- no performance, readiness, or robustness ranking;
- no requirement that every taxonomy label appear before planning can proceed.
- no inference that absence of a case means the behavior is unsupported, valid, invalid, ready, or certified.

The matrix is a reviewer-coverage aid, not a system coverage guarantee. Its purpose is to reveal whether narrative wording and grouping remain understandable across representative replay evidence patterns.

### Wave 2: Reviewer Rubric

Define a qualitative rubric for reviewer interpretation:

| Dimension | Reviewer Question | Acceptable Finding Language | Common Misread To Avoid |
|---|---|---|---|
| Sequence readability | Can a reviewer reconstruct the replay sequence without reading every raw log line? | `clear sequence`, `sequence gap`, `ordering ambiguous` | Treating narrative order as causal proof |
| Event grouping clarity | Are related events grouped without hiding source evidence? | `grouping clear`, `grouping overloaded`, `needs source split` | Treating grouped events as a unified replay state |
| Taxonomy consistency | Are event categories and labels stable across similar evidence? | `label consistent`, `label overloaded`, `taxonomy drift risk` | Treating taxonomy labels as parser fields or runtime states |
| Provenance visibility | Are source paths, warnings, seed/cohort lineage, and missing metadata visible? | `provenance clear`, `lineage incomplete`, `warning needs prominence` | Treating lineage as certification or comparability proof |
| Caveat visibility | Are interpretation caveats visible without dominating the report? | `caveat visible`, `caveat buried`, `caveat duplicated` | Treating caveats as approval, severity, or operational status |
| Ambiguity handling | Are ambiguity and visibility limits presented as uncertainty, not diagnosis? | `ambiguity bounded`, `uncertainty unclear`, `visibility limit hidden` | Treating ambiguity as causal diagnosis |
| Authority-overread risk | Does wording preserve derived/non-authoritative status? | `boundary clear`, `authority overread risk`, `wording too strong` | Treating narrative findings as tactical authority |
| Maintainability | Can maintainers update labels and summaries without duplicating governance text? | `maintainable`, `repetition drift risk`, `needs central reference` | Treating maintainability feedback as runtime redesign mandate |

Each rubric dimension should use descriptive findings such as `clear`, `ambiguous`, `overloaded`, `missing provenance`, or `needs wording review`. Avoid numeric scores, pass/fail gates, approval terms, severity labels, or readiness bands.

Allowed reviewer finding patterns:

- `The sequence is clear for reviewer reconstruction, with source references visible.`
- `The label is overloaded across cases and should be clarified before tooling expansion.`
- `The warning is visible but could be misread as severity; wording should emphasize interpretation prompt.`
- `The divergence description is localized in time but should avoid causal phrasing.`
- `The case is provenance-limited; the limitation is visible and should remain in the report.`

Forbidden finding patterns:

- `validated`, `approved`, `certified`, `ready`, `robust`, or `operationally acceptable`;
- numeric or color-coded readiness scores;
- causal conclusions such as `fragmentation caused the miss`;
- parser or runtime prescriptions such as `promote this label to a parser field`;
- operator or HITL language such as `reviewer accepted the decision`.

Reviewer questions:

- Can the reviewer reconstruct the replay sequence without reading every raw log line?
- Are event categories and labels stable across similar evidence?
- Are divergence and ambiguity presented as localization or association rather than causality?
- Are warnings visible without reading as operational status?
- Are parser-visible summaries kept separate from derived narrative interpretation?
- Can maintainers update labels without duplicating governance caveats?

Rubric findings must remain descriptive reviewer notes and must not become readiness, approval, severity, certification, or robustness scores.

### Wave 3: Static Validation Report Shape

If later needed, document a static report shape for interpretation findings only. This wave is not a schema commitment, implementation commitment, validator, or parser contract.

Candidate sections:

- `purpose`: reviewer-interpretation validation only, with explicit non-authoritative scope;
- `source narrative artifacts`: paths, artifact types, lineage status, warnings, and caveats copied from existing static artifacts;
- `case matrix`: selected Wave 1 case ids, replay pattern, source artifact, and why the case was selected;
- `reviewer-interpretation findings`: qualitative Wave 2 rubric findings, not scores or approvals;
- `taxonomy-coherence findings`: label consistency, overloaded labels, missing context, and maintainability notes;
- `semantic-drift findings`: wording that risks causal, tactical, operational, readiness, or authority overread;
- `ambiguity findings`: uncertainty, visibility limits, fragmentation adjacency, and provenance gaps;
- `maintainability findings`: repeated caveats, duplicated language, or unclear ownership of terminology;
- `warnings and caveats`: source warnings plus report-level interpretation caveats.

Required report properties if a later wave implements it:

- static file input and output only;
- deterministic ordering by case id, finding category, source artifact, and finding text;
- no wall-clock generation claims unless explicitly excluded from deterministic checks;
- preserved source artifact paths and warnings;
- explicit statement that findings are reviewer-interpretation observations only.
- no generated readiness, certification, severity, approval, capability, or robustness fields;
- no live links to runtime processes, websocket streams, ROS topics, or operator actions.

Suggested qualitative finding shape for future documentation or static artifacts:

- `case_id`: stable Wave 1 case id;
- `source_artifact`: static replay narrative or replay observability artifact path;
- `finding_category`: `usability`, `taxonomy`, `semantic_drift`, `ambiguity`, `provenance`, or `maintainability`;
- `finding`: bounded reviewer note using Wave 2 language;
- `evidence_pointer`: source section, event id, warning text, or caveat reference;
- `interpretation_boundary`: short reminder when a finding could be overread.

The report shape should support maintainability by centralizing repeated findings instead of copying long caveat blocks into every case.

### Wave 4: Taxonomy And Drift Review

Review whether event labels remain coherent across cases and whether wording risks causal, tactical, operational, or readiness overread.

Taxonomy coherence checklist:

- `detection`, `selection`, `ambiguity`, `lifecycle`, `divergence`, `outcome`, and `provenance_warning` labels are used consistently with Phase 2 boundaries.
- `commitment` remains reviewer shorthand only when backed by existing selected-id evidence and never implies operator or tactical authority.
- Divergence windows are described as replay-side localization, not explanation or proof.
- Lifecycle/churn labels remain overlays over raw log lines, not tracker lifecycle truth.
- Warning language remains an interpretation prompt, not severity, approval, or operational status.
- Provenance gaps remain visible and are not smoothed into clean narratives.
- Repeated caveats are linked or summarized to reduce drift and reviewer fatigue.

Semantic drift checklist:

- `validation` means reviewer-interpretation validation only.
- `clear`, `ambiguous`, or `overloaded` describe reviewer readability, not runtime correctness.
- `localized near`, `adjacent to`, `associated with`, and `consistent with` remain preferred over causal wording.
- `missing provenance` remains a limitation, not a failed operational status.
- `case matrix` remains a review aid, not coverage evidence.
- `static report` remains an explanatory artifact, not an approval record.

Ambiguity review checklist:

- uncertainty is visible where observer signals, lineage, or selection evidence are limited;
- fragmented-gap adjacency is not promoted to cause;
- visibility-limited classifications remain inconclusive rather than negative or severe;
- missing metadata is not hidden for narrative readability.

Maintainability checklist:

- repeated wording points back to the reviewer interpretation guide or Phase 2 boundaries where possible;
- label changes are documented as interpretation wording, not parser or runtime changes;
- future report sections have clear ownership and do not duplicate freeze-audit prose unnecessarily.

Drift findings should identify wording or taxonomy maintenance issues. They should not prescribe runtime behavior, parser changes, or broader realism expansion.

### Wave 5: Freeze Audit

Freeze only after documentation, any later static tooling, and focused checks confirm the work remains replay-side, static, deterministic, and interpretation-only.

Audit scope:

- documentation-only if Waves 1-4 remain documentation-only;
- static artifact checks only if later waves explicitly add generated validation reports;
- no runtime, parser, schema, topic, test, renderer, validator, or tooling changes unless separately approved in a later scoped wave.

The freeze audit should confirm:

- all findings are reviewer-interpretation findings only;
- no runtime, parser, schema, topic, tracker, fusion, tactical authority, or live-system surface changed;
- no readiness, certification, capability, or robustness score was introduced;
- case-matrix limitations are visible;
- provenance and caveats remain visible in any static reports;
- deterministic output checks exist if later waves add generated artifacts;
- the reviewer interpretation guide and Phase 2 narrative boundaries remain linked.
- no causal AI explanation, automated causal diagnosis, or generated causal narrative was introduced;
- no reviewer finding is framed as approval, acceptance, severity, readiness, or operational status.

Freeze evidence should remain proportional to scope:

- for documentation-only waves: markdown diagnostics and diff review are sufficient;
- for later static generated artifacts: deterministic repeat-generation checks may be required;
- for any proposed tooling beyond static generation: stop and create a new scoped plan before implementation.

## Risks

- `validation` could be misread as certification, readiness, runtime correctness, or capability evidence.
- Usability findings could become informal scores or approval gates.
- Taxonomy review could drift into parser-contract enforcement.
- Ambiguity findings could be mistaken for causal diagnosis.
- Case matrices could be mistaken for coverage guarantees.
- Static validation reports could become a new authority layer.
- Repeated reviewer caveats could diverge if copied into every finding instead of referenced.
- Maintainability findings could be misread as a mandate for architecture or runtime refactoring.

## Deterministic Artifact Guidance

If later waves add generated validation artifacts, keep them static and deterministic:

- use sorted JSON keys and stable Markdown ordering;
- avoid wall-clock timestamps or generated run ids in deterministic outputs;
- sort cases by stable case id, source path, and finding category;
- preserve source paths, warnings, and missing-provenance notes exactly;
- keep reviewer notes textual and bounded, not scored;
- do not call LLMs, causal models, or heuristic explanation engines during generation;
- render identical inputs to byte-identical outputs where practical.

## Evaluation Workflow

1. Select a small static replay narrative case matrix.
2. Generate or reuse deterministic Phase 2 narrative artifacts.
3. Review each narrative against the interpretation rubric.
4. Record ambiguity, drift, taxonomy, and provenance findings as descriptive reviewer notes.
5. Compare findings across cases for repeated interpretation problems.
6. Update documentation wording only where clarity improves without changing semantics.
7. Add tooling or tests only in later scoped waves, after documentation boundaries are stable.
8. Freeze with an audit before expanding case coverage or report structure.

## Minimal Implementation Workflow

Use one narrow future request per wave:

- Wave 1: document the static case matrix and case selection rules only.
- Wave 2: document the reviewer rubric only.
- Wave 3: document the static validation report shape only; do not implement schemas or renderers unless explicitly approved later.
- Wave 4: perform a documentation-only taxonomy and semantic drift review across selected artifacts.
- Wave 5: add a focused freeze audit after prior waves stabilize.

For each later implementation turn:

- read `AGENTS.md` first;
- restate allowed and forbidden scope;
- touch only the named files for that wave;
- run markdown checks or focused deterministic checks relevant to the edited files;
- separate pre-existing dirty files from that wave's edits;
- stop before runtime expansion, parser changes, live UI work, scoring, or causal explanation.

## Freeze Boundaries

PHASE 3 freezes as a reviewer-interpretation validation layer only. Freeze requires:

- no runtime, launch, topic, schema, parser-visible field, tracker, fusion, or tactical authority changes;
- no operational-readiness, certification, system-capability, field-performance, or robustness scoring claims;
- no live dashboard, websocket, ROS subscription, operator control, or approval workflow;
- no causal explanation system;
- deterministic static artifacts if later waves add generated reports;
- visible provenance and caveats;
- case-matrix and rubric findings remain qualitative reviewer notes;
- static reports, if later added, remain explanatory reviewer artifacts rather than authority surfaces;
- continued linkage to the reviewer interpretation guide and Phase 2 replay narrative boundaries.
