# RT-X3 — Governance Review R1

**Phase:** PLAN-RT-X3 — experiment workbench v3 (docs only)  
**Plan:** [rt_x3_experiment_workbench_v3_plan.md](../platform/rt_x3_experiment_workbench_v3_plan.md)  
**Freeze audit:** [rt_x3_freeze_audit.md](rt_x3_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| New bridge commands? | **No** |
| SA viewer changes? | **No** |
| Auto-import from review packet? | **No** |
| Import semantic changes? | **No** — [rt_experiment_import_hardening_v1.md](rt_experiment_import_hardening_v1.md) unchanged |
| Browser capture/import? | **No** |
| Experiment = authority? | **No** — all v3 artifacts explanatory |
| F8 scope in X3? | **No** — optional `advisory_refs` packet section only |

**Recommendation:** Freeze **PLAN-RT-X3** (docs only).

**Contamination review:** Required at **PLAT-RT-X3 P1** when grouped dock surfaces F6/F7 adjacent to experiment flows — not required for PLAN freeze.

---

## 1. Authority boundaries

| Artifact | Authoritative? |
|----------|----------------|
| `rt_experiment_manifest_v1` | Explanatory mirror at pin time |
| Cohort index | Reference catalog only |
| F1/F5/F5b reports | Derived |
| Review packet + `sections[]` | Advisory export — not import commit |
| Step completion badges | Explanatory — not workflow gates |
| Compare-status vocabulary | Display only |

| Finding ID | Verdict |
|------------|---------|
| X3-GOV-AUTH-01 | Pass |

---

## 2. SA isolation

| Rule | Verdict |
|------|---------|
| No `platform/sa-r0-viewer/` in contracts | Pass |
| Forbidden `sa_corpus_ref` on packet/cohort | Pass |
| Review packet ≠ `replay_sa_bundle_v1` | Pass |
| `sections[]` must not include SA import commands | Pass |

| Finding ID | Verdict |
|------------|---------|
| X3-GOV-SA-01 | Pass |

---

## 3. Lexicon and banners

Normative banners in v3 contracts avoid:

- readiness score, winner, effectiveness, operational picture, auto-import

Cross-reference [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) for F6/F7 footer and `advisory_refs` section copy.

| Finding ID | Verdict |
|------------|---------|
| X3-GOV-LEX-01 | Pass |

---

## 4. Import and capture (frozen)

| Rule | Verdict |
|------|---------|
| No change to `useJsonPromptImport` contract in PLAN | Pass |
| No new browser subprocess batch | Pass |
| Ref validation hints are shell/documentation only in PLAN | Pass |

| Finding ID | Verdict |
|------------|---------|
| X3-GOV-IMPORT-01 | Pass |

---

## 5. F5b coexistence

v3 references fidelity step completion; does not redefine `truth_attested` or coupling semantics.

| Finding ID | Verdict |
|------------|---------|
| X3-GOV-F5B-01 | Pass |

---

## Governance verdict

**Pass — suitable for freeze (docs only).**
