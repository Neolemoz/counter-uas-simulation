# RT-X2 — Governance Review R1

**Phase:** PLAN-RT-X2 — experiment workbench v2 (docs only)  
**Plan:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)  
**Freeze audit:** [rt_x2_freeze_audit.md](rt_x2_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| New bridge commands? | **No** |
| SA viewer changes? | **No** |
| Auto-import from review packet? | **No** |
| Browser capture/import? | **No** |
| Experiment = authority? | **No** — all X2 artifacts explanatory |
| F8 scope in X2? | **No** — F6/F7 display reference only |

**Recommendation:** Freeze **PLAN-RT-X2** (docs only).

**Contamination review:** Recommended at **PLAT-RT-X2 P1** when unified review panel surfaces F6/F7 adjacent to experiment flows — not required for PLAN freeze (experiment-focused, lower than F8).

---

## 1. Authority boundaries

| Artifact | Authoritative? |
|----------|----------------|
| `rt_experiment_manifest_v1` | Explanatory mirror at pin time |
| Cohort index | Reference catalog only |
| F1/F5/F5b reports | Derived |
| Review packet | Advisory export — not import commit |
| Compare badges | Explanatory only |

| Finding ID | Verdict |
|------------|---------|
| X2-GOV-AUTH-01 | Pass |

---

## 2. SA isolation

| Rule | Verdict |
|------|---------|
| No `platform/sa-r0-viewer/` in contracts | Pass |
| Forbidden `sa_corpus_ref` on cohort/packet | Pass |
| Review packet ≠ `replay_sa_bundle_v1` | Pass |

| Finding ID | Verdict |
|------------|---------|
| X2-GOV-SA-01 | Pass |

---

## 3. Lexicon and banners

Normative banners in workbench v2, cohort, and review packet contracts avoid:

- readiness score, winner, effectiveness, operational picture, auto-import

Cross-reference [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) for F6/F7 footer copy when PLAT wires handoff strips.

| Finding ID | Verdict |
|------------|---------|
| X2-GOV-LEX-01 | Pass |

---

## 4. F5b coexistence

X2 references fidelity labels; does not redefine `truth_attested` or coupling semantics.

| Finding ID | Verdict |
|------------|---------|
| X2-GOV-F5B-01 | Pass |

---

## Governance verdict

**Pass — suitable for freeze (docs only).**
