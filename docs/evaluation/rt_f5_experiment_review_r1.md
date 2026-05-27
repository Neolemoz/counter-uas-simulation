# RT-F5 — Experiment Review R1

**Phase:** PLAN-RT-F5 — metric completeness & determinism (docs only)  
**Plan:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md)  
**Contracts:** [rt_experiment_model_v1.md](rt_experiment_model_v1.md), [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md)

---

## 1. Experiment class coverage

| Class | Spec fixture | Batch compile | Metrics rollup | Verdict |
|-------|--------------|---------------|----------------|---------|
| `terrain_comparison` | Yes | explicit_list | `terrain_rollup` | **Pass** |
| `sensor_range_comparison` | Yes | explicit_list | F1 entity contrast | **Pass** |
| `tactical_mode_comparison` | Yes | explicit_list | `tactical_rollup` | **Pass** |
| `repeatability_sweep` | Yes | repeat_expand | `repeatability_rollup` | **Pass** |
| `parameter_matrix` | Yes | cartesian | `matrix_rollup` | **Pass** |

---

## 2. Determinism

| Check | Result |
|-------|--------|
| Pure derive forbids `Date.now()` | **Pass** — metrics §8 |
| Stable sort orders documented | **Pass** |
| `spec_fingerprint` canonical rules | **Pass** |
| Missing inputs → `null` / `unavailable` | **Pass** |
| F1 report required input — no fork of F1 schema | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| F5-EXP-DET-01 | Pass |

---

## 3. Forbidden lexicon

| Location | Scan | Result |
|----------|------|--------|
| Model contract | No winner/readiness/effectiveness | **Pass** |
| Metrics contract | Forbidden badges listed | **Pass** |
| UI contract | No outcome colors / winner column | **Pass** |
| Fixtures | Explanatory hypothesis labels only | **Pass** |

---

## 4. Compare alignment

| Check | Result |
|-------|--------|
| X1 badge ids reused | **Pass** |
| F5 additive badges documented | **Pass** |
| Comparison criteria table normative | **Pass** |
| `capture_asymmetric` for partial captures | **Pass** |

---

## 5. Handoff eligibility

| Check | Result |
|-------|--------|
| Gates mirror SA1 checklist themes | **Pass** |
| Eligibility does not call approve/import | **Pass** |
| `pose_cognition_ack` maintainer attestation only | **Pass** |
| `partial` experiment level documented | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| F5-EXP-HO-01 | Pass-with-conditions | PLAT must not auto-set `pose_cognition_ack` true |

---

## 6. Workflow completeness

| Phase A–I | Documented | Verdict |
|-----------|------------|---------|
| Build → compile → batch → capture | Yes | **Pass** |
| F1 then F5 derive order | Yes | **Pass** |
| F3 annex optional | Yes | **Pass** |
| SA handoff manual only | Yes | **Pass** |

---

## 7. Verdict

**Pass** — Experiment taxonomy, metrics extension, and workflow are complete for PLAN freeze. PLAT-RT-F5 should add golden tests from `f5_spec_examples/` at implementation time.
