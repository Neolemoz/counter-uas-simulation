# Evaluation Expansion Plan

## Objective

Expand the evaluation roadmap so runtime realism and runtime supervision changes
can be judged with deterministic, replay-compatible metrics.

## Planning guardrails

- preserve parser-safe additive evolution
- keep metric semantics explicit and grepable
- prefer deterministic aggregates over impressionistic run reviews
- avoid conflating descriptive robustness scores with operational readiness

## Workstreams

### 1. Suppression and stability metrics

- suppression histograms
- command-gating counts
- planner churn metrics
- assignment/selection stability metrics
- hold/switch frequency metrics

### 2. Stale and watchdog metrics

- stale-event counts
- stale-duration metrics
- watchdog trigger counts
- degraded-state dwell metrics
- failsafe transition counts

### 3. Divergence and recovery metrics

- truth-vs-track divergence classification
- recovery-after-stale metrics
- recovery-after-dropout metrics
- recovery-after-fragmentation metrics

### 4. Timing and latency metrics

- runtime latency metrics
- publication skew metrics
- delayed-detection onset metrics
- supervisor reaction-latency metrics

### 5. Composite robustness scoring

- robustness score planning only
- must remain decomposable into explicit metric families
- must not replace underlying raw metric reporting

## Roadmap skeleton

### Near-term

- define metric families and naming
- define which metrics must remain parser-safe additive
- map realism and supervisor stressors to evaluation outputs

### Mid-term

- extend per-run and aggregate reporting with new metric families
- add divergence/recovery classifiers
- add deterministic scenario stress benchmarks

## Explicit non-goals

- operational readiness scoring
- opaque single-number grading without supporting metrics
- approval or operator performance metrics
