# Runtime Supervisor Plan

## Objective

Add a planning track for runtime supervision that can detect degraded autonomy
conditions, gate unsafe action, and expose replay-visible safety state without
pulling the repo into operator workflow or approval-chain design.

## Planning guardrails

- runtime supervisor planning is not HITL planning
- preserve existing tactical governance freezes
- separate supervisor state from mirrors and explanatory evidence
- keep metrics and replay evidence additive

## Candidate runtime states

- NORMAL
- DEGRADED
- FAILSAFE

These are planning labels only in this document. They are not runtime contracts.

## Workstreams

### 1. State-quality and stale-state detection

- stale track detection
- stale command-state detection
- inconsistent target/interceptor geometry detection
- missing-update watchdogs

### 2. Safety gating

- impossible-intercept rejection
- runtime envelope checks
- command gating before unsafe motion emission
- emergency freeze conditions

### 3. Supervisor observability

- replay-visible supervisor state
- watchdog transition evidence
- gated-command evidence
- safety metric extraction hooks

### 4. Metrics

- degraded-state dwell time
- failsafe triggers
- watchdog trigger counts
- gated-command counts
- recovery-from-degraded metrics

## Roadmap skeleton

### Near-term

- define supervisor state semantics and non-semantics
- define replay-visible evidence needs
- define evaluation metrics and scenario stressors

### Mid-term

- implement watchdog and stale-state detection paths
- add supervisor metrics to evaluation outputs
- build supervisor stress scenarios

## Explicit non-goals

- operator intervention flow
- approval workflow
- UI state machine design
- distributed safety authority
