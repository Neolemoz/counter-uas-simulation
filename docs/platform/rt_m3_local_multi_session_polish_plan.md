# RT-M3 — Local Multi-Session Polish (PLAN-RT-M3)

**Phase:** PLAN-RT-M3 — local multi-session polish planning (docs only)  
**Prerequisite:** PLAN-RT-C1 frozen; PLAT-RT-M2 frozen; PLAN-RT-M1 contracts frozen  
**Baseline:** [rt_m2_multi_session_implementation_plan.md](rt_m2_multi_session_implementation_plan.md)  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-M3** (this wave) | Polish **planning** for local multi-session UX and maintainer tooling — documentation only |
| **PLAT-RT-M3** | Future **implementation** wave — not authorized by this plan |
| **Distributed multi-bridge** | **Forbidden** — not M3 |
| **PLAN-RT-M1 / PLAT-RT-M2** | Architecture and delivery baseline — not re-opened |

New artifacts use `rt_m3_*` filenames to avoid collision with `rt_m1_*` / `rt_m2_*`.

## Goal

Plan additive polish for the **existing** local single-bridge multi-session RT sandbox (cap=3): background polling ergonomics, maintainer `rt_session_inspect` CLI, and tab/workspace UX improvements — with governance re-validation, PLAT-RT-M3 scope definition, and freeze — **without** changing runtime behavior.

## Allowed

- Plan, local multi-session UX review, governance review, PLAT roadmap, freeze audit
- Optional contract annex: [rt_multi_session_poll_policy_v1.md](../evaluation/rt_multi_session_poll_policy_v1.md)
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Pointer in [rt_roadmap_m1_m2_v1.md](../evaluation/rt_roadmap_m1_m2_v1.md) (cross-link only)
- Regression evidence citations from existing CI

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- New bridge commands, telemetry channels, parser/topic/schema changes in PLAN-RT-M3
- Distributed multi-bridge, cloud orchestration, browser→ROS expansion
- Tactical redesign, F6/SA workflow automation changes, SA viewer changes
- Authorization of **PLAT-RT-M3** implementation in this wave

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Background polling | [rt_multi_session_telemetry_routing_v1.md](../evaluation/rt_multi_session_telemetry_routing_v1.md), [rt_multi_session_poll_policy_v1.md](../evaluation/rt_multi_session_poll_policy_v1.md) |
| 2 | Session inspection | `rt_session_inspect.py` design (PLAT); `list_sessions` + audit log paths |
| 3 | Tab / workspace polish | [rt_multi_session_workstation_ui_v1.md](../evaluation/rt_multi_session_workstation_ui_v1.md), UX review |
| 4 | Governance | [rt_multi_session_governance_v1.md](../evaluation/rt_multi_session_governance_v1.md) |
| 5 | PLAT scope + roadmap | [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md) |

## Deliverables

| Artifact | Path |
|----------|------|
| Local multi-session UX review | [rt_m3_local_multi_session_ux_review_r1.md](../evaluation/rt_m3_local_multi_session_ux_review_r1.md) |
| Governance review | [rt_m3_governance_review_r1.md](../evaluation/rt_m3_governance_review_r1.md) |
| PLAT-RT-M3 roadmap | [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md) |
| Freeze audit | [rt_m3_freeze_audit.md](../evaluation/rt_m3_freeze_audit.md) |

## `rt_session_inspect` (PLAT design)

Maintainer CLI under `scripts/rt/rt_session_inspect.py` — **not** a `RUNTIME_SUBCOMMAND`:

| Subcommand | Behavior |
|------------|----------|
| `list [--json]` | Loopback `list_sessions` |
| `show SESSION_ID` | Registry row + lifecycle + editing flag |
| `summary` | `non_terminal_count`, `capacity`, `editing_session_id` |
| `audit SESSION_ID [--tail N]` | Read-only tail of `runs/rt_sandbox/audit/{session_id}.json` |

Loopback-only; fails closed if bridge unreachable. Pattern: [rt_adapter_inspect.py](../../scripts/rt/rt_adapter_inspect.py).

## Background polling (PLAT target)

| Role | Channels | Hz cap | Stale UI |
|------|----------|--------|----------|
| Active | Full `TELEMETRY_CHANNELS` | User ≤ 10 Hz | Full cognition strips |
| Background | `DIAGNOSTIC_TELEMETRY_CHANNELS` | 1 Hz | `BackgroundDiagnostics` compact row |

See [rt_multi_session_poll_policy_v1.md](../evaluation/rt_multi_session_poll_policy_v1.md) for normative PLAT rules.

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

Documentation-only diff hygiene: no changes under `platform/*` implementation trees or `src/counter_uas/`.

## Stop line

**PLAN-RT-M3** freezes polish planning. Do not start **PLAT-RT-M3** until:

1. [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md) scope is accepted, and  
2. PLAT wave completes governance review + freeze audit + regression.

Recommended next step is **advisory only** — not authorization.

## Related

- [rt_m3_local_multi_session_ux_review_r1.md](../evaluation/rt_m3_local_multi_session_ux_review_r1.md)
- [rt_m3_governance_review_r1.md](../evaluation/rt_m3_governance_review_r1.md)
- [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md)
- [rt_m3_freeze_audit.md](../evaluation/rt_m3_freeze_audit.md)
- [rt_roadmap_next_frontiers_v2.md](../evaluation/rt_roadmap_next_frontiers_v2.md)
