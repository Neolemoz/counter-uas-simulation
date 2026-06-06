# INTEL-LIVE1 Live Intelligence Advisory

**Phase:** PLAT-RT-INTEL-LIVE1
**Status:** frozen
**Authority:** recommendation only

## Architecture

INTEL-LIVE1 assembles the existing `rt_intelligence_advisory_input_v1` at
`intelligence_advisory` snapshot time. It does not cache or reuse a previous
advisory input. The existing advisory engine and
`rt_intelligence_advisory_transport_v1` remain unchanged.

Source authority is bounded by data category:

| Input | Authority |
|---|---|
| Protected-center, attacker, and defender positions | Session `EntityRegistry` |
| Attacker velocity and descent rate | Raw current `TelemetryMirror.entity_pose_mirror` |
| Pairwise TTI and feasibility | `tactical_geometry.compute_intercept` |
| Defender speed cap | Existing session tactical state |

Distances to the protected center use horizontal ENU X/Y distance only.

## Protected Center Contract

Exactly one session-scoped entity may be designated with
`designate_protected_center`. Designation is explicit only. Existing
designation replacement requires `replace: true`. Deleting the designated
entity or resetting the session clears the designation.

No protected-center authority is inferred from world origin, operational
rings, radar type, waypoint markers, scenario groups, or Gazebo semantics.
Missing or deleted designation fails closed with
`protected_center_unavailable`.

## Assembler Contract

The assembler captures registry revision, telemetry revision, registry entity
rows, raw telemetry rows, and tactical speed cap. It validates all required
inputs before deriving the complete attacker-by-defender TTI matrix. Registry
or telemetry revision change during assembly fails closed.

A velocity is valid only when exactly one raw telemetry row for the attacker
contains explicit finite numeric `velocity.x`, `velocity.y`, and `velocity.z`.
Explicit zero velocity is valid. Missing components, normalized defaults,
duplicate rows, non-finite values, and retained older-revision values are not
valid live inputs. Mixed freshness and partial advisory output are prohibited.

Empty attacker and defender sets are valid complete snapshots. An empty
attacker set produces a fresh empty advisory list. An empty defender set
produces attacker advisories without a recommended defender.

## Stale Reasons

Fail-closed stale-reason precedence is:

1. `protected_center_unavailable`
2. `telemetry_feedback_lost`
3. `telemetry_stale`
4. `attacker_velocity_missing`
5. `attacker_velocity_stale`
6. `attacker_velocity_nonfinite`
7. `defender_invalid`
8. `snapshot_validation_failed`

Every failed validation publishes `stale: true` and `advisories: []`.

## Publication Contract

Publication follows:

```text
registry or telemetry update
-> invalidate previous advisory by construction
-> assemble immutable snapshot
-> validate and derive
-> publish world/entity telemetry
-> publish intelligence_advisory
```

Telemetry poll failure sets `telemetry_health: feedback_lost` and publishes the
existing telemetry channel set, including a stale empty
`intelligence_advisory`. No previous valid advisory remains current.

## Governance Boundaries

INTEL-LIVE1 remains recommendation-only and read-only with respect to tactical
authority. It adds no assignment authority, engagement authority, weapon
authority, autonomy, ROS topics, telemetry channels, parser contracts, or
transport schema fields. The assembler does not mutate the entity registry,
tactical controller, assignments, or runtime adapter.
