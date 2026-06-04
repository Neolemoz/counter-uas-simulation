# RT Intelligence Advisory V1

## Scope

`rt_intelligence_advisory_v1` is a recommendation-only intelligence layer for
runtime counter-UAS scenarios. It derives threat evaluation, defender ranking,
confidence, and reasoning from already-audited inputs.

This contract is additive to the RT sandbox. It does not change tactical state,
tactical recommendations, entity pose mirrors, parser contracts, ROS topics, or
Gazebo behavior.

## Advisory Contract

Each advisory object has schema `rt_intelligence_advisory_v1` and contains:

- `identity`: `advisory_id`, `attacker_id`, `advisory_utc`
- `threat_evaluation`: `threat_score`, `threat_rank`, `threat_components`
- `recommended_defender`: `defender_id`, `feasibility`, `tti_s`
- `defender_ranking`: `ranked_defenders`
- `reasoning`: `reason_codes`, `explanation`
- `confidence`: `heuristic_confidence`
- `governance`: advisory authority and banner

Threat components are limited to audited inputs:

- `distance_to_protected_center`
- `best_feasible_tti`
- `descent_factor`
- `critical_zone_factor`

No perception, sensor-fusion, hardware, PX4, MAVLink, or HITL fields are part
of this contract.

## Transport Contract

The read-only telemetry surface uses schema
`rt_intelligence_advisory_transport_v1`:

```json
{
  "schema": "rt_intelligence_advisory_transport_v1",
  "session_id": "...",
  "advisory_utc": "...",
  "source": "rt_intelligence_advisory_engine",
  "authority": "recommendation_only",
  "governance_banner": "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority",
  "refresh_reason": "snapshot",
  "stale": false,
  "stale_reason": null,
  "advisories": []
}
```

The telemetry channel is `intelligence_advisory`.

The transport payload is a mirror only. It may include zero or more
`rt_intelligence_advisory_v1` objects in `advisories`.

## Refresh Semantics

Current transport generation is snapshot-based. If a session has a scoped
`rt_intelligence_advisory_input_v1` object, the transport builds advisories from
that input. If the input is unavailable or invalid, the transport emits a stale,
empty advisory list.

Valid `refresh_reason` values are descriptive only. They do not imply command
authority.

## Stale Semantics

`stale: true` means the advisory transport should not be treated as current.

Known stale reasons:

- `input_unavailable`
- `input_invalid`
- `source_stale`

Stale advisory transport remains read-only. Consumers may dim or label stale
output, but must not mutate assignments, command defenders, approve
recommendations, or trigger engagement.

## Governance Boundaries

The intelligence advisory layer is recommendation only.

Explicitly prohibited:

- assignment authority
- engagement authority
- weapon authority
- autonomous engagement behavior
- interceptor command mutation
- tactical-controller state mutation
- `tactical_state` schema changes
- `tactical_recommendation` schema changes
- `entity_pose_mirror` schema changes
- ROS or Gazebo imports in the advisory engine
- SA coupling
- parser/topic redesign

The `heuristic_confidence` field is advisory quality confidence only. It is not
probability of kill, mission success confidence, engagement confidence, or
authorization confidence.

## Limitations

The transport does not derive live advisory inputs from ROS, Gazebo, or tactical
controller internals. It exposes only a read-only payload when scoped advisory
inputs are present.

No UI rendering, tactical overlays, advisory panels, or operator workflows are
included in V1.

## Future UI Consumers

Future UI work may read this channel for:

- threat ranking cues
- recommended defender display
- defender ranking lists
- confidence labels
- advisory explanation panels
- stale/governance banners

Future UI work must remain read-only unless a separately frozen governance wave
opens an explicit approval path.
