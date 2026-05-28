# RT - PLAT-RT-V4 P2 Architecture Review R1

**Phase:** PLAT-RT-V4 P2
**Status:** accepted for freeze

## Review summary

P2 completes V4 by polishing workstation layout and multi-session cognition without adding command paths. It strengthens additive compatibility for layer-toggle persistence and improves display readability around density and session roles.

## Architecture decisions

| Decision | Rationale | Boundary |
|----------|-----------|----------|
| Normalize persisted visibility | Additive registry keys should not break older saved display preferences | Local browser display state only |
| Add compact compare chips | Makes selected/comparison/background state readable in dense rails | No command affordance |
| Add chrome summary helper | Keeps multi-session role counts deterministic and testable | Derived UI text only |
| Show rail active counts | Improves density scanning without enforcing budgets | Warn-only |
| Align workstation shell | Improves cognition rail and diagnostics readability | Layout-only |

## Coupling assessment

- **Bridge/runtime coupling:** none. No bridge files changed.
- **SA coupling:** none. No SA viewer files changed.
- **Registry coupling:** low. No new registry truth; persistence normalization uses current registry defaults.
- **UI coupling:** low. P2 modifies existing workstation, rail, and comparison components.
- **Governance coupling:** low. All new text preserves explanatory/display-only semantics.

## Accepted residual risk

The main residual risk is confusing persisted display preferences with authoritative session state. P2 mitigates this by wording memory as local display memory and by clearing per-session entries on disconnect as before.
