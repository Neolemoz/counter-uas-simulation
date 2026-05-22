# Runtime Autonomy Planning Overview

This planning tree extends the repository from governance-foundation work into
implementation-oriented runtime autonomy roadmap work.

Scope of this tree:

- runtime realism
- runtime supervisor and safety
- scenario infrastructure
- evaluation expansion

Planning constraints carried forward from prior governance freezes:

- mirrors do not imply authority
- explanatory evidence does not imply authoritative state
- replay logs are not parser contracts
- additive planning must not alter runtime semantics

Out of scope for this tree:

- PX4 or MAVLink integration
- hardware bringup
- operator workflow design
- approval-chain design
- distributed authority/control design

Track index:

- [runtime_realism/OVERVIEW.md](runtime_realism/OVERVIEW.md)
- [runtime_supervisor/OVERVIEW.md](runtime_supervisor/OVERVIEW.md)
- [scenario_infrastructure/OVERVIEW.md](scenario_infrastructure/OVERVIEW.md)
- [evaluation_expansion/OVERVIEW.md](evaluation_expansion/OVERVIEW.md)

Execution policy for all tracks:

1. Freeze semantics before implementation where governance-sensitive boundaries exist.
2. Prefer parser-safe additive evolution.
3. Preserve replay reproducibility and deterministic evaluation where practical.
4. Keep runtime realism and runtime safety changes auditable through replay and metrics.
