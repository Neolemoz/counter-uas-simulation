# RT-F2 — Runtime Platform Hardening (PLAN-RT-F2)

**Phase:** PLAN-RT-F2 — stability maintenance (docs only)  
**Prerequisite:** PLAT-RT-F3 frozen; PLAN-RT-R2 frozen  
**Contracts:** [rt_runtime_cleanup_hardening_v1.md](../evaluation/rt_runtime_cleanup_hardening_v1.md), [rt_ui_hardening_v1.md](../evaluation/rt_ui_hardening_v1.md), [rt_experiment_import_hardening_v1.md](../evaluation/rt_experiment_import_hardening_v1.md)

## Goal

Harden RT runtime teardown, experiment import/cache lifecycle, and UI orphan-state handling — without new capabilities or authority changes.

## Forbidden

- Bridge protocol / new commands
- SA viewer, federation, auto-import
- Parser/topic changes
- Feature expansion (F4/M3/F5)

## PLAT-RT-F2 scope (advisory)

See [rt_roadmap_plat_rt_f2_v1.md](../evaluation/rt_roadmap_plat_rt_f2_v1.md).

## Stop line

PLAN-RT-F2 frozen. Do not start PLAT-RT-F2 without governance review + freeze audit.
