# RT-F8 P2 — Architecture Review R1

**Phase:** PLAT-RT-F8 P2  
**Plan:** [rt_plat_f8_p2_advisory_guardrails_plan.md](../platform/rt_plat_f8_p2_advisory_guardrails_plan.md)

---

## Summary

P2 completes the F8 advisory wave by refining maintainer-only preview paths:

- **Corpus preview** remains read-only and is now policy-validated to `fixtures/sa_r0/**` destinations only.
- **Dry-run review** remains `dry_run: true` and now exposes explicit per-capture status buckets (`ran/skipped/error`) to improve maintainer cognition.
- **Preview export polish** remains client-only preview output (no authority change).

No bridge protocol changes, no SA viewer changes, no subcommand registry changes.

---

## Layering

| Layer | Change | Verdict |
|-------|--------|---------|
| Bridge HTTP | None | Pass |
| Subcommand registry | None | Pass |
| Python batch helpers | Additive preview fields + status buckets | Pass |
| CLIs | Messaging only; no new write commands | Pass |
| UI | Preview-only rollup additions | Pass |

---

## Verdict

**Pass** — PLAT-RT-F8 P2 may freeze; PLAT-RT-F8 becomes complete after P2.

