# Demo case: governance_warnings

## What this demonstrates

How to read **warning panels** and `governance-lint` output when lineage is incomplete. Warnings are interpretation prompts — not severity, readiness, or approval status.

## What reviewers should look for

- Warnings array in observability bundle and narrative JSON
- Missing meta, seed, cohort, or git fields flagged explicitly
- `governance-lint` issues list — wording gaps vs data gaps
- HTML warning panel rendered in composite report
- Demonstrate **stopping interpretation** when lineage is insufficient

## What NOT to conclude

- “Failed validation” or “bad system” from warnings
- Approval or certification from `governance-lint ok: true`
- That fabricating seed/profile id is acceptable to “clean up” a demo
- Operational status from lint output

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| lineage appendix | Missing field visibility |
| HTML warning panel | Mentor teaching surface |
| observability JSON → warnings | Raw prompt list |
| governance-lint JSON | Wording contract check |
| Minimal fixture HTML (optional) | Static viz without full log |

## Governance caveats

- Lint checks caveat presence — not correctness, comparability, or approval.
- Test fixture `src/counter_uas/test/fixtures/replay_narrative_minimal.json` is valid for viz-only walkthrough; state log re-read limits.

## Suggested run source

- Redacted or synthetic run with missing meta fields, seed, or git state
- Or: minimal test fixture for static viz-only walkthrough (no live log)

## Fixture hints

- Run lint: `python3 scripts/evaluation/replay_observability.py governance-lint <json>`
- Generate observability from unit-test synthetic log pattern in `test_replay_observability.py`

## README_case.md (copy to demo bundle)

1. Warnings mean **review limitations** — not “failed validation” or “bad system.”
2. `governance-lint ok: true` checks wording presence — not correctness or approval.
3. Never fabricate seed, cohort, or profile id to clear warnings.
4. Teach recipients to stop interpretation when lineage is insufficient.
5. Close: governance discipline demo; not operational status.
