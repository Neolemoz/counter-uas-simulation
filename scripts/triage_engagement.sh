#!/usr/bin/env bash
# Triage the engagement chain by grepping the most recent capture log for the six
# pipeline markers.  The first marker that reports 0 is the blocker; everything below
# that point is downstream and will not fire until the blocker is fixed.
#
# Usage:
#   scripts/triage_engagement.sh                # auto-pick newest runs/logs/*.log
#   scripts/triage_engagement.sh <log_path>     # explicit path

set -euo pipefail

LOG="${1:-}"
if [[ -z "${LOG}" ]]; then
    LOG=$(ls -t runs/logs/*.log 2>/dev/null | head -1 || true)
fi

if [[ -z "${LOG}" || ! -s "${LOG}" ]]; then
    echo "ERROR: no log file found (looked in runs/logs/*.log).  Run a capture first." >&2
    exit 2
fi

echo "=== Triage log: ${LOG} ($(wc -l <"${LOG}") lines) ==="

# Each marker is grouped by branch label (A..G) so the operator can map the first
# missing marker to the corresponding fix branch in the plan.
print_count() {
    local label="$1" pattern="$2"
    local n
    n=$(grep -c -E "${pattern}" "${LOG}" || true)
    printf '  %-48s %6d\n' "${label}" "${n}"
}

echo "--- Pipeline markers (top to bottom = order they should fire) ---"
print_count "A) sensor/fusion -> Candidate detected"     'Candidate detected:'
print_count "B) tracking -> Track created"               'Candidate confirmed.*Track created'
print_count "C) feasibility -> [FEASIBILITY] feasible=True" '\[FEASIBILITY\] feasible=True'
print_count "C') feasibility -> any [FEASIBILITY] line"  '\[FEASIBILITY\]'
print_count "D) selection -> [ASSIGN] interceptor"       '\[ASSIGN\] interceptor assignment'
print_count "E) controller -> [LAUNCH] interceptor_"     '\[LAUNCH\] interceptor_'
print_count "G) chatter   -> [STANDBY]"                  '\[STANDBY\]'
print_count "*) HIT recorded"                            '\[HIT\]'
print_count "*) GUIDANCE_WARN (no interceptor pos)"      '\[GUIDANCE_WARN\]'
print_count "*) HIT_BLOCKED_SAFETY"                      '\[HIT_BLOCKED_SAFETY\]'
print_count "*) FEAS_GUARD"                              '\[FEAS_GUARD\]'

echo "--- First feasibility line (if any) ---"
grep -m1 -E '\[FEASIBILITY\]' "${LOG}" || echo "  (none)"

echo "--- First track line (if any) ---"
grep -m1 -E 'Track created' "${LOG}" || echo "  (none)"

echo "--- Last 5 selection_id values published (if any) ---"
grep -E '\[ASSIGN\]|\[SWITCH\]|\[LAUNCH\]|\[STANDBY\]' "${LOG}" | tail -5 || echo "  (none)"

echo "--- Explosion / stop chain (single-target) ---"
print_count "/) HIT logs" '\[HIT\]'
print_count "/) interception HIT: publishing (stop burst)" 'HIT: publishing'
print_count "/) target Received /target/stop True" 'Received .*/target/stop.*True'
print_count "/) [EXPLOSION] sdf exists line" '\[EXPLOSION\] sdf_path='
print_count "/) gz create ok (ok_sp=true)" 'ok_sp=True'
print_count "/) spawned visual OK" 'spawned visual OK'
print_count "?) gz create FAILED warn" 'gz create explosion visual failed'
