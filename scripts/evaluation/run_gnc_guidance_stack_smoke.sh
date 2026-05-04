#!/usr/bin/env bash
# Paired Monte Carlo smoke: rollout gate spine vs stacked guidance knobs (Steps 1–3).
# Set SKIP_GAZEBO=1 to no-op (e.g. in CI without Gazebo install).
#
# Usage: from repo root
#   bash scripts/evaluation/run_gnc_guidance_stack_smoke.sh

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

if [[ -n "${SKIP_GAZEBO:-}" ]]; then
  echo '[run_gnc_guidance_stack_smoke] SKIP_GAZEBO set — skipping Monte Carlo launches.'
  exit 0
fi

test -f install/setup.bash || {
  echo 'Missing install/setup.bash — build the workspace first (colcon build).' >&2
  exit 2
}

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
PY="${PYTHON:-python3}"

BASE_LAUNCH='use_gazebo_gui:=false eng_rollout_feasibility_gate:=true'
STEP123_STACK="${BASE_LAUNCH} guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35 t_go_filter_max_step_s:=0.8 align_speed_use_smooth_hit_range:=true guidance_u_max_step_rad:=0.45 use_pn_refinement:=true pn_blend_gain:=0.12 pn_blend_terminal_range_m:=1500"

echo '[smoke] arm A: gate spine only'
"$PY" scripts/monte_carlo.py run --n "${GNC_SMOKE_N:-3}" --seed-base "${GNC_SMOKE_SEED:-9401}" \
  --scenario single --timeout-s "${GNC_SMOKE_TIMEOUT:-75}" \
  --label gnc_smoke_gate --cohort gnc_smoke_A_gate --launch-args "$BASE_LAUNCH" --out-dir runs/mc

echo '[smoke] arm B: gate + stacked guidance knobs'
"$PY" scripts/monte_carlo.py run --n "${GNC_SMOKE_N:-3}" --seed-base "${GNC_SMOKE_SEED:-9401}" \
  --scenario single --timeout-s "${GNC_SMOKE_TIMEOUT:-75}" \
  --label gnc_smoke_guidance_stack --cohort gnc_smoke_B_stack --launch-args "$STEP123_STACK" --out-dir runs/mc

"$PY" scripts/monte_carlo.py compare \
  --inputs runs/mc/gnc_smoke_gate.json runs/mc/gnc_smoke_guidance_stack.json

echo '[run_gnc_guidance_stack_smoke] done'
