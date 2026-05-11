#!/usr/bin/env bash
# Physics-aware guidance tuning playbook driver (Phases 0–4).
# See docs/evaluation/gnc_guidance_tuning_playbook_ops.md
#
# Usage (repo root):
#   bash scripts/evaluation/run_gnc_tuning_playbook.sh reference
#   bash scripts/evaluation/run_gnc_tuning_playbook.sh phase1
#   TUNING_QUICK=1 bash scripts/evaluation/run_gnc_tuning_playbook.sh all
#
# Env:
#   SKIP_GAZEBO=1          — no MC launches
#   TUNING_QUICK=1         — TUNING_PILOT_N=2, TUNING_TIMEOUT_S=60
#   TUNING_PILOT_N         — runs per arm (default 15)
#   TUNING_SEED_BASE       — default 9501
#   TUNING_TIMEOUT_S       — run_capture timeout (default 90)
#   PHASE1_APPEND          — winning fragment after phase1 (default mid bracket)
#   PHASE2_APPEND          — after phase2
#   PHASE3_APPEND          — after phase3

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
PY="${PYTHON:-python3}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

SPINE='use_gazebo_gui:=false eng_rollout_feasibility_gate:=true'

if [[ "${TUNING_QUICK:-}" == 1 ]]; then
  export TUNING_PILOT_N="${TUNING_PILOT_N:-2}"
  export TUNING_TIMEOUT_S="${TUNING_TIMEOUT_S:-60}"
else
  export TUNING_PILOT_N="${TUNING_PILOT_N:-15}"
  export TUNING_TIMEOUT_S="${TUNING_TIMEOUT_S:-90}"
fi
TUNING_SEED_BASE="${TUNING_SEED_BASE:-9501}"
OUT_MC="${OUT_MC:-runs/mc}"
FIXTURE="$ROOT/scripts/evaluation/fixtures"

_git_short() {
  git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown
}

require_gazebo() {
  if [[ -n "${SKIP_GAZEBO:-}" ]]; then
    echo '[tuning_playbook] SKIP_GAZEBO set — skipping MC.'
    exit 0
  fi
  test -f "$ROOT/install/setup.bash" || {
    echo 'Missing install/setup.bash (colcon build first).' >&2
    exit 2
  }
}

cmd_reference() {
  mkdir -p "$FIXTURE"
  local gs
  gs="$(_git_short)"
  local utc
  utc="$(date -u +%Y-%m-%dT%H-%M-%SZ)"
  cat >"$FIXTURE/gnc_tuning_phase0_reference.txt" <<EOF
# Phase 0 — frozen tuning reference (physics-aware playbook)
created_utc=$utc
git_short=$gs

spine_launch_args=
  $SPINE

cohort_example=
  gnc_tune_p0_reference_${gs}

instructions=
  Phase 1+ arms MUST use identical spine plus only declared deltas per playbook.
EOF
  echo "[tuning_playbook] wrote $FIXTURE/gnc_tuning_phase0_reference.txt"
}

_run_mc() {
  local label=$1 cohort=$2 launch_fragment=$3 n=${4:-$TUNING_PILOT_N}
  echo "[tuning_playbook] mc label=$label cohort=$cohort n=$n"
  "$PY" scripts/monte_carlo.py run \
    --n "$n" \
    --seed-base "$TUNING_SEED_BASE" \
    --scenario single \
    --timeout-s "$TUNING_TIMEOUT_S" \
    --label "$label" \
    --cohort "$cohort" \
    --launch-args "$SPINE $launch_fragment" \
    --out-dir "$OUT_MC"
}

cmd_confirm_phase1_arm() {
  require_gazebo
  local frag=${1:?fragment}
  local cohort=${2:?cohort}
  local n=${TUNING_CONFIRM_N:-40}
  echo "[tuning_playbook] CONFIRM Phase1 arm n=$n cohort=$cohort"
  "$PY" scripts/monte_carlo.py run \
    --n "$n" \
    --seed-base "${TUNING_CONFIRM_SEED:-$TUNING_SEED_BASE}" \
    --scenario single \
    --timeout-s "$TUNING_TIMEOUT_S" \
    --label "tune_p1_confirm_${cohort}" \
    --cohort "$cohort" \
    --launch-args "$SPINE $frag" \
    --out-dir "$OUT_MC"
}

cmd_phase1() {
  require_gazebo
  local gs
  gs="$(_git_short)"
  # Bracket terminal range @ fixed blend 0.35 (playbook §Phase 1)
  _run_mc tune_p1_r400  "gnc_tune_p1_r400_b035_${gs}"       'guidance_terminal_range_m:=400 guidance_terminal_pursuit_blend:=0.35'
  _run_mc tune_p1_r650  "gnc_tune_p1_r650_b035_${gs}"       'guidance_terminal_range_m:=650 guidance_terminal_pursuit_blend:=0.35'
  _run_mc tune_p1_r800  "gnc_tune_p1_r800_b035_${gs}"       'guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35'
  echo '[tuning_playbook] Phase1 done. Compare runs/mc/tune_p1_r*.json — pick winner → export PHASE1_APPEND'
}

cmd_phase2() {
  require_gazebo
  local gs
  gs="$(_git_short)"
  local p1="${PHASE1_APPEND:-guidance_terminal_range_m:=650 guidance_terminal_pursuit_blend:=0.35}"
  # Single-axis: max_step coarse (alpha fixed default in node ~0.5)
  _run_mc tune_p2_ms0    "gnc_tune_p2_mstep0_${gs}"        "$p1 t_go_filter_max_step_s:=0.0"
  _run_mc tune_p2_ms08   "gnc_tune_p2_mstep08_${gs}"       "$p1 t_go_filter_max_step_s:=0.8"
  _run_mc tune_p2_ms12   "gnc_tune_p2_mstep12_${gs}"       "$p1 t_go_filter_max_step_s:=1.2"
  echo '[tuning_playbook] Phase2 done. Tune alpha separately if jitter remains (manual arm).'
}

cmd_phase3() {
  require_gazebo
  local gs
  gs="$(_git_short)"
  local p2="${PHASE2_APPEND:-${PHASE1_APPEND:-guidance_terminal_range_m:=650 guidance_terminal_pursuit_blend:=0.35} t_go_filter_max_step_s:=0.8}"
  _run_mc tune_p3_smooth0_slew_pi   "gnc_tune_p3_sm0_spi_${gs}"     "$p2 align_speed_use_smooth_hit_range:=false guidance_u_max_step_rad:=$(python3 -c 'import math; print(math.pi)')"
  _run_mc tune_p3_smooth1_slew_pi   "gnc_tune_p3_sm1_spi_${gs}"     "$p2 align_speed_use_smooth_hit_range:=true guidance_u_max_step_rad:=$(python3 -c 'import math; print(math.pi)')"
  _run_mc tune_p3_smooth1_slew045   "gnc_tune_p3_sm1_s045_${gs}"    "$p2 align_speed_use_smooth_hit_range:=true guidance_u_max_step_rad:=0.45"
}

cmd_phase4() {
  require_gazebo
  local gs
  gs="$(_git_short)"
  local p3="${PHASE3_APPEND:-${PHASE2_APPEND:-${PHASE1_APPEND:-guidance_terminal_range_m:=650 guidance_terminal_pursuit_blend:=0.35} t_go_filter_max_step_s:=0.8} align_speed_use_smooth_hit_range:=true guidance_u_max_step_rad:=0.45}"
  _run_mc tune_p4_nopn   "gnc_tune_p4_nopn_${gs}"    "$p3 use_pn_refinement:=false pn_blend_gain:=0.0 pn_blend_terminal_range_m:=0.0"
  _run_mc tune_p4_pn     "gnc_tune_p4_pn_${gs}"      "$p3 use_pn_refinement:=true pn_blend_gain:=0.08 pn_blend_terminal_range_m:=1400"
  _run_mc tune_p4_pn2    "gnc_tune_p4_pn2_${gs}"     "$p3 use_pn_refinement:=true pn_blend_gain:=0.10 pn_blend_terminal_range_m:=1800"
}

cmd_compare_phase1() {
  local gs
  gs="$(_git_short)"
  "$PY" scripts/monte_carlo.py compare \
    --inputs \
    "$OUT_MC/tune_p1_r400.json" \
    "$OUT_MC/tune_p1_r650.json" \
    "$OUT_MC/tune_p1_r800.json" \
    2>/dev/null || echo '[tuning_playbook] compare: run phase1 first'
}

cmd_all() {
  cmd_reference
  require_gazebo
  cmd_phase1
  cmd_compare_phase1
  cmd_phase2
  cmd_phase3
  cmd_phase4
  echo '[tuning_playbook] ALL phases finished (see runs/mc/tune_p*.json).'
}

usage() {
  cat <<USAGE
Commands: reference | phase1 | phase2 | phase3 | phase4 | compare_phase1 | confirm_phase1_arm <launch_fragment> <cohort> | all
USAGE
}

main() {
  case "${1:-}" in
    reference) cmd_reference ;;
    phase1) cmd_phase1 ;;
    phase2) cmd_phase2 ;;
    phase3) cmd_phase3 ;;
    phase4) cmd_phase4 ;;
    compare_phase1) cmd_compare_phase1 ;;
    confirm_phase1_arm) shift; cmd_confirm_phase1_arm "$@" ;;
    all) cmd_all ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
