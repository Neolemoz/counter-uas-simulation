#!/usr/bin/env bash
# Run scenario generalization MC for gate-only A vs frozen terminal+weak-PN D.
#
# Defaults are intentionally conservative and match the SG1 plan:
#   matrix: scripts/evaluation/fixtures/scenario_generalization_a_vs_d_matrix.csv
#   N:      40 per cell per arm
#   seeds:  9701..9740 per cell, matched between A and D
#
# Set DRY_RUN=1 to print commands without launching Gazebo.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-python3}"
MATRIX="${MATRIX:-scripts/evaluation/fixtures/scenario_generalization_a_vs_d_matrix.csv}"
N="${N:-40}"
SEED_BASE="${SEED_BASE:-9701}"
TIMEOUT_S="${TIMEOUT_S:-90}"
STUDY="${STUDY:-gnc_sg1}"
OUT_DIR="${OUT_DIR:-runs/mc}"
GIT_SHORT="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"

SPINE='use_gazebo_gui:=false eng_rollout_feasibility_gate:=true'
D_FRAGMENT='guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35 t_go_filter_max_step_s:=0 align_speed_use_smooth_hit_range:=false guidance_u_max_step_rad:=3.14159 use_pn_refinement:=true pn_blend_gain:=0.05 pn_blend_terminal_range_m:=1200'

run_arm() {
  local arm="$1"
  local cell_tag="$2"
  local cell_args="$3"
  local arm_fragment=""

  if [[ "$arm" == "D" ]]; then
    arm_fragment="$D_FRAGMENT"
  fi

  local label="${STUDY}_${arm}_${cell_tag}_n${N}_s${SEED_BASE}"
  local cohort="${STUDY}_${arm}_${cell_tag}_gs${GIT_SHORT}"
  local launch_args="$SPINE $cell_args $arm_fragment"
  local cmd=(
    "$PY" scripts/monte_carlo.py run
    --n "$N"
    --seed-base "$SEED_BASE"
    --scenario single
    --timeout-s "$TIMEOUT_S"
    --launch-args "$launch_args"
    --label "$label"
    --cohort "$cohort"
    --out-dir "$OUT_DIR"
  )

  printf '[sg1] arm=%s cell=%s cohort=%s\n' "$arm" "$cell_tag" "$cohort"
  if [[ "${DRY_RUN:-}" == "1" ]]; then
    printf '  '
    printf '%q ' "${cmd[@]}"
    printf '\n'
  else
    "${cmd[@]}"
  fi
}

if [[ ! -f "$MATRIX" ]]; then
  echo "matrix not found: $MATRIX" >&2
  exit 2
fi

if [[ "${DRY_RUN:-}" != "1" && ! -f "$ROOT/install/setup.bash" ]]; then
  echo "Missing install/setup.bash (run colcon build first)." >&2
  exit 2
fi

_trim() {
  local s="${1:-}"
  s="${s//$'\r'/}"
  s="${s#"${s%%[![:space:]]*}"}"
  s="${s%"${s##*[![:space:]]}"}"
  printf '%s' "$s"
}

while IFS=, read -r cell_tag los dive layout notes_meta; do
  cell_tag="$(_trim "$cell_tag")"
  [[ -z "${cell_tag:-}" || "$cell_tag" == "cell_tag" ]] && continue
  los="$(_trim "$los")"
  dive="$(_trim "$dive")"
  layout="$(_trim "$layout")"
  cell_args="target_los_closing_m_s:=$los target_los_dive_gain:=$dive interceptor_ic_layout:=$layout"
  run_arm A "$cell_tag" "$cell_args"
  run_arm D "$cell_tag" "$cell_args"
done < "$MATRIX"
