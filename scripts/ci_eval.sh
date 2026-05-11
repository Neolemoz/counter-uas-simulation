#!/usr/bin/env bash
# Efficient evaluation tiers: fast gates (Tier0), Gazebo smoke (Tier1),
# scenario matrix DOE (Tier2), Monte Carlo aggregate/run (Tier3).
# Usage: scripts/ci_eval.sh <command> [options]
# Run from repo root (or any cwd; paths are resolved from this script).

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-python3}"

die() {
  echo "ci_eval: $*" >&2
  exit 2
}

usage() {
  cat <<'EOF'
ci_eval — layered counter-UAS evaluation (see scripts/evaluation/README.md).

Commands (run from repo root):
  tier0              Fast gate: pytest + validate_heatmap --dry-run (no Gazebo).
  tier1              One Gazebo capture (headless). Needs install/setup.bash.
  tier2              Full scenario matrix from default CSV (slow).
  tier2-smoke        Single-row matrix (smoke_scenario_matrix.csv).
  tier3-aggregate    Summarize existing logs with monte_carlo aggregate.
  tier3-run          Run N captures + MC summary (slow).

Environment:
  PYTHON             Python interpreter (default: python3).
  LIBGL_ALWAYS_SOFTWARE  If unset, tier1/tier2/tier3-run default to 1 for headless GL.

Examples:
  scripts/ci_eval.sh tier0
  scripts/ci_eval.sh tier1 --timeout-s 180
  scripts/ci_eval.sh tier2 --timeout-s 200 --out-csv runs/evaluation/my_matrix.csv
  scripts/ci_eval.sh tier2-smoke --timeout-s 120
  scripts/ci_eval.sh tier3-aggregate --label nightly --pattern '2026-*_single.log'
  scripts/ci_eval.sh tier3-aggregate --label cohort --meta-cohort my_matrix_v1 --pattern '*.log'
  scripts/ci_eval.sh tier3-run --n 5 --timeout-s 60 --launch-args 'use_gazebo_gui:=false'

Notes:
  run_capture exit 124 means coreutils timeout cut the sim; widen --timeout-s for full engagement.
EOF
}

require_install() {
  test -f "$ROOT/install/setup.bash" || die "Missing install/setup.bash — run colcon build first."
}

tier0() {
  echo "[ci_eval] tier0: pytest"
  "$PY" -m pytest src/counter_uas/test/ -q --tb=short
  echo "[ci_eval] tier0: validate_heatmap_vs_gazebo --dry-run"
  "$PY" scripts/validate_heatmap_vs_gazebo.py \
    --heatmap-csv scripts/evaluation/fixtures/sample_heatmap_for_validate.csv \
    --dry-run --seed 1
  echo "[ci_eval] tier0: OK"
}

export_libgl_default() {
  if [[ -z "${LIBGL_ALWAYS_SOFTWARE+x}" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
  fi
}

tier1() {
  require_install
  export_libgl_default
  local timeout_s="${TIER1_TIMEOUT:-180}"
  local launch_extra="${TIER1_LAUNCH_ARGS:-use_gazebo_gui:=false}"
  echo "[ci_eval] tier1: run_capture timeout_s=${timeout_s} launch_args=${launch_extra}"
  "$PY" scripts/run_capture.py --scenario single --timeout-s "$timeout_s" --launch-args "$launch_extra"
  # run_capture exits 0 even when inner timeout returns 124; inspect log + .meta.json under runs/logs/
  echo "[ci_eval] tier1: done (inspect printed paths above)"
}

tier2() {
  require_install
  export_libgl_default
  local csv="${MATRIX_CSV:-scripts/evaluation/fixtures/threat_spawn_matrix_example.csv}"
  local out="${OUT_CSV:-runs/evaluation/ci_eval_matrix_latest.csv}"
  local scenario="${SCENARIO:-single}"
  local timeout_s="${TIMEOUT_S:-180}"
  echo "[ci_eval] tier2: matrix-csv=${csv} out=${out} scenario=${scenario} timeout_s=${timeout_s}"
  "$PY" scripts/run_scenario_matrix.py \
    --matrix-csv "$csv" \
    --out-csv "$out" \
    --scenario "$scenario" \
    --timeout-s "$timeout_s"
  echo "[ci_eval] tier2: wrote ${out}"
}

tier2_smoke() {
  MATRIX_CSV="$ROOT/scripts/evaluation/fixtures/smoke_scenario_matrix.csv" \
    OUT_CSV="${OUT_CSV:-$ROOT/runs/evaluation/ci_eval_matrix_smoke_latest.csv}" \
    TIMEOUT_S="${TIMEOUT_S:-120}" \
    SCENARIO="${SCENARIO:-single}" \
    tier2
}

tier3_aggregate() {
  local logs_dir="${LOGS_DIR:-$ROOT/runs/logs}"
  local pattern="${PATTERN:-*.log}"
  local label="${LABEL:-ci_aggregate}"
  local out_dir="${OUT_DIR:-$ROOT/runs/mc}"
  local mc_extra=()
  if [[ -n "${META_COHORT:-}" ]]; then
    mc_extra+=(--meta-cohort "${META_COHORT}")
  fi
  if [[ -n "${NOTES_SUBSTRING:-}" ]]; then
    mc_extra+=(--notes-substring "${NOTES_SUBSTRING}")
  fi
  echo "[ci_eval] tier3-aggregate: logs-dir=${logs_dir} pattern=${pattern} label=${label}"
  "$PY" scripts/monte_carlo.py aggregate \
    --logs-dir "$logs_dir" \
    --pattern "$pattern" \
    --label "$label" \
    --out-dir "$out_dir" \
    "${mc_extra[@]:-}"
  echo "[ci_eval] tier3-aggregate: OK (see ${out_dir}/${label}.json)"
}

tier3_run() {
  require_install
  export_libgl_default
  local n="${N_RUNS:-10}"
  local timeout_s="${TIMEOUT_S:-60}"
  local scenario="${SCENARIO:-single}"
  local label="${LABEL:-ci_mc_run}"
  local out_dir="${OUT_DIR:-$ROOT/runs/mc}"
  local launch="${LAUNCH_ARGS:-use_gazebo_gui:=false}"
  local seed_base="${SEED_BASE:-1}"
  local geometry="${GEOMETRY_ID:-}"
  local geo_args=()
  if [[ -n "$geometry" ]]; then
    geo_args=(--geometry-id "$geometry")
  fi
  local cohort_extra=()
  if [[ -n "${MC_COHORT:-}" ]]; then
    cohort_extra=(--cohort "${MC_COHORT}")
  fi
  echo "[ci_eval] tier3-run: n=${n} timeout_s=${timeout_s} label=${label}"
  "$PY" scripts/monte_carlo.py run \
    --n "$n" \
    --timeout-s "$timeout_s" \
    --scenario "$scenario" \
    --label "$label" \
    --out-dir "$out_dir" \
    --seed-base "$seed_base" \
    "${geo_args[@]}" \
    "${cohort_extra[@]:-}" \
    --launch-args "$launch"
  echo "[ci_eval] tier3-run: OK (see ${out_dir}/${label}.json)"
}

# Optional passthrough for tier1/tier3-run from argv after command
parse_tier1_args() {
  TIER1_TIMEOUT=180
  TIER1_LAUNCH_ARGS="use_gazebo_gui:=false"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --timeout-s)
        TIER1_TIMEOUT="$2"
        shift 2
        ;;
      --launch-args)
        TIER1_LAUNCH_ARGS="$2"
        shift 2
        ;;
      *)
        die "unknown tier1 flag: $1"
        ;;
    esac
  done
  tier1
}

parse_tier2_flags() {
  # Clear so empty ${VAR:-default} applies when a flag is omitted.
  MATRIX_CSV=""
  OUT_CSV=""
  SCENARIO=""
  TIMEOUT_S=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --matrix-csv)
        MATRIX_CSV="$2"
        shift 2
        ;;
      --out-csv)
        OUT_CSV="$2"
        shift 2
        ;;
      --scenario)
        SCENARIO="$2"
        shift 2
        ;;
      --timeout-s)
        TIMEOUT_S="$2"
        shift 2
        ;;
      *)
        die "unknown tier2 flag: $1"
        ;;
    esac
  done
}

parse_tier3_agg_args() {
  META_COHORT=""
  NOTES_SUBSTRING=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --logs-dir)
        LOGS_DIR="$2"
        shift 2
        ;;
      --pattern)
        PATTERN="$2"
        shift 2
        ;;
      --label)
        LABEL="$2"
        shift 2
        ;;
      --out-dir)
        OUT_DIR="$2"
        shift 2
        ;;
      --meta-cohort)
        META_COHORT="$2"
        shift 2
        ;;
      --notes-substring)
        NOTES_SUBSTRING="$2"
        shift 2
        ;;
      *)
        die "unknown tier3-aggregate flag: $1"
        ;;
    esac
  done
  tier3_aggregate
}

parse_tier3_run_args() {
  N_RUNS=10
  TIMEOUT_S=60
  SCENARIO=single
  LABEL=ci_mc_run
  OUT_DIR="$ROOT/runs/mc"
  SEED_BASE=1
  LAUNCH_ARGS="use_gazebo_gui:=false"
  GEOMETRY_ID=""
  MC_COHORT=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --n)
        N_RUNS="$2"
        shift 2
        ;;
      --timeout-s)
        TIMEOUT_S="$2"
        shift 2
        ;;
      --scenario)
        SCENARIO="$2"
        shift 2
        ;;
      --label)
        LABEL="$2"
        shift 2
        ;;
      --out-dir)
        OUT_DIR="$2"
        shift 2
        ;;
      --seed-base)
        SEED_BASE="$2"
        shift 2
        ;;
      --launch-args)
        LAUNCH_ARGS="$2"
        shift 2
        ;;
      --geometry-id)
        GEOMETRY_ID="$2"
        shift 2
        ;;
      --cohort)
        MC_COHORT="$2"
        shift 2
        ;;
      *)
        die "unknown tier3-run flag: $1"
        ;;
    esac
  done
  tier3_run
}

cmd="${1:-}"
shift || true
case "$cmd" in
  ""|-h|--help|help)
    usage
    [[ -n "$cmd" ]] || exit 0
    exit 0
    ;;
  tier0)
    tier0
    ;;
  tier1)
    parse_tier1_args "$@"
    ;;
  tier2)
    parse_tier2_flags "$@"
    tier2
    ;;
  tier2-smoke)
    parse_tier2_flags "$@"
    tier2_smoke
    ;;
  tier3-aggregate)
    parse_tier3_agg_args "$@"
    ;;
  tier3-run)
    parse_tier3_run_args "$@"
    ;;
  *)
    die "unknown command: ${cmd:-}. Try: scripts/ci_eval.sh --help"
    ;;
esac
