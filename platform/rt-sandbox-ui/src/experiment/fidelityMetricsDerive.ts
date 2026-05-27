import {
  experimentFidelityMetricsReportSchema,
  FIDELITY_METRICS_GOVERNANCE_BANNER,
  type ComparePairFidelity,
  type ExperimentFidelityMetricsReport,
  type ExperimentManifest,
  type ExperimentMetricsReport,
  type ExperimentRun,
  type ExperimentSpec,
  type FidelityContext,
  type PerRunFidelity,
  type RollupFidelity,
} from "./experimentSchema";
import type { StagingReader } from "./metricsDerive";
import { sha256Hex16 } from "./sha256Hex";
import { safeParseFidelityMetricsReport } from "./experimentImportGuards";

export const POSE_TRUTH_DRIFT_COMPARE_EPSILON_M = 0.001;

export const FORBIDDEN_FIDELITY_ROLLUP_KEYS = new Set([
  "success_rate",
  "winner",
  "best_run",
  "readiness_index",
]);

export type DeriveFidelityOptions = {
  spec?: ExperimentSpec;
  stagingReader?: StagingReader;
};

type TruthSnapshot = {
  attestation_status?: string;
  visibility_truth?: { ref?: string };
  los_truth?: { label?: string };
  dome_truth?: { sensor_id?: string };
};

type FidelityPoseBlock = {
  attestation_status?: string;
  per_entity?: Array<{
    pose_truth_drift_m?: number;
    sim_agl_m?: number;
  }>;
};

function sortKeysDeep(value: unknown): unknown {
  if (Array.isArray(value)) {
    return value.map(sortKeysDeep);
  }
  if (value && typeof value === "object") {
    const obj = value as Record<string, unknown>;
    const sorted: Record<string, unknown> = {};
    for (const key of Object.keys(obj).sort()) {
      sorted[key] = sortKeysDeep(obj[key]);
    }
    return sorted;
  }
  return value;
}

export function computeTruthFingerprint(
  fidelityContext: FidelityContext | undefined,
  truthSnapshot: TruthSnapshot | null,
): string | null {
  if (!fidelityContext?.enable_fidelity_coupling) return null;
  const payload = {
    enable_fidelity_coupling: fidelityContext.enable_fidelity_coupling,
    adapter_mode: fidelityContext.adapter_mode ?? null,
    visibility_truth_ref: truthSnapshot?.visibility_truth?.ref ?? null,
    dome_truth_ref: truthSnapshot?.dome_truth?.sensor_id ?? null,
  };
  const canonical = JSON.stringify(sortKeysDeep(payload));
  return `truth-fp-${sha256Hex16(canonical)}`;
}

function readJsonRef(reader: StagingReader | undefined, ref: string | null | undefined): unknown | null {
  if (!reader || !ref) return null;
  const text = reader(ref);
  if (!text) return null;
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
}

function readTruthSnapshot(
  run: ExperimentRun,
  reader?: StagingReader,
): TruthSnapshot | null {
  const ref = run.fidelity_context?.truth_snapshot_ref;
  const raw = readJsonRef(reader, ref);
  return raw && typeof raw === "object" ? (raw as TruthSnapshot) : null;
}

function readPoseBlock(
  run: ExperimentRun,
  reader?: StagingReader,
): FidelityPoseBlock | null {
  const stagingRef = run.capture_staging_ref;
  if (!reader || !stagingRef) return null;
  const raw = readJsonRef(reader, `${stagingRef}/normalized_manifest.json`);
  if (!raw || typeof raw !== "object") return null;
  const block = (raw as { fidelity_pose_block?: FidelityPoseBlock }).fidelity_pose_block;
  return block ?? null;
}

function maxNumeric(values: Array<number | undefined>): number | null {
  const nums = values.filter((v): v is number => typeof v === "number");
  if (nums.length === 0) return null;
  return Math.max(...nums);
}

function attestationStatus(
  truth: TruthSnapshot | null,
  poseBlock: FidelityPoseBlock | null,
  couplingOn: boolean,
): PerRunFidelity["fidelity_attestation_status"] {
  if (!couplingOn) return "unavailable";
  const fromTruth = truth?.attestation_status;
  if (fromTruth === "available" || fromTruth === "stale") return fromTruth;
  const fromBlock = poseBlock?.attestation_status;
  if (fromBlock === "available" || fromBlock === "stale") return fromBlock;
  if (truth || poseBlock) return "unavailable";
  return "unavailable";
}

export function perRunFidelityFromRun(
  run: ExperimentRun,
  stagingReader?: StagingReader,
): PerRunFidelity {
  const couplingOn = Boolean(run.fidelity_context?.enable_fidelity_coupling);
  const truth = readTruthSnapshot(run, stagingReader);
  const poseBlock = readPoseBlock(run, stagingReader);

  const losTruthLabel = couplingOn ? (truth?.los_truth?.label ?? null) : null;
  const losCognitionLabel = run.visibility_context?.los_cognition_label ?? null;

  const cognitionTruthDivergence =
    losTruthLabel != null &&
    losCognitionLabel != null &&
    losTruthLabel !== losCognitionLabel;

  return {
    run_id: run.run_id,
    fidelity_attestation_status: attestationStatus(truth, poseBlock, couplingOn),
    los_truth_label: losTruthLabel,
    los_cognition_label: losCognitionLabel,
    visibility_truth_ref: couplingOn ? (truth?.visibility_truth?.ref ?? null) : null,
    dome_truth_ref: couplingOn ? (truth?.dome_truth?.sensor_id ?? null) : null,
    pose_truth_drift_m: couplingOn
      ? maxNumeric((poseBlock?.per_entity ?? []).map((e) => e.pose_truth_drift_m))
      : null,
    agl_truth_m: couplingOn
      ? maxNumeric((poseBlock?.per_entity ?? []).map((e) => e.sim_agl_m))
      : null,
    cognition_truth_divergence: cognitionTruthDivergence,
  };
}

function countMapInc(map: Record<string, number>, key: string): void {
  map[key] = (map[key] ?? 0) + 1;
}

export function buildFidelityComparePairs(
  perRun: PerRunFidelity[],
): ComparePairFidelity[] {
  const sorted = [...perRun].sort((a, b) => a.run_id.localeCompare(b.run_id));
  const pairs: ComparePairFidelity[] = [];

  for (let i = 0; i < sorted.length; i += 1) {
    for (let j = i + 1; j < sorted.length; j += 1) {
      const a = sorted[i];
      const b = sorted[j];
      const badges: ComparePairFidelity["badges"] = [];

      if (a.cognition_truth_divergence || b.cognition_truth_divergence) {
        badges.push({
          id: "cognition_truth_divergence",
          label: "cognition_truth_divergence",
        });
      }

      if (
        a.los_truth_label != null &&
        b.los_truth_label != null &&
        a.los_truth_label !== b.los_truth_label
      ) {
        badges.push({ id: "los_truth_label_diff", label: "los_truth_label_diff" });
      }

      if (
        a.pose_truth_drift_m != null &&
        b.pose_truth_drift_m != null &&
        Math.abs(a.pose_truth_drift_m - b.pose_truth_drift_m) >
          POSE_TRUTH_DRIFT_COMPARE_EPSILON_M
      ) {
        badges.push({ id: "pose_truth_drift_delta", label: "pose_truth_drift_delta" });
      }

      const statusAsymmetric =
        (a.fidelity_attestation_status === "available") !==
        (b.fidelity_attestation_status === "available");
      if (statusAsymmetric) {
        badges.push({
          id: "fidelity_attestation_asymmetric",
          label: "fidelity_attestation_asymmetric",
        });
      }

      pairs.push({
        run_id_a: a.run_id,
        run_id_b: b.run_id,
        badges,
      });
    }
  }

  return pairs.sort(
    (x, y) => x.run_id_a.localeCompare(y.run_id_a) || x.run_id_b.localeCompare(y.run_id_b),
  );
}

export function rollupFidelity(
  perRun: PerRunFidelity[],
  manifest: ExperimentManifest,
  options?: DeriveFidelityOptions,
): RollupFidelity {
  const status_counts: Record<string, number> = {};
  let cognition_truth_divergence_count = 0;

  const fingerprintGroups = new Map<
    string,
    { spec_fingerprint: string; truth_fingerprint: string; run_count: number }
  >();

  for (const row of perRun) {
    countMapInc(status_counts, row.fidelity_attestation_status);
    if (row.cognition_truth_divergence) cognition_truth_divergence_count += 1;
  }

  for (const run of manifest.runs) {
    const row = perRun.find((r) => r.run_id === run.run_id);
    if (!row) continue;
    const specFp = run.spec_fingerprint ?? "unknown";
    const truth = readTruthSnapshot(run, options?.stagingReader);
    const truthFp =
      computeTruthFingerprint(run.fidelity_context, truth) ?? "unknown";
    const groupKey = `${specFp}::${truthFp}`;
    if (!fingerprintGroups.has(groupKey)) {
      fingerprintGroups.set(groupKey, {
        spec_fingerprint: specFp,
        truth_fingerprint: truthFp,
        run_count: 0,
      });
    }
    fingerprintGroups.get(groupKey)!.run_count += 1;
  }

  const couplingFlag = manifest.runs.some(
    (r) => r.fidelity_context?.enable_fidelity_coupling,
  );

  return {
    attestation_rollup: { status_counts },
    divergence_rollup: { cognition_truth_divergence_count },
    repeatability_truth_rollup: {
      truth_fingerprints: [...fingerprintGroups.values()]
        .sort(
          (a, b) =>
            a.spec_fingerprint.localeCompare(b.spec_fingerprint) ||
            a.truth_fingerprint.localeCompare(b.truth_fingerprint),
        )
        .map((g) => ({
          spec_fingerprint: g.spec_fingerprint,
          truth_fingerprint: g.truth_fingerprint,
          run_count: g.run_count,
          coupling_flag: couplingFlag,
        })),
    },
  };
}

function specFingerprintFromManifest(manifest: ExperimentManifest): string | null {
  for (const run of manifest.runs) {
    if (run.spec_fingerprint) return run.spec_fingerprint;
  }
  return null;
}

function couplingRequired(
  manifest: ExperimentManifest,
  spec?: ExperimentSpec,
): boolean {
  if (spec && (spec as ExperimentSpec & { enable_fidelity_coupling?: boolean }).enable_fidelity_coupling) {
    return true;
  }
  return manifest.runs.some((r) => r.fidelity_context?.enable_fidelity_coupling);
}

export function deriveExperimentFidelityMetrics(
  manifest: ExperimentManifest,
  _f5MetricsReport?: ExperimentMetricsReport,
  options?: DeriveFidelityOptions,
): ExperimentFidelityMetricsReport {
  const per_run_fidelity = manifest.runs
    .map((run) => perRunFidelityFromRun(run, options?.stagingReader))
    .sort((a, b) => a.run_id.localeCompare(b.run_id));

  const compare_pairs_fidelity = buildFidelityComparePairs(per_run_fidelity);
  const rollup_fidelity = rollupFidelity(per_run_fidelity, manifest, options);

  return {
    schema: "rt_experiment_fidelity_metrics_report_v1",
    experiment_id: manifest.experiment_id,
    governance_banner: FIDELITY_METRICS_GOVERNANCE_BANNER,
    spec_fingerprint: specFingerprintFromManifest(manifest),
    coupling_required: couplingRequired(manifest, options?.spec),
    per_run_fidelity,
    compare_pairs_fidelity,
    rollup_fidelity,
  };
}

export function exportFidelityMetricsJson(
  report: ExperimentFidelityMetricsReport,
): string {
  return JSON.stringify(
    { ...report, derived_at_utc: new Date().toISOString() },
    null,
    2,
  );
}

export function parseFidelityMetricsJson(text: string): ExperimentFidelityMetricsReport {
  const parsed = safeParseFidelityMetricsReport(text);
  if (!parsed.ok) throw new Error(parsed.error);
  return parsed.data;
}

export function validateFidelityMetricsReport(
  report: ExperimentFidelityMetricsReport,
): ExperimentFidelityMetricsReport {
  return experimentFidelityMetricsReportSchema.parse(report);
}
