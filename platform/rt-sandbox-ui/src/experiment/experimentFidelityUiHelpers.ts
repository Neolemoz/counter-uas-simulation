import type { ComparePairFidelity, PerRunFidelity } from "./experimentSchema";

const FIDELITY_COMPARE_BADGE_LABELS: Record<string, string> = {
  cognition_truth_divergence: "cognition_truth_divergence",
  pose_truth_drift_delta: "pose_truth_drift_delta",
  los_truth_label_diff: "los_truth_label_diff",
  fidelity_attestation_asymmetric: "fidelity_attestation_asymmetric",
};

export function fidelityCompareBadgeLabel(id: string): string {
  return FIDELITY_COMPARE_BADGE_LABELS[id] ?? id;
}

export function attestationFreshnessBadge(
  status: PerRunFidelity["fidelity_attestation_status"],
): { label: string; tone: "ok" | "warn" | "muted" } {
  switch (status) {
    case "available":
      return { label: "truth_attested", tone: "ok" };
    case "stale":
      return { label: "stale", tone: "warn" };
    default:
      return { label: "unavailable", tone: "muted" };
  }
}

export function truthVsExplanatorySummary(row: PerRunFidelity): string {
  const truth = row.los_truth_label ?? "—";
  const explanatory = row.los_cognition_label ?? "—";
  return `LOS truth ${truth} · cognition ${explanatory}`;
}

export function driftSummary(row: PerRunFidelity): string {
  const drift =
    row.pose_truth_drift_m != null ? `${row.pose_truth_drift_m.toFixed(2)}m` : "—";
  const agl = row.agl_truth_m != null ? `${row.agl_truth_m.toFixed(1)}m` : "—";
  return `drift ${drift} · sim AGL ${agl}`;
}

export function fidelityRepeatabilitySummary(
  truthFingerprints: Array<{ truth_fingerprint: string; run_count: number }>,
): string | null {
  if (truthFingerprints.length === 0) return null;
  const parts = truthFingerprints.map(
    (fp) => `${fp.truth_fingerprint} ×${fp.run_count}`,
  );
  return `truth fingerprints: ${parts.join("; ")}`;
}

export function pairBadgesForRuns(
  runIdA: string,
  runIdB: string,
  pairs: ComparePairFidelity[],
): ComparePairFidelity["badges"] {
  const pair = pairs.find(
    (p) =>
      (p.run_id_a === runIdA && p.run_id_b === runIdB) ||
      (p.run_id_a === runIdB && p.run_id_b === runIdA),
  );
  return pair?.badges ?? [];
}
