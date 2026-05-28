import type { CompareStatusId } from "./compareStatusVocabulary";
import type { MultiManifestDiffRow } from "./multiManifestDiff";

export const MULTI_MANIFEST_DIFF_COLUMNS = [
  { id: "field", label: "Field" },
  { id: "primary", label: "Primary" },
  { id: "secondary", label: "Secondary" },
  { id: "status", label: "Status" },
] as const;

export const MULTI_MANIFEST_FIELD_ORDER = [
  "experiment_id",
  "runs.length",
  "experiment_class",
  "spec_fingerprint",
  "capture_count",
  "tag_overlap",
] as const;

export function sortMultiManifestRows(rows: MultiManifestDiffRow[]): MultiManifestDiffRow[] {
  const order = new Map(
    MULTI_MANIFEST_FIELD_ORDER.map((field, index) => [field, index]),
  );
  return [...rows].sort((a, b) => {
    const ai = order.get(a.field as (typeof MULTI_MANIFEST_FIELD_ORDER)[number]) ?? 999;
    const bi = order.get(b.field as (typeof MULTI_MANIFEST_FIELD_ORDER)[number]) ?? 999;
    return ai - bi;
  });
}

export function truncateManifestRef(ref: string, maxLen = 28): string {
  if (ref.length <= maxLen) return ref;
  return `${ref.slice(0, maxLen - 1)}…`;
}

export function summarizeMultiManifestDiffStatus(rows: MultiManifestDiffRow[]): string {
  const counts: Record<CompareStatusId, number> = {
    aligned: 0,
    divergent: 0,
    missing: 0,
    explanatory: 0,
    not_comparable: 0,
  };
  for (const row of rows) {
    counts[row.compare_status] += 1;
  }
  const parts: string[] = [];
  if (counts.aligned > 0) parts.push(`${counts.aligned} aligned`);
  if (counts.divergent > 0) parts.push(`${counts.divergent} divergent`);
  if (counts.missing > 0) parts.push(`${counts.missing} missing`);
  if (counts.explanatory > 0) parts.push(`${counts.explanatory} explanatory`);
  return parts.length > 0 ? parts.join(", ") : "no rows";
}
