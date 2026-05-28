import type { CompareStatusId } from "./compareStatusVocabulary";
import { deriveRowCompareStatus } from "./compareStatusVocabulary";
import type { CohortManifestRef, ExperimentCohortIndex } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";

export const MULTI_MANIFEST_DIFF_BANNER =
  "Manifest metadata compare — not run outcome compare";

export type MultiManifestDiffSource = "cohort_index" | "loaded_manifest" | "overlap";

export type MultiManifestDiffRow = {
  field: string;
  primary: string;
  secondary: string;
  source: MultiManifestDiffSource;
  note?: string;
  compare_status: CompareStatusId;
};

export type ManifestSummaryChip = {
  role: "primary" | "secondary";
  experiment_id: string;
  runsLabel: string;
  experiment_class: string;
  manifest_ref: string;
};

function refByManifestRef(
  cohort: ExperimentCohortIndex | null,
  manifestRef: string | null,
): CohortManifestRef | null {
  if (!cohort || !manifestRef) return null;
  return cohort.manifest_refs.find((r) => r.manifest_ref === manifestRef) ?? null;
}

function uniqueSpecFingerprints(manifest: ExperimentManifest): string[] {
  const set = new Set<string>();
  for (const run of manifest.runs) {
    if (run.spec_fingerprint) set.add(run.spec_fingerprint);
  }
  return [...set].sort();
}

function captureCount(manifest: ExperimentManifest): number {
  return manifest.runs.filter((r) => r.capture_candidate_id).length;
}

function runsLengthLabel(
  ref: CohortManifestRef,
  loaded: ExperimentManifest | null,
  matched: boolean,
): { primary: string; note?: string } {
  const hint = ref.run_count_hint != null ? String(ref.run_count_hint) : "—";
  if (!matched || !loaded) {
    return { primary: hint };
  }
  const loadedLen = String(loaded.runs.length);
  if (ref.run_count_hint != null && ref.run_count_hint !== loaded.runs.length) {
    return {
      primary: loadedLen,
      note: `cohort hint ${hint}`,
    };
  }
  return { primary: loadedLen };
}

function supplementClass(ref: CohortManifestRef, loaded: ExperimentManifest | null): string {
  if (ref.experiment_class) return ref.experiment_class;
  if (loaded) {
    const fromRun = loaded.runs.find((r) => r.experiment_class)?.experiment_class;
    if (fromRun) return fromRun;
  }
  return "—";
}

function specFingerprintLabel(
  ref: CohortManifestRef,
  loaded: ExperimentManifest | null,
  matched: boolean,
): { value: string; source: MultiManifestDiffSource } {
  if (matched && loaded) {
    const fps = uniqueSpecFingerprints(loaded);
    if (fps.length > 0) {
      return { value: fps.join(", "), source: "loaded_manifest" };
    }
  }
  if (ref.spec_fingerprint) {
    return { value: ref.spec_fingerprint, source: "cohort_index" };
  }
  return { value: "—", source: "cohort_index" };
}

function tagOverlapLabel(
  cohort: ExperimentCohortIndex,
  primaryRef: CohortManifestRef,
  secondaryRef: CohortManifestRef,
): string {
  const cohortTags = new Set(cohort.tags ?? []);
  const refTags = new Set<string>();
  if (primaryRef.experiment_class) refTags.add(primaryRef.experiment_class);
  if (secondaryRef.experiment_class) refTags.add(secondaryRef.experiment_class);
  if (cohortTags.size === 0 && refTags.size === 0) return "—";
  const overlap = [...cohortTags].filter((t) => refTags.has(t));
  if (overlap.length > 0) return overlap.join(", ");
  if (cohortTags.size === 0) {
    return `ref classes: ${[...refTags].join(", ")} (no cohort tags)`;
  }
  return "none";
}

export function buildManifestSummaryChips(options: {
  primaryRef: string | null;
  secondaryRef: string | null;
  cohort: ExperimentCohortIndex | null;
}): ManifestSummaryChip[] {
  const { primaryRef, secondaryRef, cohort } = options;
  const chips: ManifestSummaryChip[] = [];
  const primary = refByManifestRef(cohort, primaryRef);
  const secondary = refByManifestRef(cohort, secondaryRef);
  if (primary) {
    chips.push({
      role: "primary",
      experiment_id: primary.experiment_id,
      runsLabel:
        primary.run_count_hint != null ? `~${primary.run_count_hint} runs` : "runs ?",
      experiment_class: primary.experiment_class ?? "—",
      manifest_ref: primary.manifest_ref,
    });
  }
  if (secondary) {
    chips.push({
      role: "secondary",
      experiment_id: secondary.experiment_id,
      runsLabel:
        secondary.run_count_hint != null ? `~${secondary.run_count_hint} runs` : "runs ?",
      experiment_class: secondary.experiment_class ?? "—",
      manifest_ref: secondary.manifest_ref,
    });
  }
  return chips;
}

export function buildMultiManifestDiff(options: {
  cohort: ExperimentCohortIndex | null;
  primaryManifestRef: string | null;
  secondaryManifestRef: string | null;
  loadedManifest?: ExperimentManifest | null;
}): MultiManifestDiffRow[] {
  const { cohort, primaryManifestRef, secondaryManifestRef, loadedManifest } = options;

  if (!primaryManifestRef || !secondaryManifestRef) {
    return [];
  }
  if (primaryManifestRef === secondaryManifestRef) {
    return [];
  }
  if (!cohort) {
    return [];
  }

  const primaryRef = refByManifestRef(cohort, primaryManifestRef);
  const secondaryRef = refByManifestRef(cohort, secondaryManifestRef);
  if (!primaryRef || !secondaryRef) {
    return [];
  }

  const primaryLoaded =
    loadedManifest && loadedManifest.experiment_id === primaryRef.experiment_id
      ? loadedManifest
      : null;
  const secondaryLoaded =
    loadedManifest && loadedManifest.experiment_id === secondaryRef.experiment_id
      ? loadedManifest
      : null;

  const primaryRuns = runsLengthLabel(primaryRef, primaryLoaded, primaryLoaded != null);
  const secondaryRuns = runsLengthLabel(
    secondaryRef,
    secondaryLoaded,
    secondaryLoaded != null,
  );

  const primaryFp = specFingerprintLabel(primaryRef, primaryLoaded, primaryLoaded != null);
  const secondaryFp = specFingerprintLabel(
    secondaryRef,
    secondaryLoaded,
    secondaryLoaded != null,
  );

  const primaryCapture =
    primaryLoaded != null ? String(captureCount(primaryLoaded)) : "—";
  const secondaryCapture =
    secondaryLoaded != null ? String(captureCount(secondaryLoaded)) : "—";

  const row = (
    field: string,
    primary: string,
    secondary: string,
    source: MultiManifestDiffSource,
    note?: string,
    explanatory = false,
  ): MultiManifestDiffRow => ({
    field,
    primary,
    secondary,
    source,
    note,
    compare_status: deriveRowCompareStatus(primary, secondary, { explanatory }),
  });

  const rows: MultiManifestDiffRow[] = [
    row(
      "experiment_id",
      primaryRef.experiment_id,
      secondaryRef.experiment_id,
      "cohort_index",
    ),
    row(
      "runs.length",
      primaryRuns.primary,
      secondaryRuns.primary,
      primaryLoaded || secondaryLoaded ? "loaded_manifest" : "cohort_index",
      [primaryRuns.note, secondaryRuns.note].filter(Boolean).join("; ") || undefined,
    ),
    row(
      "experiment_class",
      supplementClass(primaryRef, primaryLoaded),
      supplementClass(secondaryRef, secondaryLoaded),
      "cohort_index",
    ),
    row(
      "spec_fingerprint",
      primaryFp.value,
      secondaryFp.value,
      primaryFp.source === "loaded_manifest" || secondaryFp.source === "loaded_manifest"
        ? "loaded_manifest"
        : "cohort_index",
    ),
    row(
      "capture_count",
      primaryCapture,
      secondaryCapture,
      primaryLoaded || secondaryLoaded ? "loaded_manifest" : "cohort_index",
      !primaryLoaded && !secondaryLoaded
        ? "load manifest in workbench to enrich"
        : undefined,
    ),
    row(
      "tag_overlap",
      tagOverlapLabel(cohort, primaryRef, secondaryRef),
      "—",
      "overlap",
      "cohort tags vs ref experiment_class (explanatory)",
      true,
    ),
  ];

  return rows;
}
