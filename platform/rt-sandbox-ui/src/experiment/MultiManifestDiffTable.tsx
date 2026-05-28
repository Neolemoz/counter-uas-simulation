import {
  buildMultiManifestDiff,
  buildManifestSummaryChips,
  MULTI_MANIFEST_DIFF_BANNER,
} from "./multiManifestDiff";
import { MultiManifestMetadataDrillDown } from "./MultiManifestMetadataDrillDown";
import type { ExperimentCohortIndex } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";

export function MultiManifestDiffTable({
  cohort,
  primaryManifestRef,
  secondaryManifestRef,
  loadedManifest,
  onOpenManifestRef,
}: {
  cohort: ExperimentCohortIndex | null;
  primaryManifestRef: string | null;
  secondaryManifestRef: string | null;
  loadedManifest?: ExperimentManifest | null;
  onOpenManifestRef?: (manifestRef: string, role: "primary" | "secondary") => void;
}) {
  const rows = buildMultiManifestDiff({
    cohort,
    primaryManifestRef,
    secondaryManifestRef,
    loadedManifest,
  });

  const chips = buildManifestSummaryChips({
    cohort,
    primaryRef: primaryManifestRef,
    secondaryRef: secondaryManifestRef,
  });

  if (!primaryManifestRef || !secondaryManifestRef) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">
          Set primary and secondary manifests in the manifest roster.
        </p>
      </div>
    );
  }

  if (primaryManifestRef === secondaryManifestRef) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">Primary and secondary must be different manifests.</p>
      </div>
    );
  }

  if (rows.length === 0) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">
          Import a cohort index and select refs that exist in the active cohort.
        </p>
      </div>
    );
  }

  return (
    <div data-testid="multi-manifest-diff-table">
      <MultiManifestMetadataDrillDown
        rows={rows}
        chips={chips}
        primaryManifestRef={primaryManifestRef}
        secondaryManifestRef={secondaryManifestRef}
        onOpenManifestRef={onOpenManifestRef}
      />
    </div>
  );
}
