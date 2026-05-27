import type { MirrorEntity } from "@/cesium/entityMarkers";
import { visibilityHint } from "@/cesium/terrainCognition";
import type { FidelityContext } from "@/fidelity/fidelityCognition";
import {
  domeTruthSummary,
  formatFidelityLabel,
  losDivergenceBadge,
  losTruthSummary,
  partialTruthFlags,
  poseTruthDriftRows,
  poseTruthDriftSummary,
  selectedEntityPoseTruthReadout,
  truthFreshnessSummary,
  truthStaleBadge,
} from "@/fidelity/fidelityCognition";
import { BANNER_FIDELITY_TRUTH } from "@/governance/banners";
import { StatusBadge } from "@/workstation/StatusBadge";

export function FidelityTruthCognitionStrip({
  fidelityContext,
  worldSummary,
  selectedEntity,
  entities = [],
  showLosDivergence = false,
  compact = false,
}: {
  fidelityContext: FidelityContext;
  worldSummary?: Record<string, unknown>;
  selectedEntity?: MirrorEntity | null;
  entities?: MirrorEntity[];
  showLosDivergence?: boolean;
  compact?: boolean;
}) {
  if (!fidelityContext.enableFidelityCoupling) {
    return null;
  }

  const staleBadge = truthStaleBadge(fidelityContext);
  const driftRows = poseTruthDriftRows(fidelityContext, worldSummary);
  const driftSummary = poseTruthDriftSummary(driftRows);
  const partialFlags = partialTruthFlags(fidelityContext, worldSummary);
  const losLine = losTruthSummary(fidelityContext);
  const domeLine = domeTruthSummary(fidelityContext);
  const heuristicLos =
    showLosDivergence && selectedEntity
      ? visibilityHint(selectedEntity, entities)
      : null;
  const divergence = losDivergenceBadge(fidelityContext, heuristicLos);
  const poseReadout = selectedEntityPoseTruthReadout(
    fidelityContext,
    selectedEntity?.entity_id,
    worldSummary,
  );

  return (
    <section
      className={`rounded border border-violet-900/50 bg-violet-950/20 ${
        compact ? "mt-2 space-y-2 p-2" : "mt-3 space-y-3 p-3"
      }`}
      data-testid="fidelity-truth-cognition-strip"
    >
      <p className="text-[10px] text-amber-100/80">{BANNER_FIDELITY_TRUTH}</p>
      <div className="flex flex-wrap items-center gap-2">
        <StatusBadge
          label={formatFidelityLabel(
            fidelityContext.fidelityLabel ?? "truth_attested",
          )}
          tone="ok"
        />
        <StatusBadge label="explanatory" tone="neutral" />
        {staleBadge && (
          <StatusBadge label={staleBadge.label} tone={staleBadge.tone} />
        )}
        {divergence && (
          <StatusBadge label={divergence.label} tone={divergence.tone} />
        )}
        {partialFlags.map((flag) => (
          <StatusBadge key={flag} label={flag} tone="warn" />
        ))}
        {driftSummary.count > 0 && (
          <StatusBadge
            label={`pose_truth_drift ×${driftSummary.count}`}
            tone="warn"
          />
        )}
      </div>
      <p className="text-[10px] text-slate-400">{truthFreshnessSummary(fidelityContext)}</p>
      {!compact && losLine && (
        <p className="text-[10px] text-slate-400">
          <span className="font-medium text-violet-300/90">truth_attested · </span>
          {losLine}
        </p>
      )}
      {!compact && domeLine && (
        <p className="text-[10px] text-slate-400">
          <span className="font-medium text-violet-300/90">truth_attested · </span>
          {domeLine}
        </p>
      )}
      {!compact && driftSummary.maxDriftM != null && (
        <p className="text-[10px] text-slate-500">
          Pose drift summary — {driftSummary.count} entities · max{" "}
          {driftSummary.maxDriftM.toFixed(3)} m (command vs truth)
        </p>
      )}
      {poseReadout && (
        <p className="text-[10px] text-slate-300">
          <span className="font-medium text-slate-500">selected · </span>
          {poseReadout}
        </p>
      )}
    </section>
  );
}
