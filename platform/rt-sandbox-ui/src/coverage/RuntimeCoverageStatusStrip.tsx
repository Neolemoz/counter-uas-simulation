import {
  deriveRuntimeCoverageStatus,
  type DeriveRuntimeCoverageStatusParams,
  type ProtectedCenterCoverageState,
  type RuntimeCoverageStatusModel,
} from "./runtimeCoverageSelectors";

function formatCoverageState(state: ProtectedCenterCoverageState): string {
  if (state === "yes") return "Yes";
  if (state === "no") return "No";
  return "Unavailable";
}

function formatMetric(value: number | null, suffix = ""): string {
  if (value == null) return "—";
  return `${value}${suffix}`;
}

function chipTone(
  kind: "neutral" | "good" | "warn" | "muted",
): string {
  switch (kind) {
    case "good":
      return "border-emerald-700/50 bg-emerald-950/35 text-emerald-100";
    case "warn":
      return "border-amber-700/50 bg-amber-950/35 text-amber-100";
    case "muted":
      return "border-slate-700 bg-slate-900/70 text-slate-400";
    default:
      return "border-cyan-800/45 bg-cyan-950/25 text-cyan-100";
  }
}

function StatusChip({
  label,
  value,
  tone = "neutral",
  testId,
}: {
  label: string;
  value: string;
  tone?: "neutral" | "good" | "warn" | "muted";
  testId: string;
}) {
  return (
    <span
      className={`rounded border px-2 py-1 text-[10px] font-semibold uppercase tracking-wide ${chipTone(tone)}`}
      data-testid={testId}
    >
      {label}: {value}
    </span>
  );
}

export function RuntimeCoverageStatusStripView({
  model,
}: {
  model: RuntimeCoverageStatusModel;
}) {
  const coveredTone =
    model.protectedCenterCovered === "yes"
      ? "good"
      : model.protectedCenterCovered === "no"
        ? "warn"
        : "muted";

  return (
    <div
      className="rounded border border-cyan-800/45 bg-cyan-950/20 px-3 py-2 text-xs"
      data-testid="runtime-coverage-status-strip"
    >
      <p className="text-[10px] text-cyan-100/85" data-testid="runtime-coverage-banner">
        {model.banner}
      </p>
      {model.guidance ? (
        <p className="mt-1 text-[11px] text-slate-400" data-testid="runtime-coverage-guidance">
          {model.guidance}
        </p>
      ) : null}
      <div className="mt-2 flex flex-wrap gap-1.5" data-testid="runtime-coverage-chips">
        <StatusChip
          label="Center covered"
          value={formatCoverageState(model.protectedCenterCovered)}
          tone={coveredTone}
          testId="runtime-coverage-center-covered"
        />
        <StatusChip
          label="Covering radars"
          value={formatMetric(model.coveringRadarCount)}
          tone={model.coveringRadarCount === 0 ? "warn" : "neutral"}
          testId="runtime-coverage-covering-count"
        />
        <StatusChip
          label="Coverage"
          value={formatMetric(model.coveragePercent, "%")}
          tone="neutral"
          testId="runtime-coverage-percent"
        />
        <StatusChip
          label="Overlap"
          value={formatMetric(model.overlapPercent, "%")}
          tone="neutral"
          testId="runtime-coverage-overlap"
        />
        <StatusChip
          label="Nearest edge"
          value={
            model.nearestEdgeDistanceM == null
              ? "—"
              : `${model.nearestEdgeDistanceM} m`
          }
          tone={model.nearestEdgeDistanceM === 0 ? "good" : "neutral"}
          testId="runtime-coverage-nearest-edge"
        />
        <StatusChip
          label="Top blind spot"
          value={model.topBlindSpotSector ?? "—"}
          tone={model.topBlindSpotSector ? "warn" : "muted"}
          testId="runtime-coverage-blind-spot"
        />
        <StatusChip
          label="Corridor uncovered"
          value={formatMetric(model.corridorUncoveredPercent, "%")}
          tone={model.corridorUncoveredPercent == null ? "muted" : "neutral"}
          testId="runtime-coverage-corridor"
        />
      </div>
    </div>
  );
}

export function RuntimeCoverageStatusStrip(
  params: DeriveRuntimeCoverageStatusParams,
) {
  const model = deriveRuntimeCoverageStatus(params);
  return <RuntimeCoverageStatusStripView model={model} />;
}
