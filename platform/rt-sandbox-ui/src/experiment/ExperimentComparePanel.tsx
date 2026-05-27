import { BANNER_EXPERIMENT } from "@/governance/banners";
import { compareBadges, type CompareSide } from "./experimentCompare";
import { TacticalAbCompareTable } from "./TacticalAbCompareTable";
import { TelemetryCompareStrip } from "./TelemetryCompareStrip";
import { TerrainCompareNote } from "./TerrainCompareNote";
import { TacticalAnnexCompareStrip } from "./TacticalAnnexCompareStrip";

export function ExperimentComparePanel({
  sideA,
  sideB,
  experimentId,
}: {
  sideA: CompareSide | null;
  sideB: CompareSide | null;
  experimentId: string;
}) {
  if (!sideA || !sideB) {
    return (
      <p className="text-xs text-slate-500">
        Select compare sources A and B (live sessions or pinned runs).
      </p>
    );
  }

  const badges = compareBadges(sideA, sideB);

  return (
    <section className="space-y-3" data-testid="experiment-compare-panel">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT}</p>
      <p className="text-[10px] text-slate-500">
        experiment <span className="font-mono text-slate-400">{experimentId}</span> — mirrors
        only; not operational comparison
      </p>
      {badges.length > 0 && (
        <div className="flex flex-wrap gap-1">
          {badges.map((b) => (
            <span
              key={b.id}
              className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-sky-200/90"
              title={b.detail}
            >
              {b.label}
            </span>
          ))}
        </div>
      )}
      <TacticalAbCompareTable sideA={sideA} sideB={sideB} />
      <TelemetryCompareStrip sideA={sideA} sideB={sideB} />
      <TerrainCompareNote sideA={sideA} sideB={sideB} />
      {(sideA.annexSummary || sideB.annexSummary) && (
        <dl className="rounded border border-slate-800 bg-slate-950/50 p-2 text-[10px] text-slate-400">
          <dt className="font-medium text-slate-500">capture annex summary (counts only)</dt>
          {sideA.annexSummary && (
            <dd>
              A: mode={sideA.annexSummary.final_tactical_mode ?? "—"} switches=
              {sideA.annexSummary.timeline_counts?.mode_switches ?? 0}
            </dd>
          )}
          {sideB.annexSummary && (
            <dd>
              B: mode={sideB.annexSummary.final_tactical_mode ?? "—"} switches=
              {sideB.annexSummary.timeline_counts?.mode_switches ?? 0}
            </dd>
          )}
        </dl>
      )}
      {(sideA.runId || sideB.runId) && (
        <TacticalAnnexCompareStrip
          runIdA={sideA.runId}
          runIdB={sideB.runId}
          labelA={sideA.label}
          labelB={sideB.label}
        />
      )}
    </section>
  );
}
