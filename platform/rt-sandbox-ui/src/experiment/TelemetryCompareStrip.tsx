import { entityCount, type CompareSide } from "./experimentCompare";

export function TelemetryCompareStrip({ sideA, sideB }: { sideA: CompareSide; sideB: CompareSide }) {
  const syncA = String(sideA.worldSummary?.sync_health ?? "—");
  const syncB = String(sideB.worldSummary?.sync_health ?? "—");
  const adapterA = String(sideA.worldSummary?.adapter_mode ?? "—");
  const adapterB = String(sideB.worldSummary?.adapter_mode ?? "—");
  return (
    <dl className="grid gap-1 text-xs text-slate-400">
      <div>
        <dt className="inline font-medium text-slate-500">entities: </dt>
        <dd className="inline font-mono">
          {entityCount(sideA.worldSummary)} vs {entityCount(sideB.worldSummary)}
        </dd>
      </div>
      <div>
        <dt className="inline font-medium text-slate-500">sync_health: </dt>
        <dd className="inline font-mono">
          {syncA} vs {syncB}
        </dd>
      </div>
      <div>
        <dt className="inline font-medium text-slate-500">adapter_mode: </dt>
        <dd className="inline font-mono">
          {adapterA} vs {adapterB}
        </dd>
      </div>
    </dl>
  );
}
