import type { ExperimentReviewPacket } from "./reviewPacketSchema";

export function ReviewPacketGroupedSummary({ packet }: { packet: ExperimentReviewPacket }) {
  const scope = packet.scope;
  const artifactCount = packet.artifact_refs?.length ?? 0;
  const stepsCount = packet.review_steps_completed?.length ?? 0;

  return (
    <div
      className="rounded border border-slate-800 bg-slate-950/50 p-2"
      data-testid="review-packet-grouped-summary"
    >
      <p className="text-[10px] text-slate-500">
        Grouped summary — export JSON below is unchanged.
      </p>
      <dl className="mt-2 grid gap-1 text-[10px] text-slate-400">
        <div className="flex gap-2">
          <dt className="text-slate-500">Packet</dt>
          <dd className="font-mono text-slate-300">{packet.packet_id}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Cohort</dt>
          <dd>{scope.cohort_id ?? "—"}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Primary</dt>
          <dd className="truncate font-mono">{scope.primary_manifest_ref ?? "—"}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Secondary</dt>
          <dd className="truncate font-mono">{scope.secondary_manifest_ref ?? "—"}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Compare mode</dt>
          <dd className="font-mono">{packet.compare_mode}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Artifacts</dt>
          <dd>{artifactCount}</dd>
        </div>
        <div className="flex gap-2">
          <dt className="text-slate-500">Steps completed</dt>
          <dd>{stepsCount}</dd>
        </div>
      </dl>
    </div>
  );
}
