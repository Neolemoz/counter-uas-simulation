import type { RtTacticalReplayContinuity } from "../tacticalReplayContinuitySchema";

type Props = {
  continuity: RtTacticalReplayContinuity;
};

export function RtCaptureOriginBadge({ continuity }: Props) {
  return (
    <span className="rounded-md border border-cyan-800/40 bg-cyan-950/30 px-2 py-0.5 text-[11px] font-medium text-cyan-200/90">
      RT capture — replay explanatory
      {continuity.capture_candidate_id ? (
        <span className="ml-1 font-mono text-[10px] text-cyan-300/70">
          {continuity.capture_candidate_id.slice(0, 8)}…
        </span>
      ) : null}
    </span>
  );
}
