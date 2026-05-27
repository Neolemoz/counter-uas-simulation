import {
  cognitionSummary,
  formatAuthorityChip,
  sessionContextLine,
} from "@/telemetry/cognition";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";

const TONE_CLASS = {
  ok: "bg-emerald-900/60 text-emerald-200 border-emerald-700",
  warn: "bg-amber-900/60 text-amber-200 border-amber-700",
  error: "bg-red-900/60 text-red-200 border-red-700",
  neutral: "bg-slate-800 text-slate-300 border-slate-600",
} as const;

export function TelemetryCognitionStrip({
  snapshot,
  sessionId,
  sessionRole = "active",
}: {
  snapshot: ChannelSnapshot | undefined;
  sessionId?: string | null;
  sessionRole?: "active" | "background";
}) {
  if (!snapshot) {
    return (
      <p className="mt-2 text-xs text-slate-500">No telemetry snapshot yet.</p>
    );
  }

  const cognition = cognitionSummary(snapshot.payload);

  const sessionLine = sessionContextLine(sessionId, sessionRole);

  return (
    <div className="mt-3 space-y-2 rounded border border-slate-700/80 bg-slate-950/50 p-2.5 text-xs">
      {sessionLine && (
        <p className="font-mono text-[10px] text-slate-500">{sessionLine}</p>
      )}
      <div className="flex flex-wrap gap-2">
        <span className="rounded border border-slate-600 bg-slate-800 px-2.5 py-1 text-slate-200">
          source: {cognition.source}
        </span>
        <span className="rounded border border-slate-600 bg-slate-800 px-2.5 py-1 font-medium text-slate-200">
          {formatAuthorityChip(cognition.authorityLabel)}
        </span>
        {cognition.stale && (
          <span className="rounded border border-amber-600 bg-amber-950 px-2.5 py-1 font-medium text-amber-100 ring-1 ring-amber-500/50">
            stale
          </span>
        )}
      </div>
      <p className="text-slate-400">{cognition.authorityDescription}</p>
      <p className="text-slate-500">{cognition.sourceDescription}</p>
      <p className="italic text-slate-500">
        Explanatory telemetry — transient runtime truth; not replay authority.
      </p>
      {cognition.badges.length > 0 && (
        <div className="flex flex-wrap gap-1">
          {cognition.badges.map((badge) => (
            <span
              key={badge.label}
              className={`rounded border px-2.5 py-1 text-[11px] font-medium ${TONE_CLASS[badge.tone]}`}
            >
              {badge.label}
            </span>
          ))}
        </div>
      )}
      {cognition.governanceBanner && (
        <p className="text-slate-500">{cognition.governanceBanner}</p>
      )}
      <p className="text-slate-600">updated {snapshot.timestamp_utc}</p>
    </div>
  );
}
