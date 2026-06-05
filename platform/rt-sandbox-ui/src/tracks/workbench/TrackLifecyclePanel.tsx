import { formatTimestamp, labelFromToken } from "./formatters";
import type { TrackLifecycleEvent } from "./trackSensorWorkbenchTypes";

const DEFAULT_EVENTS = ["first_seen", "confirmed", "updated", "coasted", "dropped"] as const;

function eventRows(events: readonly TrackLifecycleEvent[]): TrackLifecycleEvent[] {
  const byKind = new Map(events.map((event) => [event.event, event]));
  const defaults = DEFAULT_EVENTS.map(
    (event): TrackLifecycleEvent =>
      byKind.get(event) ?? {
        event,
        timestamp_utc: null,
        reason: "No lifecycle event recorded.",
        source: "not_available",
      },
  );
  const extras = events.filter((event) => !DEFAULT_EVENTS.includes(event.event as never));
  return [...defaults, ...extras];
}

export function TrackLifecyclePanel({
  events,
}: {
  events: readonly TrackLifecycleEvent[];
}) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="track-lifecycle-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Track lifecycle
      </h3>
      <ol className="space-y-2">
        {eventRows(events).map((event) => (
          <li
            key={`${event.event}-${event.timestamp_utc ?? "missing"}`}
            className="rounded border border-slate-800 bg-slate-950/50 px-2 py-1.5"
          >
            <div className="flex flex-wrap items-center justify-between gap-2">
              <span className="font-semibold text-slate-200">
                {labelFromToken(event.event)}
              </span>
              <span className="font-mono text-[10px] text-slate-500">
                {formatTimestamp(event.timestamp_utc)}
              </span>
            </div>
            <p className="mt-1 text-slate-300">{event.reason}</p>
            <p className="mt-1 text-[10px] text-slate-500">Source: {event.source}</p>
          </li>
        ))}
      </ol>
    </section>
  );
}
