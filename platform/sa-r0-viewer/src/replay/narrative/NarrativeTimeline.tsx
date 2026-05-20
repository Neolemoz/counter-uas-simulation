import { useClockStore } from "../clockStore";

export function NarrativeTimeline() {
  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  const selectedEventId = useClockStore((s) => s.selectedEventId);
  const setCurrentT = useClockStore((s) => s.setCurrentT);
  const setPlaying = useClockStore((s) => s.setPlaying);
  const setSelectedEventId = useClockStore((s) => s.setSelectedEventId);
  const setHighlightedTrackIds = useClockStore((s) => s.setHighlightedTrackIds);

  if (!bundle) return null;

  const selectEvent = (eventId: string, lineIndex: number | null) => {
    setPlaying(false);
    setSelectedEventId(eventId);
    if (lineIndex != null) setCurrentT(lineIndex);
    const sel = bundle.narrative.events.find((e) => e.event_id === eventId);
    if (sel?.category === "selection") {
      setHighlightedTrackIds(
        bundle.tracks.filter((t) => t.role === "interceptor").map((t) => t.track_id),
      );
    }
  };

  return (
    <section className="max-h-64 overflow-y-auto rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Narrative events</h2>
      <ul className="space-y-2">
        {bundle.narrative.events.map((ev) => {
          const lineIndex = ev.line_index as number | null | undefined;
          const active = lineIndex != null && lineIndex <= currentT;
          const selected = ev.event_id === selectedEventId;
          return (
            <li key={String(ev.event_id)}>
              <button
                type="button"
                className={`w-full rounded border px-2 py-1 text-left ${
                  selected
                    ? "border-amber-600 bg-amber-950/40"
                    : active
                      ? "border-slate-600 bg-slate-800"
                      : "border-transparent opacity-60"
                }`}
                onClick={() => selectEvent(String(ev.event_id), lineIndex ?? null)}
              >
                <span className="text-xs uppercase text-slate-500">{String(ev.category)}</span>
                <p className="text-slate-200">{String(ev.label)}</p>
                <p className="text-xs text-slate-500">{String(ev.interpretation_caveat ?? "")}</p>
              </button>
            </li>
          );
        })}
      </ul>
    </section>
  );
}
