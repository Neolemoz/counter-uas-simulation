import type { CompareSlotId } from "../compareStore";
import { useCompareStore } from "../compareStore";
import { findAlignedEventId } from "../annotationAlign";

type Props = { slot: CompareSlotId; label: string };

export function CompareSlotTimeline({ slot, label }: Props) {
  const slotState = useCompareStore((s) => (slot === "A" ? s.slotA : s.slotB));
  const syncClock = useCompareStore((s) => s.syncClock);
  const otherBundle = useCompareStore((s) =>
    slot === "A" ? s.slotB.bundle : s.slotA.bundle,
  );
  const setSlotCurrentT = useCompareStore((s) => s.setSlotCurrentT);
  const setSlotPlaying = useCompareStore((s) => s.setSlotPlaying);
  const setSlotSelectedEventId = useCompareStore((s) => s.setSlotSelectedEventId);
  const otherSlot = slot === "A" ? "B" : "A";
  const setOtherSelected = useCompareStore((s) => s.setSlotSelectedEventId);

  const bundle = slotState.bundle;
  if (!bundle) return null;
  const { start, end } = bundle.clock.duration;

  const selectMarker = (t: number, eventId?: string) => {
    setSlotPlaying(slot, false);
    setSlotCurrentT(slot, t);
    if (eventId) {
      setSlotSelectedEventId(slot, eventId);
      if (syncClock && otherBundle && bundle) {
        const aligned = findAlignedEventId(bundle, otherBundle, eventId);
        if (aligned) setOtherSelected(otherSlot, aligned);
      }
    }
  };

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-2">
      <p className="mb-1 text-xs font-semibold uppercase text-slate-500">{label}</p>
      <div className="mb-1 flex items-center gap-2">
        <button
          type="button"
          className="rounded bg-slate-700 px-2 py-0.5 text-xs hover:bg-slate-600"
          onClick={() => setSlotPlaying(slot, !slotState.playing)}
        >
          {slotState.playing ? "Pause" : "Play"}
        </button>
        <span className="text-xs text-slate-400">t = {slotState.currentT}</span>
      </div>
      <input
        type="range"
        min={start}
        max={Math.max(end, start + 1)}
        step={bundle.clock.duration.step}
        value={slotState.currentT}
        className="w-full"
        onChange={(e) => {
          setSlotPlaying(slot, false);
          setSlotCurrentT(slot, Number(e.target.value));
        }}
      />
      <div className="mt-1 flex max-h-16 flex-wrap gap-0.5 overflow-y-auto">
        {bundle.clock.markers.map((m) => (
          <button
            key={`${m.t}-${m.event_id ?? m.label}`}
            type="button"
            className={`rounded px-1 py-0.5 text-[10px] ${
              m.t === slotState.currentT ? "bg-amber-700 text-white" : "bg-slate-800 text-slate-500"
            }`}
            onClick={() => selectMarker(m.t, m.event_id ? String(m.event_id) : undefined)}
          >
            {m.category?.slice(0, 3)}@{m.t}
          </button>
        ))}
      </div>
    </section>
  );
}
