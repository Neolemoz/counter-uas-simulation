import type { CompareSlotId } from "../compareStore";
import { useCompareStore } from "../compareStore";
import { findAlignedEventId } from "../annotationAlign";
import { sandboxButtons, sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

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
    <section className={`${sandboxSurfaces.panelInset} p-2`}>
      <p className={`mb-0.5 ${sandboxTypography.sectionLabel}`}>{label}</p>
      {!syncClock && (
        <p className={`mb-2 ${sandboxTypography.caption}`}>Independent replay clock for this slot.</p>
      )}
      <div className="mb-1 flex items-center gap-2">
        <button
          type="button"
          className={sandboxButtons.subtle}
          onClick={() => setSlotPlaying(slot, !slotState.playing)}
        >
          {slotState.playing ? "Pause" : "Play"}
        </button>
        <span className="text-xs tabular-nums text-slate-400">t = {slotState.currentT}</span>
      </div>
      <input
        type="range"
        min={start}
        max={Math.max(end, start + 1)}
        step={bundle.clock.duration.step}
        value={slotState.currentT}
        className="w-full accent-amber-600"
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
              m.t === slotState.currentT
                ? "bg-amber-800/50 text-amber-100"
                : "bg-slate-800/80 text-slate-500 hover:text-slate-300"
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
