import { useCompareStore } from "../compareStore";

export function CompareModeControls() {
  const syncClock = useCompareStore((s) => s.syncClock);
  const cameraLocked = useCompareStore((s) => s.cameraLocked);
  const emphasizeDelta = useCompareStore((s) => s.emphasizeDelta);
  const focusSlot = useCompareStore((s) => s.focusSlot);
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);
  const setSyncClock = useCompareStore((s) => s.setSyncClock);
  const setCameraLocked = useCompareStore((s) => s.setCameraLocked);
  const setEmphasizeDelta = useCompareStore((s) => s.setEmphasizeDelta);
  const setFocusSlot = useCompareStore((s) => s.setFocusSlot);
  const setSlotPlaying = useCompareStore((s) => s.setSlotPlaying);
  const exitCompare = useCompareStore((s) => s.exitCompare);
  const activePairId = useCompareStore((s) => s.activePairId);

  const playing = slotA.playing || slotB.playing;

  return (
    <section className="rounded border border-amber-900/40 bg-amber-950/20 p-3 text-sm">
      <div className="mb-2 flex items-center justify-between gap-2">
        <h2 className="font-semibold text-amber-100">Compare controls</h2>
        <button
          type="button"
          className="rounded bg-slate-800 px-2 py-0.5 text-xs text-slate-300 hover:bg-slate-700"
          onClick={() => exitCompare()}
        >
          Exit compare
        </button>
      </div>
      {activePairId && (
        <p className="mb-2 text-xs text-slate-500">Pair: {activePairId}</p>
      )}
      <label className="mb-1 flex items-center gap-2 text-slate-300">
        <input
          type="checkbox"
          checked={syncClock}
          onChange={(e) => setSyncClock(e.target.checked)}
        />
        Shared replay clock
      </label>
      <label className="mb-1 flex items-center gap-2 text-slate-300">
        <input
          type="checkbox"
          checked={cameraLocked}
          onChange={(e) => setCameraLocked(e.target.checked)}
        />
        Lock camera (B follows A)
      </label>
      <label className="mb-2 flex items-center gap-2 text-slate-300">
        <input
          type="checkbox"
          checked={emphasizeDelta}
          onChange={(e) => setEmphasizeDelta(e.target.checked)}
        />
        Emphasize topology delta
      </label>
      <div className="mb-2 flex gap-1">
        <button
          type="button"
          className={`rounded px-2 py-0.5 text-xs ${focusSlot === "A" ? "bg-amber-800 text-amber-100" : "bg-slate-800 text-slate-400"}`}
          onClick={() => setFocusSlot("A")}
        >
          Focus A
        </button>
        <button
          type="button"
          className={`rounded px-2 py-0.5 text-xs ${focusSlot === "B" ? "bg-amber-800 text-amber-100" : "bg-slate-800 text-slate-400"}`}
          onClick={() => setFocusSlot("B")}
        >
          Focus B
        </button>
      </div>
      <button
        type="button"
        className="w-full rounded bg-slate-800 py-1 text-xs text-slate-200 hover:bg-slate-700"
        onClick={() => {
          const next = !playing;
          setSlotPlaying("A", next);
          setSlotPlaying("B", next);
        }}
      >
        {playing ? "Pause both" : "Play both"}
      </button>
      <p className="mt-2 text-[10px] text-slate-500">
        Explanatory comparison only — not operational benchmarking.
      </p>
    </section>
  );
}
