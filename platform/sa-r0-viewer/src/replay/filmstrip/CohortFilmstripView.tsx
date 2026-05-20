import { useEffect } from "react";
import { useCohortFilmstripStore } from "../cohortFilmstripStore";
import { FilmstripSlotMapPane } from "./FilmstripSlotMapPane";
import { LayerToggles } from "../LayerToggles";
import { ReplayNarrativePanel } from "../analytics/ReplayNarrativePanel";

export function CohortFilmstripView() {
  const slots = useCohortFilmstripStore((s) => s.slots);
  const sweep = useCohortFilmstripStore((s) => s.sweep);
  const syncClock = useCohortFilmstripStore((s) => s.syncClock);
  const playbackMs = useCohortFilmstripStore((s) => s.playbackMs);
  const tickFilmstripPlayback = useCohortFilmstripStore((s) => s.tickFilmstripPlayback);
  const setSyncClock = useCohortFilmstripStore((s) => s.setSyncClock);
  const setSlotPlaying = useCohortFilmstripStore((s) => s.setSlotPlaying);
  const setSlotCurrentT = useCohortFilmstripStore((s) => s.setSlotCurrentT);
  const exitFilmstrip = useCohortFilmstripStore((s) => s.exitFilmstrip);

  const playing = slots.some((s) => s.playing);

  useEffect(() => {
    if (!playing) return;
    const id = window.setInterval(tickFilmstripPlayback, playbackMs);
    return () => window.clearInterval(id);
  }, [playing, playbackMs, tickFilmstripPlayback]);

  const leader = slots[0];
  const duration = leader?.bundle?.clock.duration;

  return (
    <main className="grid min-h-0 flex-1 grid-cols-1 gap-3 p-3 lg:grid-cols-12">
      <aside className="flex flex-col gap-3 lg:col-span-3">
        <section className="rounded border border-violet-800/50 bg-violet-950/30 p-3 text-sm">
          <h2 className="mb-2 font-semibold text-violet-200">Cohort filmstrip</h2>
          <p className="mb-2 text-xs text-slate-400">
            {sweep?.title} — synchronized replay variants (explanatory only).
          </p>
          <button
            type="button"
            className="rounded bg-slate-800 px-2 py-1 text-xs text-slate-300 hover:bg-slate-700"
            onClick={() => {
              exitFilmstrip();
            }}
          >
            Exit filmstrip
          </button>
        </section>
        <ReplayNarrativePanel />
        <LayerToggles />
        <label className="flex items-center gap-2 text-xs text-slate-400">
          <input
            type="checkbox"
            checked={syncClock}
            onChange={(e) => setSyncClock(e.target.checked)}
          />
          Synchronized playback
        </label>
        {duration && leader?.bundle && (
          <div className="rounded border border-slate-700 bg-slate-900/80 p-2 text-xs">
            <input
              type="range"
              className="w-full"
              min={duration.start ?? 0}
              max={duration.end ?? 0}
              step={duration.step ?? 1}
              value={leader.currentT}
              onChange={(e) => setSlotCurrentT(0, Number(e.target.value))}
            />
            <div className="mt-1 flex justify-between text-slate-500">
              <button type="button" onClick={() => setSlotPlaying(0, !playing)}>
                {playing ? "Pause" : "Play"}
              </button>
              <span>t={leader.currentT}</span>
            </div>
          </div>
        )}
      </aside>
      <div className="flex flex-col gap-2 lg:col-span-9">
        <div
          className="grid min-h-[240px] flex-1 gap-2"
          style={{ gridTemplateColumns: `repeat(${Math.min(slots.length, 4)}, minmax(0, 1fr))` }}
        >
          {slots.map((slot, i) => {
            const member = sweep?.members[slot.memberIndex];
            return (
              <FilmstripSlotMapPane
                key={slot.memberIndex}
                slot={slot}
                label={member?.member_id ?? `Slot ${i + 1}`}
              />
            );
          })}
        </div>
      </div>
    </main>
  );
}
