import { useEffect, useState } from "react";
import { useSweepStore } from "../useSweepStore";
import { loadCrossSweepSynthesis } from "./loadSynthesis";
import type { CrossSweepSynthesis } from "./synthesisSchema";

export function SynthesisSummaryPanel() {
  const sweepId = useSweepStore((s) => s.sweep?.sweep_id);
  const [synthesis, setSynthesis] = useState<CrossSweepSynthesis | null>(null);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    loadCrossSweepSynthesis()
      .then(setSynthesis)
      .catch(() => setSynthesis(null));
  }, []);

  if (!synthesis) return null;
  const bullets = synthesis.cognition_rollup?.bullets ?? [];

  return (
    <section className="rounded border border-teal-900/40 bg-teal-950/20 p-3 text-sm">
      <button
        type="button"
        className="mb-1 flex w-full items-center justify-between font-semibold text-teal-100"
        onClick={() => setOpen((v) => !v)}
      >
        <span>Cross-sweep synthesis</span>
        <span className="text-xs text-slate-400">{open ? "−" : "+"}</span>
      </button>
      <p className="mb-2 text-xs text-slate-400">
        {synthesis.governance?.notice ??
          "Deterministic corpus rollup — explanatory only, not operational planning."}
      </p>
      {open && (
        <>
          <p className="mb-2 text-xs text-slate-300">
            Corpus sweeps: {synthesis.sweep_ids.join(", ")}
            {sweepId ? ` · viewing \`${sweepId}\`` : ""}
          </p>
          <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
            {bullets.map((b) => (
              <li key={b.bullet_id}>
                {b.observation} — replay-local; {b.caveat}
              </li>
            ))}
          </ul>
          {(synthesis.interpretation_caveats ?? []).length > 0 && (
            <ul className="mt-2 list-inside list-disc space-y-1 text-[10px] text-slate-500">
              {synthesis.interpretation_caveats!.map((c, i) => (
                <li key={i}>{c}</li>
              ))}
            </ul>
          )}
        </>
      )}
    </section>
  );
}
