import { useEffect, useState } from "react";
import { loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import type { ReplayCorpusIndex } from "../synthesis/synthesisSchema";
import { loadCorpusEvolutionManifest, loadCorpusEvolutionSummary } from "./loadCorpusEvolution";
import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";
import { readCorpusChronologyFromUrl, useCorpusStore } from "./useCorpusStore";

type Props = {
  hooks: NavigateHooks;
};

export function CorpusEvolutionPanel({ hooks }: Props) {
  const [open, setOpen] = useState(false);
  const [manifest, setManifest] = useState<Awaited<ReturnType<typeof loadCorpusEvolutionManifest>> | null>(null);
  const [summary, setSummary] = useState<Awaited<ReturnType<typeof loadCorpusEvolutionSummary>> | null>(null);
  const [index, setIndex] = useState<ReplayCorpusIndex | null>(null);
  const highlightTier = useCorpusStore((s) => s.highlightChronologyTier);

  useEffect(() => {
    const tier = readCorpusChronologyFromUrl();
    if (tier) useCorpusStore.getState().setHighlightChronologyTier(tier);
  }, []);

  useEffect(() => {
    Promise.all([loadCorpusEvolutionManifest(), loadCorpusEvolutionSummary(), loadReplayCorpusIndex()])
      .then(([m, s, idx]) => {
        setManifest(m);
        setSummary(s);
        setIndex(idx);
      })
      .catch(() => {
        setManifest(null);
        setSummary(null);
      });
  }, []);

  if (!manifest || !summary) return null;

  return (
    <section className="mb-3 rounded border border-violet-900/40 bg-violet-950/15 p-3 text-sm">
      <button
        type="button"
        className="mb-2 flex w-full items-center justify-between font-semibold text-violet-100"
        onClick={() => setOpen((v) => !v)}
      >
        <span>Corpus evolution (long-horizon)</span>
        <span className="text-xs text-slate-400">{open ? "−" : "+"}</span>
      </button>
      <p className="mb-2 text-[10px] text-slate-400">
        {manifest.governance?.notice ??
          "Chronology and release comparison — explanatory replay research only."}
      </p>
      {open && (
        <>
          <div className="mb-3">
            <p className="mb-1 text-[10px] uppercase text-slate-500">Chronology timeline</p>
            <ul className="space-y-2">
              {manifest.chronology_tiers.map((tier) => (
                <li
                  key={tier.tier_id}
                  className={
                    highlightTier === tier.tier_id
                      ? "rounded border border-violet-700/50 bg-violet-950/30 p-2"
                      : "rounded border border-slate-800 p-2"
                  }
                >
                  <p className="text-xs font-medium text-violet-200">{tier.descriptor}</p>
                  <p className="text-[10px] text-slate-500">
                    {tier.entry_count} entries · {tier.release_generation_id}
                  </p>
                  <ul className="mt-1 max-h-16 overflow-y-auto text-[10px] text-slate-400">
                    {tier.entry_ids.slice(0, 8).map((eid) => (
                      <li key={eid}>
                        <button
                          type="button"
                          className="text-cyan-300 hover:underline"
                          onClick={() => void navigateToCorpusEntryById(eid, hooks)}
                        >
                          {eid.split("__").pop()}
                        </button>
                      </li>
                    ))}
                    {tier.entry_ids.length > 8 ? (
                      <li className="text-slate-600">+{tier.entry_ids.length - 8} more</li>
                    ) : null}
                  </ul>
                </li>
              ))}
            </ul>
          </div>
          {summary.divergence_chronology && summary.divergence_chronology.length > 0 && (
            <div className="mb-3">
              <p className="mb-1 text-[10px] uppercase text-slate-500">Divergence chronology</p>
              <ul className="list-inside list-disc space-y-1 text-[10px] text-slate-400">
                {summary.divergence_chronology.slice(0, 8).map((line, i) => (
                  <li key={i}>{line}</li>
                ))}
              </ul>
            </div>
          )}
          {summary.long_horizon_family_narratives &&
            summary.long_horizon_family_narratives.length > 0 && (
              <div className="mb-2">
                <p className="mb-1 text-[10px] uppercase text-slate-500">Family narratives</p>
                {summary.long_horizon_family_narratives.slice(0, 4).map((n) => (
                  <p key={n.replay_family} className="mb-1 text-[10px] text-slate-400">
                    <span className="text-violet-200">{n.replay_family}</span>: {n.summary}
                  </p>
                ))}
              </div>
            )}
          {index && (
            <p className="text-[10px] text-slate-600">
              Releases tracked: {manifest.releases.filter((r) => r.release_id !== "canonical_index").length}{" "}
              frozen + canonical index.
            </p>
          )}
        </>
      )}
    </section>
  );
}
