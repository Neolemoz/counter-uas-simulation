import type { CorpusIndexEntry } from "../synthesis/synthesisSchema";

type Props = {
  entry: CorpusIndexEntry | null | undefined;
};

export function CorpusEvolutionBadges({ entry }: Props) {
  if (!entry) return null;
  const tags = entry.evolution_tags ?? [];
  const gen = entry.release_generation_id;
  if (!gen && tags.length === 0) return null;

  return (
    <p className="mb-2 flex flex-wrap gap-1 text-[10px]">
      {gen && (
        <span className="rounded border border-violet-900/50 bg-violet-950/40 px-1.5 py-0.5 text-violet-200">
          gen: {gen}
        </span>
      )}
      {tags.map((t) => (
        <span
          key={t}
          className="rounded border border-slate-700 bg-slate-900/60 px-1.5 py-0.5 text-slate-400"
        >
          {t}
        </span>
      ))}
    </p>
  );
}
