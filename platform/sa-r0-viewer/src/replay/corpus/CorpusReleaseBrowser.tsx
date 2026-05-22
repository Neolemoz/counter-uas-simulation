import { useEffect, useState } from "react";
import type { CorpusReleaseManifest } from "../synthesis/synthesisSchema";
import { DEFAULT_RELEASE_ID, loadCorpusReleaseManifest } from "./loadCorpusAudits";
import { loadCorpusEvolutionManifest } from "./loadCorpusEvolution";
import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";

type Props = {
  hooks: NavigateHooks;
  selectedEntryId: string | null;
};

type ReleaseRow = {
  release_id: string;
  parent_release_ids: string[];
  indexed_entry_count?: number;
  manifest: CorpusReleaseManifest | null;
};

export function CorpusReleaseBrowser({ hooks, selectedEntryId }: Props) {
  const [releases, setReleases] = useState<ReleaseRow[]>([]);
  const [open, setOpen] = useState(false);
  const [expandedId, setExpandedId] = useState<string | null>(DEFAULT_RELEASE_ID);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    loadCorpusEvolutionManifest()
      .then(async (evo) => {
        const frozen = evo.releases.filter(
          (r) => r.release_id && r.release_id !== "canonical_index",
        );
        const rows: ReleaseRow[] = [];
        for (const r of frozen) {
          const rid = String(r.release_id);
          let manifest: CorpusReleaseManifest | null = null;
          try {
            manifest = await loadCorpusReleaseManifest(rid);
          } catch {
            manifest = null;
          }
          rows.push({
            release_id: rid,
            parent_release_ids: (r.parent_release_ids as string[]) ?? [],
            indexed_entry_count: r.indexed_entry_count as number | undefined,
            manifest,
          });
        }
        if (rows.length === 0) {
          const manifest = await loadCorpusReleaseManifest(DEFAULT_RELEASE_ID);
          rows.push({
            release_id: manifest.release_id,
            parent_release_ids: manifest.parent_release_ids ?? [],
            indexed_entry_count: manifest.indexed_entry_ids.length,
            manifest,
          });
        }
        setReleases(rows);
        setErr(null);
      })
      .catch((e: unknown) => setErr(String(e)));
  }, []);

  if (err) return null;
  if (releases.length === 0) return null;

  return (
    <section className="mt-2 rounded border border-slate-800 bg-slate-950/40 p-2">
      <button
        type="button"
        className="flex w-full items-center justify-between text-xs font-medium text-slate-300"
        onClick={() => setOpen((v) => !v)}
      >
        <span>Release snapshots ({releases.length})</span>
        <span className="text-slate-500">{open ? "−" : "+"}</span>
      </button>
      <p className="mt-1 text-[10px] text-slate-500">
        Frozen release manifests from evolution tracking — offline reproducibility only.
      </p>
      {open && (
        <ul className="mt-2 space-y-2">
          {releases.map((row) => (
            <li key={row.release_id} className="rounded border border-slate-800 p-2">
              <button
                type="button"
                className="flex w-full items-center justify-between text-[10px] font-medium text-slate-300"
                onClick={() =>
                  setExpandedId(expandedId === row.release_id ? null : row.release_id)
                }
              >
                <span>{row.release_id}</span>
                <span className="text-slate-500">
                  {row.indexed_entry_count ?? row.manifest?.indexed_entry_ids.length ?? "?"} entries
                </span>
              </button>
              {row.parent_release_ids.length > 0 && (
                <p className="mt-1 text-[10px] text-violet-300/80">
                  Parents: {row.parent_release_ids.join(" → ")}
                </p>
              )}
              {expandedId === row.release_id && row.manifest && (
                <ul className="mt-2 max-h-32 overflow-y-auto text-[10px] text-slate-400">
                  {row.manifest.indexed_entry_ids.map((id) => (
                    <li key={id}>
                      <button
                        type="button"
                        className={
                          id === selectedEntryId
                            ? "text-cyan-200"
                            : "text-slate-400 hover:text-cyan-100"
                        }
                        onClick={() => void navigateToCorpusEntryById(id, hooks)}
                      >
                        {id}
                      </button>
                    </li>
                  ))}
                </ul>
              )}
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
