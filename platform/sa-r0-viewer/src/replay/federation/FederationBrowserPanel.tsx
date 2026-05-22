import { useEffect, useState } from "react";
import { loadFederationIndex, loadFederationManifest } from "./loadFederationArtifacts";
import type { ReplayFederationIndex, ReplayFederationManifest } from "./federationSchema";
import { useFederationStore, writeFederationToUrl } from "./useFederationStore";

export function FederationBrowserPanel() {
  const [manifest, setManifest] = useState<ReplayFederationManifest | null>(null);
  const [index, setIndex] = useState<ReplayFederationIndex | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const corpusGroupId = useFederationStore((s) => s.corpusGroupId);
  const setCorpusGroupId = useFederationStore((s) => s.setCorpusGroupId);
  const setFederationId = useFederationStore((s) => s.setFederationId);

  useEffect(() => {
    Promise.all([loadFederationManifest(), loadFederationIndex()])
      .then(([m, idx]) => {
        setManifest(m);
        setIndex(idx);
        setFederationId(m.federation_id);
        setErr(null);
      })
      .catch((e: Error) => setErr(e.message));
  }, [setFederationId]);

  const summaries = index?.corpus_group_summaries ?? [];

  const selectGroup = (gid: string) => {
    setCorpusGroupId(gid);
    writeFederationToUrl(manifest?.federation_id ?? null, gid, null);
  };

  if (err) {
    return <p className="text-xs text-red-300/80">{err}</p>;
  }

  if (!manifest) {
    return <p className="text-xs text-slate-500">Loading federation registry…</p>;
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-cyan-200/90">
        FEDERATION REGISTRY — read-only; not cloud sync
      </p>
      <p className="font-mono text-[10px] text-slate-500">{manifest.federation_id}</p>
      <ul className="space-y-1">
        {manifest.corpus_groups.map((g) => {
          const summary = summaries.find((s) => s.corpus_group_id === g.corpus_group_id);
          const active = corpusGroupId === g.corpus_group_id;
          return (
            <li key={g.corpus_group_id}>
              <button
                type="button"
                className={`w-full rounded border px-2 py-1 text-left ${
                  active
                    ? "border-cyan-600/60 bg-cyan-950/40"
                    : "border-slate-700 bg-slate-900/60 hover:border-slate-600"
                }`}
                onClick={() => selectGroup(g.corpus_group_id)}
              >
                <span className="font-mono text-cyan-300/90">{g.corpus_group_id}</span>
                {g.study_label && (
                  <span className="ml-1 text-slate-400">— {g.study_label}</span>
                )}
                {summary && (
                  <span className="block text-[10px] text-slate-500">
                    {summary.entry_count} entries · {g.corpus_id}
                    {g.release_id ? ` · release ${g.release_id}` : ""}
                  </span>
                )}
              </button>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
