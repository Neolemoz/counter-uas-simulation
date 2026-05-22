import { useEffect, useState } from "react";
import { loadBundleFromUrl } from "./loadBundle";
import { loadSweepsIndex, sweepEntryById } from "./loadSweep";
import { loadSweepManifest } from "./loadSweep";
import type { SweepsIndex } from "./sweepSchema";
import { SWEEP_KIND_LABELS } from "./sweepSchema";
import { useClockStore } from "./clockStore";
import { useCompareStore } from "./compareStore";
import { filteredMemberIndices, useSweepStore } from "./useSweepStore";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
};

export function SweepCatalogPicker({ onLoadError, onLoading }: Props) {
  const [index, setIndex] = useState<SweepsIndex | null>(null);
  const [selectedSweepId, setSelectedSweepId] = useState("");
  const sweep = useSweepStore((s) => s.sweep);
  const memberIndex = useSweepStore((s) => s.memberIndex);
  const setSweep = useSweepStore((s) => s.setSweep);
  const setMemberIndex = useSweepStore((s) => s.setMemberIndex);
  const setBundle = useClockStore((s) => s.setBundle);
  const exitCompare = useCompareStore((s) => s.exitCompare);

  useEffect(() => {
    loadSweepsIndex()
      .then((idx) => {
        setIndex(idx);
        const params = new URLSearchParams(window.location.search);
        const sid = params.get("sweep");
        if (sid && sweepEntryById(idx, sid)) setSelectedSweepId(sid);
        else if (idx.sweeps[0]) setSelectedSweepId(idx.sweeps[0].sweep_id);
      })
      .catch((e: unknown) => onLoadError(String(e)));
  }, [onLoadError]);

  const loadSweepMember = async (sweepId: string, memberIdx: number) => {
    onLoading(true);
    try {
      exitCompare();
      const manifest = await loadSweepManifest(sweepId);
      const idx = Math.min(memberIdx, manifest.members.length - 1);
      const member = manifest.members[idx]!;
      setSweep(manifest);
      setMemberIndex(idx);
      const b = await loadBundleFromUrl(member.demo_bundle_url);
      setBundle(b);
      onLoadError("");
      const url = new URL(window.location.href);
      url.searchParams.set("sweep", sweepId);
      url.searchParams.set("member", String(idx));
      url.searchParams.delete("pair");
      url.searchParams.delete("compare");
      url.searchParams.delete("demo");
      window.history.replaceState({}, "", url.toString());
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
  };

  if (!index?.sweeps.length) return null;

  const grouped = new Map<string, typeof index.sweeps>();
  for (const s of index.sweeps) {
    const k = s.sweep_kind;
    if (!grouped.has(k)) grouped.set(k, []);
    grouped.get(k)!.push(s);
  }

  return (
    <section className="rounded border border-violet-900/50 bg-violet-950/20 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-violet-200">Monte Carlo sweep families</h2>
      <p className="mb-2 text-xs text-slate-400">Explanatory replay experiment groups — not operational forecasts.</p>
      {Array.from(grouped.entries()).map(([kind, entries]) => (
        <div key={kind} className="mb-2">
          <p className="text-xs uppercase text-slate-500">{SWEEP_KIND_LABELS[kind] ?? kind}</p>
          <ul className="mt-1 space-y-1">
            {entries.map((e) => (
              <li key={e.sweep_id}>
                <button
                  type="button"
                  className={`w-full rounded px-2 py-1 text-left text-xs ${
                    selectedSweepId === e.sweep_id
                      ? "bg-violet-800/60 text-violet-100"
                      : "text-slate-300 hover:bg-slate-800"
                  }`}
                  onClick={() => {
                    setSelectedSweepId(e.sweep_id);
                    void loadSweepMember(e.sweep_id, 0);
                  }}
                >
                  {e.title}{" "}
                  <span className="text-slate-500">({e.member_count} replays)</span>
                </button>
              </li>
            ))}
          </ul>
        </div>
      ))}
      {sweep && sweep.members.length > 1 && (
        <div className="mt-2 border-t border-violet-900/40 pt-2">
          <label className="text-xs text-slate-400">
            Member{" "}
            <select
              className="ml-1 rounded bg-slate-800 px-1 text-slate-200"
              value={memberIndex}
              onChange={(ev) => {
                const idx = parseInt(ev.target.value, 10);
                void loadSweepMember(sweep.sweep_id, idx);
              }}
            >
              {filteredMemberIndices(sweep).map((i) => {
                const m = sweep.members[i]!;
                const isAnomaly = sweep.replay_cohorts?.some((c) =>
                  (c.anomaly_member_indices ?? []).includes(i),
                );
                return (
                  <option key={m.member_id} value={i}>
                    {m.member_id} ({m.pack_id}){isAnomaly ? " *" : ""}
                  </option>
                );
              })}
            </select>
          </label>
        </div>
      )}
    </section>
  );
}
