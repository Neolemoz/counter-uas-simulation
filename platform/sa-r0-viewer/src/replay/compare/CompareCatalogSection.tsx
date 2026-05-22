import { useEffect, useState } from "react";
import { loadBundleFromUrl } from "../loadBundle";
import { loadScenarioCatalog } from "../loadCatalog";
import { loadComparePairs, pairById } from "../loadComparePairs";
import type { ComparePairsManifest } from "../catalogSchema";
import { useCompareStore } from "../compareStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { navigateToComparePair } from "@/navigation/experimentNavigation";
import { useWorkspaceSegmentStore } from "@/workspace/workspaceSegmentStore";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  hooks?: ExperimentNavHooks;
};

export function CompareCatalogSection({ onLoadError, onLoading, hooks }: Props) {
  const [pairs, setPairs] = useState<ComparePairsManifest | null>(null);
  const [packIds, setPackIds] = useState<string[]>([]);
  const [slotAPack, setSlotAPack] = useState("");
  const [slotBPack, setSlotBPack] = useState("");
  const [selectedPair, setSelectedPair] = useState("");
  const mode = useCompareStore((s) => s.mode);
  const exitCompare = useCompareStore((s) => s.exitCompare);
  const setSegment = useWorkspaceSegmentStore((s) => s.setUserSegment);

  useEffect(() => {
    void loadComparePairs().then(setPairs).catch((e: unknown) => onLoadError(String(e)));
    void loadScenarioCatalog()
      .then((c) => setPackIds(c.packs.map((p) => p.pack_id)))
      .catch(() => setPackIds([]));
  }, [onLoadError]);

  const loadPair = async (pairId: string) => {
    if (hooks) {
      await navigateToComparePair(pairId, hooks);
      setSelectedPair(pairId);
      return;
    }
    if (!pairs) return;
    const pair = pairById(pairs, pairId);
    if (!pair) return;
    onLoading(true);
    try {
      const enterCompare = useCompareStore.getState().enterCompare;
      const [a, b] = await Promise.all([
        loadBundleFromUrl(pair.slot_a.demo_bundle_url),
        loadBundleFromUrl(pair.slot_b.demo_bundle_url),
      ]);
      enterCompare(a, b, pairId);
      const url = new URL(window.location.href);
      url.searchParams.set("pair", pairId);
      url.searchParams.delete("demo");
      url.searchParams.delete("compare");
      window.history.replaceState({}, "", url.toString());
      setSegment("compare");
      onLoadError("");
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
  };

  const loadCustomCompare = async () => {
    if (!slotAPack || !slotBPack) return;
    onLoading(true);
    try {
      const catalog = await loadScenarioCatalog();
      const packA = catalog.packs.find((p) => p.pack_id === slotAPack);
      const packB = catalog.packs.find((p) => p.pack_id === slotBPack);
      if (!packA || !packB) throw new Error("Invalid pack selection");
      const enterCompare = useCompareStore.getState().enterCompare;
      const [a, b] = await Promise.all([
        loadBundleFromUrl(packA.demo_bundle_url),
        loadBundleFromUrl(packB.demo_bundle_url),
      ]);
      enterCompare(a, b, null);
      const url = new URL(window.location.href);
      url.searchParams.set("compare", `${slotAPack},${slotBPack}`);
      url.searchParams.delete("demo");
      url.searchParams.delete("pair");
      window.history.replaceState({}, "", url.toString());
      setSegment("compare");
      onLoadError("");
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
  };

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Compare mode</h2>
      {mode === "compare" && (
        <button
          type="button"
          className="mb-2 w-full rounded bg-slate-700 py-1 text-xs text-slate-200 hover:bg-slate-600"
          onClick={() => {
            exitCompare();
            const url = new URL(window.location.href);
            url.searchParams.delete("pair");
            url.searchParams.delete("compare");
            window.history.replaceState({}, "", url.toString());
          }}
        >
          Exit compare (single replay)
        </button>
      )}
      <label className="mb-2 block text-slate-400">
        <span className="mb-1 block text-xs uppercase text-slate-500">Curated pair</span>
        <select
          className="w-full rounded border border-slate-600 bg-slate-800 px-2 py-1 text-slate-200"
          value={selectedPair}
          onChange={(e) => {
            setSelectedPair(e.target.value);
            if (e.target.value) void loadPair(e.target.value);
          }}
        >
          <option value="">— select pair —</option>
          {(pairs?.pairs ?? []).map((p) => (
            <option key={p.pair_id} value={p.pair_id}>
              {p.label}
            </option>
          ))}
        </select>
      </label>
      <div className="grid grid-cols-2 gap-2">
        <select
          className="rounded border border-slate-600 bg-slate-800 px-1 py-1 text-xs text-slate-200"
          value={slotAPack}
          onChange={(e) => setSlotAPack(e.target.value)}
        >
          <option value="">Slot A pack</option>
          {packIds.map((id) => (
            <option key={`a-${id}`} value={id}>
              {id}
            </option>
          ))}
        </select>
        <select
          className="rounded border border-slate-600 bg-slate-800 px-1 py-1 text-xs text-slate-200"
          value={slotBPack}
          onChange={(e) => setSlotBPack(e.target.value)}
        >
          <option value="">Slot B pack</option>
          {packIds.map((id) => (
            <option key={`b-${id}`} value={id}>
              {id}
            </option>
          ))}
        </select>
      </div>
      <button
        type="button"
        className="mt-2 w-full rounded bg-amber-900/50 py-1.5 text-xs text-amber-100 hover:bg-amber-900/70 disabled:opacity-40"
        disabled={!slotAPack || !slotBPack}
        onClick={() => void loadCustomCompare()}
      >
        Compare selected packs
      </button>
    </section>
  );
}
