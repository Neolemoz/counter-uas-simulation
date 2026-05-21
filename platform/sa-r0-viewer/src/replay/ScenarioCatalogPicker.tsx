import { useEffect, useMemo, useState } from "react";
import { loadBundleFromUrl } from "./loadBundle";
import { loadScenarioCatalog, allCatalogTags } from "./loadCatalog";
import type { CatalogPack, ScenarioCatalog } from "./catalogSchema";
import { CATEGORY_LABELS } from "./catalogSchema";
import { ScenarioFilterBar } from "./ScenarioFilterBar";
import { ScenarioPreviewCard } from "./ScenarioPreviewCard";
import { CompareCatalogSection } from "./compare/CompareCatalogSection";
import { CorpusBrowserPanel } from "./corpus/CorpusBrowserPanel";
import { CorpusEvolutionPanel } from "./corpus/CorpusEvolutionPanel";
import type { NavigateHooks } from "./corpus/navigateToCorpusEntry";
import { SweepCatalogPicker } from "./SweepCatalogPicker";
import { useSweepStore } from "./useSweepStore";
import { useClockStore } from "./clockStore";
import { useCompareStore } from "./compareStore";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  navigateHooks: NavigateHooks;
};

function packMatchesFilters(pack: CatalogPack, activeTags: string[]): boolean {
  if (activeTags.length === 0) return true;
  const pool = new Set([...pack.topology_tags, ...(pack.replay_tags ?? [])]);
  return activeTags.every((t) => pool.has(t));
}

export function ScenarioCatalogPicker({ onLoadError, onLoading, navigateHooks }: Props) {
  const [catalog, setCatalog] = useState<ScenarioCatalog | null>(null);
  const [activeTags, setActiveTags] = useState<string[]>([]);
  const [selectedId, setSelectedId] = useState<string>("");
  const [loadingCatalog, setLoadingCatalog] = useState(true);
  const bundle = useClockStore((s) => s.bundle);
  const setBundle = useClockStore((s) => s.setBundle);

  useEffect(() => {
    loadScenarioCatalog()
      .then((cat) => {
        setCatalog(cat);
        const params = new URLSearchParams(window.location.search);
        const demo = params.get("demo");
        const match =
          cat.packs.find((p) => p.pack_id === demo) ??
          cat.packs.find((p) => bundle?.scenario.catalog_pack_id === p.pack_id) ??
          cat.packs[0];
        if (match) setSelectedId(match.pack_id);
      })
      .catch((e: unknown) => onLoadError(String(e)))
      .finally(() => setLoadingCatalog(false));
  }, [onLoadError, bundle?.scenario.catalog_pack_id]);

  const packs = catalog?.packs ?? [];
  const filtered = useMemo(
    () => packs.filter((p) => packMatchesFilters(p, activeTags)),
    [packs, activeTags],
  );

  const grouped = useMemo(() => {
    const map = new Map<string, CatalogPack[]>();
    for (const p of filtered) {
      const cat = p.category ?? "ingress_geometry";
      if (!map.has(cat)) map.set(cat, []);
      map.get(cat)!.push(p);
    }
    return map;
  }, [filtered]);

  const selected = packs.find((p) => p.pack_id === selectedId);
  const tagPool = useMemo(() => (catalog ? allCatalogTags(catalog) : []), [catalog]);

  const toggleTag = (tag: string) => {
    setActiveTags((prev) => (prev.includes(tag) ? prev.filter((t) => t !== tag) : [...prev, tag]));
  };

  const exitCompare = useCompareStore((s) => s.exitCompare);
  const exitSweep = useSweepStore((s) => s.exitSweep);

  const loadPack = async (pack: CatalogPack) => {
    onLoading(true);
    try {
      exitCompare();
      exitSweep();
      const b = await loadBundleFromUrl(pack.demo_bundle_url);
      setBundle(b);
      onLoadError("");
      const url = new URL(window.location.href);
      url.searchParams.set("demo", pack.pack_id);
      url.searchParams.delete("bundle");
      window.history.replaceState({}, "", url.toString());
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
  };

  if (loadingCatalog) {
    return (
      <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm text-slate-500">
        Loading scenario catalog…
      </section>
    );
  }

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <CorpusBrowserPanel hooks={navigateHooks} />
      <CorpusEvolutionPanel hooks={navigateHooks} />
      <CompareCatalogSection onLoadError={onLoadError} onLoading={onLoading} />
      <SweepCatalogPicker onLoadError={onLoadError} onLoading={onLoading} />
      <h2 className="mb-2 font-semibold text-slate-200">Scenario catalog</h2>
      <ScenarioFilterBar tags={tagPool} activeTags={activeTags} onToggle={toggleTag} />
      <label className="mt-2 block text-slate-400">
        <span className="mb-1 block text-xs uppercase text-slate-500">Select scenario</span>
        <select
          className="w-full rounded border border-slate-600 bg-slate-800 px-2 py-1.5 text-slate-200"
          value={selectedId}
          onChange={(e) => {
            const pack = packs.find((p) => p.pack_id === e.target.value);
            if (pack) {
              setSelectedId(pack.pack_id);
              void loadPack(pack);
            }
          }}
        >
          {[...grouped.entries()].map(([cat, items]) => (
            <optgroup key={cat} label={CATEGORY_LABELS[cat] ?? cat}>
              {items.map((p) => (
                <option key={p.pack_id} value={p.pack_id}>
                  {p.title}
                  {p.replay_duration_class ? ` (${p.replay_duration_class})` : ""}
                </option>
              ))}
            </optgroup>
          ))}
        </select>
      </label>
      {selected && (
        <div className="mt-2">
          <ScenarioPreviewCard entry={selected} />
        </div>
      )}
      <p className="mt-2 text-[10px] text-slate-600">
        Explanatory replay fixtures only — not deployment or operational data.
      </p>
    </section>
  );
}
