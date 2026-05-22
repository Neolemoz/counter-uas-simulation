import { useEffect, useMemo, useState } from "react";
import { loadBundleFromUrl } from "@/replay/loadBundle";
import { loadScenarioCatalog, allCatalogTags } from "@/replay/loadCatalog";
import type { CatalogPack, ScenarioCatalog } from "@/replay/catalogSchema";
import { CATEGORY_LABELS } from "@/replay/catalogSchema";
import { ScenarioFilterBar } from "@/replay/ScenarioFilterBar";
import { ScenarioPreviewCard } from "@/replay/ScenarioPreviewCard";
import { useClockStore } from "@/replay/clockStore";
import { useCompareStore } from "@/replay/compareStore";
import { useSweepStore } from "@/replay/useSweepStore";
import { OrchestrationStatusPanel } from "@/orchestration/OrchestrationStatusPanel";
import { ValidationStatusPanel } from "@/orchestration/ValidationStatusPanel";
import { AuthoringPromotionPanel } from "@/authoring/AuthoringPromotionPanel";
import { AuthoringLineagePanel } from "@/authoring/AuthoringLineagePanel";
import { AuthoringTopologyInspector } from "@/authoring/AuthoringTopologyInspector";
import { AuthoringOrchestrationHandoffPanel } from "@/authoring/AuthoringOrchestrationHandoffPanel";
import { AuthoringIntegrityPanel } from "@/authoring/AuthoringIntegrityPanel";
import { loadAuthoringManifest } from "@/authoring/loadAuthoring";
import { OrchestrationAsyncPanel } from "@/orchestration/OrchestrationAsyncPanel";
import { OrchestrationBatchReviewPanel } from "@/orchestration/OrchestrationBatchReviewPanel";
import { OrchestrationRecoveryPanel } from "@/orchestration/OrchestrationRecoveryPanel";
import { OrchestrationLifecyclePanel } from "@/orchestration/OrchestrationLifecyclePanel";
import { OrchestrationIntegrityPanel } from "@/orchestration/OrchestrationIntegrityPanel";
import { OrchestrationReplayContinuityPanel } from "@/orchestration/OrchestrationReplayContinuityPanel";
import { readOrchestrationQueueFromUrl } from "@/orchestration/loadOrchestration";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { useWorkspaceSegmentStore } from "../workspaceSegmentStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { corpusEntryIdForPackId } from "@/navigation/experimentNavigation";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  hooks?: ExperimentNavHooks;
  /** When true, switch to Replay segment after loading a pack. */
  handoffToReplay?: boolean;
};

function packMatchesFilters(pack: CatalogPack, activeTags: string[]): boolean {
  if (activeTags.length === 0) return true;
  const pool = new Set([...pack.topology_tags, ...(pack.replay_tags ?? [])]);
  return activeTags.every((t) => pool.has(t));
}

export function ScenarioDiscoverSection({
  onLoadError,
  onLoading,
  hooks,
  handoffToReplay = false,
}: Props) {
  const [catalog, setCatalog] = useState<ScenarioCatalog | null>(null);
  const [activeTags, setActiveTags] = useState<string[]>([]);
  const [selectedId, setSelectedId] = useState<string>("");
  const [loadingCatalog, setLoadingCatalog] = useState(true);
  const [handoffManifestId, setHandoffManifestId] = useState<string | null>(null);
  const [handoffQueueId, setHandoffQueueId] = useState<string | null>(null);

  useEffect(() => {
    if (!selectedId) {
      setHandoffManifestId(null);
      setHandoffQueueId(null);
      return;
    }
    loadAuthoringManifest(selectedId)
      .then((m) => {
        const ref = m?.orchestration_handoff_refs?.[0];
        setHandoffManifestId(ref?.manifest_id ?? null);
        setHandoffQueueId(ref?.queue_mirror_id ?? readOrchestrationQueueFromUrl());
      })
      .catch(() => {
        setHandoffManifestId(null);
        setHandoffQueueId(readOrchestrationQueueFromUrl());
      });
  }, [selectedId]);
  const bundle = useClockStore((s) => s.bundle);
  const setBundle = useClockStore((s) => s.setBundle);
  const exitCompare = useCompareStore((s) => s.exitCompare);
  const exitSweep = useSweepStore((s) => s.exitSweep);

  useEffect(() => {
    loadScenarioCatalog()
      .then((cat) => {
        setCatalog(cat);
        const params = new URLSearchParams(window.location.search);
        const demo = params.get("demo") ?? params.get("authoring_pack");
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

  const loadPack = async (pack: CatalogPack) => {
    onLoading(true);
    try {
      exitCompare();
      exitSweep();
      const b = await loadBundleFromUrl(pack.demo_bundle_url);
      setBundle(b);
      onLoadError("");
      const corpusEntry = await corpusEntryIdForPackId(pack.pack_id);
      const url = new URL(window.location.href);
      url.searchParams.set("demo", pack.pack_id);
      url.searchParams.delete("bundle");
      if (corpusEntry) url.searchParams.set("corpus_entry", corpusEntry);
      window.history.replaceState({}, "", url.toString());
      if (handoffToReplay) {
        useWorkspaceSegmentStore.getState().setUserSegment("replay");
      }
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
  };

  if (loadingCatalog) {
    return (
      <CollapsiblePanelSection id="discover.scenario_catalog" title="Scenario catalog" defaultCollapsed={false}>
        <p className="text-slate-500">Loading scenario catalog…</p>
      </CollapsiblePanelSection>
    );
  }

  return (
    <>
    <CollapsiblePanelSection
      id="discover.scenario_catalog"
      title="Scenario catalog"
      defaultCollapsed={false}
      helper="Topology fixture packs — explanatory replay only."
    >
      <ScenarioFilterBar
        tags={tagPool}
        activeTags={activeTags}
        onToggle={(tag) =>
          setActiveTags((prev) =>
            prev.includes(tag) ? prev.filter((t) => t !== tag) : [...prev, tag],
          )
        }
      />
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
          <ScenarioPreviewCard entry={selected} hooks={hooks} />
        </div>
      )}
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.promotion_status"
      title="Promotion status"
      tier="t4"
      defaultCollapsed={false}
      helper="CLI-authored manifest mirror — not deployment state."
    >
      <AuthoringPromotionPanel packId={selectedId || null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.topology_inspector"
      title="Topology inspector"
      tier="t4"
      defaultCollapsed
    >
      <AuthoringTopologyInspector pack={selected ?? null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.lineage"
      title="Pack lineage"
      tier="t4"
      defaultCollapsed
    >
      <AuthoringLineagePanel pack={selected ?? null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.validation_mirror"
      title="Validation linkage"
      tier="t4"
      defaultCollapsed
      helper="Links authoring manifest to H3 validation mirror."
    >
      <ValidationStatusPanel scenarioPackId={selectedId || null} hooks={hooks} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.orchestration_handoff"
      title="Orchestration handoff"
      tier="t4"
      defaultCollapsed
    >
      <AuthoringOrchestrationHandoffPanel packId={selectedId || null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="authoring.integrity"
      title="Authoring integrity"
      tier="t4"
      defaultCollapsed
      helper="Corpus-wide audit mirror — CLI authoritative."
    >
      <AuthoringIntegrityPanel packId={selectedId || null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.lifecycle"
      title="Orchestration lifecycle"
      tier="t4"
      defaultCollapsed
      helper="Ops sidecar ladder — CLI authoritative."
    >
      <OrchestrationLifecyclePanel manifestId={handoffManifestId} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.async"
      title="Async orchestration"
      tier="t4"
      defaultCollapsed
      helper="Async execution plane — read-only; no orchestration controls."
    >
      <OrchestrationAsyncPanel manifestId={handoffManifestId} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.recovery"
      title="Async recovery"
      tier="t4"
      defaultCollapsed
      helper="Retry chains, claims, quarantine review — read-only."
    >
      <OrchestrationRecoveryPanel manifestId={handoffManifestId} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.batch_review"
      title="Async batch review"
      tier="t4"
      defaultCollapsed
      helper="Corpus recovery summary — explanatory only."
    >
      <OrchestrationBatchReviewPanel />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.integrity"
      title="Orchestration integrity"
      tier="t4"
      defaultCollapsed
    >
      <OrchestrationIntegrityPanel manifestId={handoffManifestId} packId={selectedId || null} />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="orchestration.replay_continuity"
      title="Replay continuity"
      tier="t4"
      defaultCollapsed
    >
      <OrchestrationReplayContinuityPanel
        manifestId={handoffManifestId}
        queueId={handoffQueueId}
      />
    </CollapsiblePanelSection>
    <CollapsiblePanelSection
      id="discover.job_status"
      title="Experiment queue"
      tier="t4"
      defaultCollapsed
    >
      <OrchestrationStatusPanel hooks={hooks} />
    </CollapsiblePanelSection>
    </>
  );
}
