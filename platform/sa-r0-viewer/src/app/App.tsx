import { useCallback, useEffect, useState } from "react";
import { GovernanceChrome } from "@/governance/GovernanceChrome";
import { CaveatsFooter } from "@/governance/CaveatsFooter";
import { loadBundleFromFile, loadBundleFromUrl, resolveInitialBundleUrl } from "@/replay/loadBundle";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { useClockStore } from "@/replay/clockStore";
import { useCompareStore } from "@/replay/compareStore";
import { tryResolveCompareFromUrl } from "@/replay/resolveCompareUrl";
import { tryResolveSweepFromUrl } from "@/replay/resolveSweepUrl";
import { SweepMetadataPanel } from "@/replay/SweepMetadataPanel";
import { OutcomeDistributionPanel } from "@/replay/analytics/OutcomeDistributionPanel";
import { LosDegradationSummary } from "@/replay/analytics/LosDegradationSummary";
import { ReplayVariabilityPanel } from "@/replay/analytics/ReplayVariabilityPanel";
import { ReplayClusterSummaryPanel } from "@/replay/analytics/ReplayClusterSummaryPanel";
import { MatchedSeedPanel } from "@/replay/analytics/MatchedSeedPanel";
import { useSweepStore } from "@/replay/useSweepStore";
import { ScenarioCatalogPicker } from "@/replay/ScenarioCatalogPicker";
import { MetadataPanel } from "@/replay/MetadataPanel";
import { LayerToggles } from "@/replay/LayerToggles";
import { TimelineScrubber } from "@/replay/timeline/TimelineScrubber";
import { NarrativeTimeline } from "@/replay/narrative/NarrativeTimeline";
import { AnnotationsPanel } from "@/replay/narrative/AnnotationsPanel";
import { StrategicMapPane } from "@/views/StrategicMapPane";
import { RadarMockPane } from "@/views/RadarMockPane";
import { EoIrMockPane } from "@/views/EoIrMockPane";
import { InterceptorCameraMockPane } from "@/views/InterceptorCameraMockPane";
import { TelemetryPanel } from "@/views/TelemetryPanel";
import { ThreatAssessmentPanel } from "@/views/ThreatAssessmentPanel";
import { CompareView } from "@/replay/compare/CompareView";
import { useCohortFilmstripStore } from "@/replay/cohortFilmstripStore";
import { CohortFilmstripView } from "@/replay/filmstrip/CohortFilmstripView";
import { SweepWorkstationShell } from "@/replay/workstation/SweepWorkstationShell";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { PresentationView } from "@/replay/presentation/PresentationView";
import { tryResolvePresentationFromUrl } from "@/replay/presentation/resolvePresentationUrl";

export function App() {
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const bundle = useClockStore((s) => s.bundle);
  const setBundle = useClockStore((s) => s.setBundle);
  const playing = useClockStore((s) => s.playing);
  const playbackMs = useClockStore((s) => s.playbackMs);
  const tickPlayback = useClockStore((s) => s.tickPlayback);
  const compareMode = useCompareStore((s) => s.mode);
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);
  const sweepMode = useSweepStore((s) => s.mode === "sweep");
  const filmstripMode = useCohortFilmstripStore((s) => s.mode === "filmstrip");
  const presentationMode = usePresentationStore((s) => s.mode === "presentation");

  const applyBundle = useCallback(
    (b: ReplaySaBundle) => {
      useCompareStore.getState().exitCompare();
      useSweepStore.getState().exitSweep();
      setBundle(b);
      setError(null);
    },
    [setBundle],
  );

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const inSweep = await tryResolveSweepFromUrl();
        if (cancelled) return;
        if (!inSweep) {
          const inCompare = await tryResolveCompareFromUrl();
          if (cancelled) return;
          if (!inCompare) {
            const url = await resolveInitialBundleUrl();
            const b = await loadBundleFromUrl(url);
            if (!cancelled) applyBundle(b);
          }
        }
        if (!cancelled) {
          await tryResolvePresentationFromUrl();
          setError(null);
        }
      } catch (e: unknown) {
        if (!cancelled) setError(String(e));
      } finally {
        if (!cancelled) setLoading(false);
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [applyBundle]);

  useEffect(() => {
    if (compareMode === "compare" || !playing || presentationMode) return;
    const id = window.setInterval(tickPlayback, playbackMs);
    return () => window.clearInterval(id);
  }, [playing, playbackMs, tickPlayback, compareMode, presentationMode]);

  const onFile = async (file: File) => {
    setLoading(true);
    try {
      applyBundle(await loadBundleFromFile(file));
      await tryResolvePresentationFromUrl();
    } catch (e: unknown) {
      setError(String(e));
    } finally {
      setLoading(false);
    }
  };

  const headerBundle = compareMode === "compare" ? slotA.bundle ?? slotB.bundle : bundle;

  return (
    <div className="flex h-full min-h-screen flex-col">
      <GovernanceChrome
        bundle={headerBundle}
        compareMode={compareMode === "compare"}
        presentationMode={presentationMode}
        compareTitles={
          compareMode === "compare"
            ? [slotA.bundle?.scenario.title, slotB.bundle?.scenario.title]
            : undefined
        }
      />
      <div className="border-b border-slate-800 bg-slate-900/50 px-4 py-2">
        <label className="text-sm text-slate-400">
          Load replay bundle (JSON):{" "}
          <input
            type="file"
            accept=".json,application/json"
            className="text-slate-200"
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) void onFile(f);
            }}
          />
        </label>
        {loading && <span className="ml-2 text-sm text-slate-500">Loading…</span>}
        {error && <p className="mt-1 text-sm text-red-400">{error}</p>}
      </div>
      {compareMode === "compare" && slotA.bundle && slotB.bundle ? (
        <CompareView />
      ) : filmstripMode ? (
        <CohortFilmstripView />
      ) : presentationMode && bundle ? (
        <PresentationView />
      ) : bundle ? (
        <main className="grid min-h-0 flex-1 grid-cols-1 gap-3 p-3 lg:grid-cols-12">
          <aside className="flex flex-col gap-3 lg:col-span-3">
            <ScenarioCatalogPicker onLoadError={setError} onLoading={setLoading} />
            {sweepMode && <SweepMetadataPanel />}
            {sweepMode && <SweepWorkstationShell />}
            <MetadataPanel bundle={bundle} />
            {sweepMode && (
              <>
                <OutcomeDistributionPanel />
                <LosDegradationSummary />
                <ReplayVariabilityPanel />
                <ReplayClusterSummaryPanel />
                <MatchedSeedPanel />
              </>
            )}
            <LayerToggles />
            <TimelineScrubber />
            <AnnotationsPanel />
          </aside>
          <div className="flex flex-col gap-3 lg:col-span-6">
            <StrategicMapPane bundle={bundle} />
            <NarrativeTimeline />
          </div>
          <aside className="grid gap-3 lg:col-span-3">
            <RadarMockPane />
            <EoIrMockPane />
            <InterceptorCameraMockPane />
            <TelemetryPanel />
            <ThreatAssessmentPanel />
          </aside>
        </main>
      ) : (
        !loading && (
          <p className="p-8 text-center text-slate-500">
            No bundle loaded. Use ?bundle=URL, ?sweep=id, ?pair=id, ?compare=packA,packB, ?presentation=id, or demo catalog.
          </p>
        )
      )}
      <CaveatsFooter compareMode={compareMode === "compare"} />
    </div>
  );
}
