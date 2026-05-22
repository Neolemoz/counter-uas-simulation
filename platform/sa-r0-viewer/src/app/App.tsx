import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { GovernanceChrome } from "@/governance/GovernanceChrome";
import { CaveatsFooter } from "@/governance/CaveatsFooter";
import { loadBundleFromFile, loadBundleFromUrl, resolveInitialBundleUrl } from "@/replay/loadBundle";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { useClockStore } from "@/replay/clockStore";
import { useCompareStore } from "@/replay/compareStore";
import { tryResolveCompareFromUrl } from "@/replay/resolveCompareUrl";
import { tryResolveSweepFromUrl } from "@/replay/resolveSweepUrl";
import { useSweepStore } from "@/replay/useSweepStore";
import { useCohortFilmstripStore } from "@/replay/cohortFilmstripStore";
import { CohortFilmstripView } from "@/replay/filmstrip/CohortFilmstripView";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { PresentationView } from "@/replay/presentation/PresentationView";
import { tryResolvePresentationFromUrl } from "@/replay/presentation/resolvePresentationUrl";
import { tryResolveCorpusEntryFromUrl } from "@/replay/corpus/resolveCorpusEntryUrl";
import type { NavigateHooks } from "@/replay/corpus/navigateToCorpusEntry";
import { readCorpusEntryFromUrl } from "@/replay/corpus/useCorpusStore";
import { SegmentNav } from "@/workspace/SegmentNav";
import { useWorkspaceSegmentStore } from "@/workspace/workspaceSegmentStore";
import type { WorkspaceSegment } from "@/workspace/types";
import { ReplayWorkspaceView } from "@/workspace/views/ReplayWorkspaceView";
import { ScenarioWorkspaceView } from "@/workspace/views/ScenarioWorkspaceView";
import { CorpusWorkspaceView } from "@/workspace/views/CorpusWorkspaceView";
import { CompareWorkspaceView } from "@/workspace/views/CompareWorkspaceView";
import { ReportWorkspaceView } from "@/workspace/views/ReportWorkspaceView";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { readOrchestrationQueueFromUrl } from "@/orchestration/loadOrchestration";
import { useAuthoringMirror } from "@/authoring/useAuthoringMirror";
import { loadAuthoringManifest } from "@/authoring/loadAuthoring";
import { loadValidationMirror } from "@/orchestration/loadOrchestration";

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

  const setUserSegment = useWorkspaceSegmentStore((s) => s.setUserSegment);
  const rememberSegmentForMode = useWorkspaceSegmentStore((s) => s.rememberSegmentForMode);
  const restoreSegmentAfterMode = useWorkspaceSegmentStore((s) => s.restoreSegmentAfterMode);
  const effectiveSegment = useWorkspaceSegmentStore((s) => s.effectiveSegment);

  const runtimeFlags = useMemo(
    () => ({
      compareMode: compareMode === "compare",
      presentationMode,
      filmstripMode,
      sweepMode,
      hasBundle: Boolean(bundle),
    }),
    [compareMode, presentationMode, filmstripMode, sweepMode, bundle],
  );

  const segment = effectiveSegment(runtimeFlags);
  const lockedSegment: WorkspaceSegment | null = runtimeFlags.compareMode
    ? "compare"
    : runtimeFlags.presentationMode
      ? "report"
      : null;

  const applyBundle = useCallback(
    (b: ReplaySaBundle) => {
      useCompareStore.getState().exitCompare();
      useSweepStore.getState().exitSweep();
      setBundle(b);
      setError(null);
    },
    [setBundle],
  );

  const navigateHooks: NavigateHooks = useMemo(
    () => ({
      onLoading: setLoading,
      onLoadError: setError,
    }),
    [],
  );

  const experimentHooks: ExperimentNavHooks = useMemo(
    () => ({
      ...navigateHooks,
    }),
    [navigateHooks],
  );

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const corpusEntry = readCorpusEntryFromUrl();
        if (corpusEntry) {
          const inCorpus = await tryResolveCorpusEntryFromUrl(navigateHooks);
          if (cancelled) return;
          if (inCorpus) {
            await tryResolvePresentationFromUrl();
            if (!cancelled) setError(null);
            return;
          }
        }
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
  }, [applyBundle, navigateHooks]);

  useEffect(() => {
    if (compareMode === "compare" || !playing || presentationMode) return;
    const id = window.setInterval(tickPlayback, playbackMs);
    return () => window.clearInterval(id);
  }, [playing, playbackMs, tickPlayback, compareMode, presentationMode]);

  const prevModesRef = useRef({
    compare: compareMode === "compare",
    presentation: presentationMode,
    filmstrip: filmstripMode,
  });
  useEffect(() => {
    const prev = prevModesRef.current;
    const nowCompare = compareMode === "compare";
    const nowPresentation = presentationMode;
    const nowFilmstrip = filmstripMode;

    if (
      (nowCompare && !prev.compare) ||
      (nowPresentation && !prev.presentation) ||
      (nowFilmstrip && !prev.filmstrip)
    ) {
      rememberSegmentForMode();
      if (nowCompare) setUserSegment("compare");
      if (nowPresentation) setUserSegment("report");
    } else if (
      (prev.compare && !nowCompare) ||
      (prev.presentation && !nowPresentation) ||
      (prev.filmstrip && !nowFilmstrip)
    ) {
      restoreSegmentAfterMode();
    }

    prevModesRef.current = {
      compare: nowCompare,
      presentation: nowPresentation,
      filmstrip: nowFilmstrip,
    };
  }, [
    compareMode,
    presentationMode,
    filmstripMode,
    rememberSegmentForMode,
    restoreSegmentAfterMode,
    setUserSegment,
  ]);

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
  const activePackId = bundle?.scenario.catalog_pack_id ?? null;
  const urlDemoPackId =
    typeof window !== "undefined"
      ? new URLSearchParams(window.location.search).get("demo")
      : null;
  const packForAuthoring = activePackId ?? urlDemoPackId;
  const authoringMirror = useAuthoringMirror(
    segment === "scenario" ? packForAuthoring : null,
  );
  const [validationOk, setValidationOk] = useState<boolean | null>(null);

  useEffect(() => {
    if (segment !== "scenario" || !packForAuthoring) {
      setValidationOk(null);
      return;
    }
    let cancelled = false;
    void (async () => {
      const [mirror, manifest] = await Promise.all([
        loadValidationMirror(packForAuthoring).catch(() => null),
        loadAuthoringManifest(packForAuthoring).catch(() => null),
      ]);
      if (cancelled) return;
      if (!mirror) {
        setValidationOk(null);
        return;
      }
      const stale =
        manifest?.validation_pack_fingerprint &&
        manifest.updated_at &&
        mirror.checked_at &&
        manifest.updated_at > mirror.checked_at;
      setValidationOk(Boolean(mirror.ok) && !stale);
    })();
    return () => {
      cancelled = true;
    };
  }, [segment, packForAuthoring]);

  const queueId = readOrchestrationQueueFromUrl();

  const workspaceBody = () => {
    if (filmstripMode) {
      return <CohortFilmstripView />;
    }
    if (presentationMode && bundle) {
      return <PresentationView />;
    }
    if (segment === "compare") {
      return (
        <CompareWorkspaceView
          onLoadError={setError}
          onLoading={setLoading}
          hooks={experimentHooks}
        />
      );
    }
    if (segment === "scenario") {
      return (
        <ScenarioWorkspaceView
          onLoadError={setError}
          onLoading={setLoading}
          hooks={experimentHooks}
        />
      );
    }
    if (segment === "corpus") {
      return (
        <CorpusWorkspaceView
          onLoadError={setError}
          onLoading={setLoading}
          navigateHooks={navigateHooks}
          experimentHooks={experimentHooks}
        />
      );
    }
    if (segment === "report" && !presentationMode) {
      return <ReportWorkspaceView hooks={experimentHooks} />;
    }
    if (bundle && segment === "replay") {
      return (
        <ReplayWorkspaceView
          bundle={bundle}
          onLoadError={setError}
          onLoading={setLoading}
          navigateHooks={navigateHooks}
          experimentHooks={experimentHooks}
        />
      );
    }
    if (!loading) {
      return (
        <p className="p-8 text-center text-slate-500">
          No bundle loaded. Use ?bundle=URL, ?sweep=id, ?corpus_entry=id, ?pair=id,
          ?compare=packA,packB, ?presentation=id, or open Scenario / Corpus to browse fixtures.
        </p>
      );
    }
    return null;
  };

  return (
    <div className="flex h-full min-h-screen flex-col">
      <GovernanceChrome
        bundle={headerBundle}
        compareMode={compareMode === "compare"}
        presentationMode={presentationMode}
        workspaceSegment={segment}
        compareTitles={
          compareMode === "compare"
            ? [slotA.bundle?.scenario.title, slotB.bundle?.scenario.title]
            : undefined
        }
        scenarioPackId={activePackId}
        orchestrationQueueId={queueId}
        authoringMirror={authoringMirror && segment === "scenario"}
        validationOk={segment === "scenario" ? validationOk : null}
      />
      <SegmentNav
        active={segment}
        locked={lockedSegment}
        authoringMirror={authoringMirror && segment === "scenario"}
        onSelect={(seg) => {
          if (lockedSegment) return;
          setUserSegment(seg);
        }}
      />
      <div className="sandbox-no-print sandbox-loader-strip">
        <label>
          Load replay bundle (JSON):{" "}
          <input
            type="file"
            accept=".json,application/json"
            className="ml-1 text-slate-300"
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) void onFile(f);
            }}
          />
        </label>
        {loading && <span className="ml-2 text-sm text-slate-500">Loading…</span>}
        {error && <p className="mt-1 text-sm text-red-400">{error}</p>}
      </div>
      {workspaceBody()}
      <CaveatsFooter compareMode={compareMode === "compare"} />
    </div>
  );
}
