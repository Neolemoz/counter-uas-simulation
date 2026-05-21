import { loadBundleFromUrl } from "../loadBundle";
import { loadScenarioCatalog, demoUrlFromPackId } from "../loadCatalog";
import { loadSweepManifest } from "../loadSweep";
import { loadStoryboard } from "../presentation/resolvePresentationUrl";
import { usePresentationStore } from "../presentation/presentationStore";
import { loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import type { CorpusIndexEntry } from "../synthesis/synthesisSchema";
import { useClockStore } from "../clockStore";
import { useCompareStore } from "../compareStore";
import { useCohortFilmstripStore } from "../cohortFilmstripStore";
import { useSweepStore } from "../useSweepStore";
import { resolveEntryTarget } from "./corpusNavigation";
import { setCorpusEntryUrlParam, useCorpusStore } from "./useCorpusStore";

export type NavigateHooks = {
  onLoading: (loading: boolean) => void;
  onLoadError: (msg: string) => void;
};

export async function navigateToCorpusEntry(
  entry: CorpusIndexEntry,
  hooks: NavigateHooks,
): Promise<void> {
  const { onLoading, onLoadError } = hooks;
  onLoading(true);
  try {
    useCorpusStore.getState().setSelectedEntryId(entry.entry_id);
    setCorpusEntryUrlParam(entry.entry_id);

    const target = resolveEntryTarget(entry);
    if (!target || target.mode === "corpus_info") {
      onLoadError("");
      return;
    }

    useCompareStore.getState().exitCompare();
    useCohortFilmstripStore.getState().exitFilmstrip();
    usePresentationStore.getState().exitPresentation();

    const url = new URL(window.location.href);

    if (target.mode === "demo") {
      useSweepStore.getState().exitSweep();
      const catalog = await loadScenarioCatalog();
      const demoUrl = demoUrlFromPackId(catalog, target.packId);
      if (!demoUrl) throw new Error(`No demo URL for pack ${target.packId}`);
      const bundle = await loadBundleFromUrl(demoUrl);
      useClockStore.getState().setBundle(bundle);
      url.searchParams.set("demo", target.packId);
      url.searchParams.delete("sweep");
      url.searchParams.delete("presentation");
      url.searchParams.delete("compare");
    } else if (target.mode === "sweep") {
      const sweep = await loadSweepManifest(target.sweepId);
      useSweepStore.getState().setSweep(sweep);
      const member = sweep.members[0];
      if (!member) throw new Error(`Sweep ${target.sweepId} has no members`);
      const bundle = await loadBundleFromUrl(member.demo_bundle_url);
      useClockStore.getState().setBundle(bundle);
      useSweepStore.getState().setMemberIndex(0);
      url.searchParams.set("sweep", target.sweepId);
      url.searchParams.delete("demo");
      url.searchParams.delete("presentation");
    } else if (target.mode === "presentation") {
      useSweepStore.getState().exitSweep();
      const storyboard = await loadStoryboard(target.storyboardId);
      usePresentationStore.getState().enterPresentation(storyboard, 0);
      url.searchParams.set("presentation", target.storyboardId);
      url.searchParams.delete("demo");
      url.searchParams.delete("sweep");
    }

    window.history.replaceState({}, "", url.toString());
    onLoadError("");
  } catch (e: unknown) {
    onLoadError(String(e));
  } finally {
    onLoading(false);
  }
}

export async function navigateToCorpusEntryById(
  entryId: string,
  hooks: NavigateHooks,
): Promise<void> {
  const index = await loadReplayCorpusIndex();
  const entry = index.entries.find((e) => e.entry_id === entryId);
  if (!entry) throw new Error(`Unknown corpus entry: ${entryId}`);
  await navigateToCorpusEntry(entry, hooks);
}
