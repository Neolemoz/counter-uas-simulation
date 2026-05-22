import { loadBundleFromUrl } from "@/replay/loadBundle";
import { loadSweepManifest } from "@/replay/loadSweep";
import { loadScenarioCatalog, demoUrlFromPackId } from "@/replay/loadCatalog";
import { loadComparePairs, pairById } from "@/replay/loadComparePairs";
import { useCompareStore } from "@/replay/compareStore";
import { useClockStore } from "@/replay/clockStore";
import { useCohortFilmstripStore } from "@/replay/cohortFilmstripStore";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { useSweepStore } from "@/replay/useSweepStore";
import type { NavigateHooks } from "@/replay/corpus/navigateToCorpusEntry";
import {
  navigateToCorpusEntry,
  navigateToCorpusEntryById,
} from "@/replay/corpus/navigateToCorpusEntry";
import { loadReplayCorpusIndex } from "@/replay/synthesis/loadSynthesis";
import type { ExperimentRunQueue } from "@/orchestration/orchestrationSchema";
import type { ReplayMcSweep } from "@/replay/sweepSchema";
import { useWorkspaceSegmentStore } from "@/workspace/workspaceSegmentStore";
import type { WorkspaceSegment } from "@/workspace/types";

export type ExperimentNavHooks = NavigateHooks & {
  /** After navigation, optionally set workspace segment. */
  segment?: WorkspaceSegment;
};

function replaceUrlParams(params: Record<string, string | null>) {
  if (typeof window === "undefined") return;
  const url = new URL(window.location.href);
  for (const [k, v] of Object.entries(params)) {
    if (v == null) url.searchParams.delete(k);
    else url.searchParams.set(k, v);
  }
  window.history.replaceState({}, "", url.toString());
}

export function syncOrchestrationQueueUrl(queueId: string | null) {
  replaceUrlParams({ orchestration_queue: queueId });
}

/** Map orchestration corpus_ref slug to corpus entry_id when possible. */
export async function resolveCorpusEntryIdFromRef(ref: string): Promise<string | null> {
  const index = await loadReplayCorpusIndex();
  const direct = index.entries.find((e) => e.entry_id === ref);
  if (direct) return direct.entry_id;
  const prefixed = `demo_bundle__${ref}`;
  if (index.entries.some((e) => e.entry_id === prefixed)) return prefixed;
  const byPack = index.entries.find((e) => {
    const scope = e.replay_scope as { pack_id?: string } | undefined;
    return scope?.pack_id === ref || e.entry_id.endsWith(`__${ref}`);
  });
  return byPack?.entry_id ?? null;
}

/** Best-effort corpus_entry for a loaded demo pack. */
export async function corpusEntryIdForPackId(packId: string): Promise<string | null> {
  return resolveCorpusEntryIdFromRef(packId);
}

export function packIdFromBundlePath(bundlePath: string): string | null {
  const norm = bundlePath.replace(/\\/g, "/");
  const m = norm.match(/demo_([a-z0-9_]+)\/?$/i) ?? norm.match(/\/([a-z0-9_]+)\/index\.json$/i);
  if (!m) return null;
  const slug = m[1]!;
  if (slug.startsWith("demo_")) return slug.slice(5);
  return slug;
}

export async function navigateToScenarioPack(
  packId: string,
  hooks: ExperimentNavHooks,
): Promise<void> {
  const { onLoading, onLoadError, segment = "replay" } = hooks;
  onLoading(true);
  try {
    useCompareStore.getState().exitCompare();
    useCohortFilmstripStore.getState().exitFilmstrip();
    usePresentationStore.getState().exitPresentation();
    useSweepStore.getState().exitSweep();

    const catalog = await loadScenarioCatalog();
    const demoUrl = demoUrlFromPackId(catalog, packId);
    if (!demoUrl) throw new Error(`No demo URL for pack ${packId}`);

    const bundle = await loadBundleFromUrl(demoUrl);
    useClockStore.getState().setBundle(bundle);

    const corpusEntry = await corpusEntryIdForPackId(packId);
    replaceUrlParams({
      demo: packId,
      bundle: null,
      sweep: null,
      pair: null,
      compare: null,
      presentation: null,
      walkthrough: null,
      corpus_entry: corpusEntry,
    });

    useWorkspaceSegmentStore.getState().setUserSegment(segment);
    onLoadError("");
  } catch (e: unknown) {
    onLoadError(String(e));
  } finally {
    onLoading(false);
  }
}

export async function navigateToComparePair(
  pairId: string,
  hooks: ExperimentNavHooks,
): Promise<void> {
  const { onLoading, onLoadError } = hooks;
  onLoading(true);
  try {
    const pairs = await loadComparePairs();
    const pair = pairById(pairs, pairId);
    if (!pair) throw new Error(`Unknown compare pair: ${pairId}`);

    const [a, b] = await Promise.all([
      loadBundleFromUrl(pair.slot_a.demo_bundle_url),
      loadBundleFromUrl(pair.slot_b.demo_bundle_url),
    ]);
    useSweepStore.getState().exitSweep();
    usePresentationStore.getState().exitPresentation();
    useCompareStore.getState().enterCompare(a, b, pairId);

    replaceUrlParams({
      pair: pairId,
      demo: null,
      compare: null,
      sweep: null,
      presentation: null,
      walkthrough: null,
    });
    useWorkspaceSegmentStore.getState().setUserSegment("compare");
    onLoadError("");
  } catch (e: unknown) {
    onLoadError(String(e));
  } finally {
    onLoading(false);
  }
}

export async function navigateFromQueueJob(
  job: ExperimentRunQueue["jobs"][number],
  hooks: ExperimentNavHooks,
): Promise<void> {
  if (job.provenance?.corpus_ref) {
    const entryId = await resolveCorpusEntryIdFromRef(job.provenance.corpus_ref);
    if (entryId) {
      await navigateToCorpusEntryById(entryId, hooks);
      return;
    }
  }
  if (job.scenario_pack_id) {
    await navigateToScenarioPack(job.scenario_pack_id, hooks);
    return;
  }
  if (job.provenance?.bundle_path) {
    const packId = packIdFromBundlePath(job.provenance.bundle_path);
    if (packId) {
      await navigateToScenarioPack(packId, hooks);
      return;
    }
  }
}

export type SweepWalkthroughStep = NonNullable<
  ReplayMcSweep["presentation_walkthrough"]
>["steps"][number];

export async function applySweepWalkthroughStep(
  step: SweepWalkthroughStep,
  hooks: ExperimentNavHooks,
): Promise<void> {
  const sweep = useSweepStore.getState().sweep;
  if (!sweep) return;

  if (step.kind === "cohort_filmstrip") {
    const indices =
      step.filmstrip_indices?.length
        ? step.filmstrip_indices
        : sweep.members.map((_, i) => i);
    await useCohortFilmstripStore.getState().enterFilmstrip(sweep, indices);
    replaceUrlParams({ filmstrip: sweep.sweep_id, pair: null, compare: null });
    useWorkspaceSegmentStore.getState().setUserSegment("replay");
    return;
  }

  if (step.kind === "compare_pair") {
    const idxA = 0;
    const idxB = step.member_index ?? 1;
    const memberA = sweep.members[idxA];
    const memberB = sweep.members[idxB];
    if (!memberA || !memberB) return;
    const { onLoading, onLoadError } = hooks;
    onLoading(true);
    try {
      const [a, b] = await Promise.all([
        loadBundleFromUrl(memberA.demo_bundle_url),
        loadBundleFromUrl(memberB.demo_bundle_url),
      ]);
      usePresentationStore.getState().exitPresentation();
      useCompareStore.getState().enterCompare(a, b, null);
      replaceUrlParams({
        compare: `${memberA.pack_id ?? "a"},${memberB.pack_id ?? "b"}`,
        pair: null,
        demo: null,
        sweep: sweep.sweep_id,
      });
      useWorkspaceSegmentStore.getState().setUserSegment("compare");
      onLoadError("");
    } catch (e: unknown) {
      onLoadError(String(e));
    } finally {
      onLoading(false);
    }
    return;
  }

  if (step.kind === "chapter") {
    usePresentationStore.getState().enterBundleWalkthrough(step.chapter_index ?? 0);
    useWorkspaceSegmentStore.getState().setUserSegment("report");
    return;
  }
}

export async function navigateSweepById(sweepId: string, hooks: ExperimentNavHooks) {
  const { onLoading, onLoadError } = hooks;
  onLoading(true);
  try {
    const sweep = await loadSweepManifest(sweepId);
    useCompareStore.getState().exitCompare();
    usePresentationStore.getState().exitPresentation();
    useSweepStore.getState().setSweep(sweep);
    const member = sweep.members[0];
    if (!member) throw new Error(`Sweep ${sweepId} has no members`);
    const bundle = await loadBundleFromUrl(member.demo_bundle_url);
    useClockStore.getState().setBundle(bundle);
    useSweepStore.getState().setMemberIndex(0);
    replaceUrlParams({
      sweep: sweepId,
      demo: null,
      pair: null,
      compare: null,
      presentation: null,
    });
    useWorkspaceSegmentStore.getState().setUserSegment("replay");
    onLoadError("");
  } catch (e: unknown) {
    onLoadError(String(e));
  } finally {
    onLoading(false);
  }
}

export { navigateToCorpusEntry, navigateToCorpusEntryById };
