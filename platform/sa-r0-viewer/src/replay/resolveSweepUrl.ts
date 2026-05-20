import { loadBundleFromUrl } from "./loadBundle";
import { loadSweepManifest } from "./loadSweep";
import { useClockStore } from "./clockStore";
import { useCompareStore } from "./compareStore";
import { useCohortFilmstripStore } from "./cohortFilmstripStore";
import { useSweepStore } from "./useSweepStore";

function parseFilmstripIndices(param: string | null, max: number): number[] | null {
  if (!param) return null;
  const parts = param.split(",").map((s) => parseInt(s.trim(), 10));
  const indices = parts.filter((n) => !Number.isNaN(n) && n >= 0 && n < max);
  if (indices.length < 2 || indices.length > 4) return null;
  return indices;
}

export async function tryResolveSweepFromUrl(): Promise<boolean> {
  const params = new URLSearchParams(window.location.search);
  const sweepId = params.get("sweep");
  if (!sweepId) return false;

  const sweep = await loadSweepManifest(sweepId);
  const members = sweep.members;
  if (!members.length) return false;

  useCompareStore.getState().exitCompare();
  useSweepStore.getState().setSweep(sweep);

  const cohortParam = params.get("cohort");
  if (cohortParam && sweep.replay_cohorts) {
    const cohort = sweep.replay_cohorts.find((c) => c.cohort_id === cohortParam);
    if (cohort && cohort.member_indices.length >= 2 && cohort.member_indices.length <= 4) {
      useSweepStore.getState().setActiveCohortId(cohort.cohort_id);
      await useCohortFilmstripStore.getState().enterFilmstrip(sweep, cohort.member_indices);
      return true;
    }
  }

  const filmstripIndices = parseFilmstripIndices(params.get("filmstrip"), members.length);
  if (filmstripIndices) {
    await useCohortFilmstripStore.getState().enterFilmstrip(sweep, filmstripIndices);
    return true;
  }

  useCohortFilmstripStore.getState().exitFilmstrip();

  const memberParam = params.get("member");
  const memberIndex = memberParam != null ? Math.max(0, parseInt(memberParam, 10) || 0) : 0;
  const idx = Math.min(memberIndex, members.length - 1);
  const member = members[idx]!;

  useSweepStore.getState().setMemberIndex(idx);
  const bundle = await loadBundleFromUrl(member.demo_bundle_url);
  useClockStore.getState().setBundle(bundle);
  return true;
}
