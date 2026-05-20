import { create } from "zustand";
import type { ReplayMcSweep } from "./sweepSchema";

export type SpatialDeclutterMode = "off" | "top_k" | "threshold";

type SweepState = {
  mode: "off" | "sweep";
  sweep: ReplayMcSweep | null;
  memberIndex: number;
  activeCohortId: string | null;
  cohortFilterTags: string[];
  emphasizeAnomalies: boolean;
  spatialDeclutter: SpatialDeclutterMode;
  setSweep: (sweep: ReplayMcSweep | null) => void;
  setMemberIndex: (index: number) => void;
  setActiveCohortId: (id: string | null) => void;
  setCohortFilterTags: (tags: string[]) => void;
  toggleCohortFilterTag: (tag: string) => void;
  setEmphasizeAnomalies: (v: boolean) => void;
  setSpatialDeclutter: (mode: SpatialDeclutterMode) => void;
  exitSweep: () => void;
};

export const useSweepStore = create<SweepState>((set) => ({
  mode: "off",
  sweep: null,
  memberIndex: 0,
  activeCohortId: null,
  cohortFilterTags: [],
  emphasizeAnomalies: false,
  spatialDeclutter: "top_k",
  setSweep: (sweep) =>
    set({
      sweep,
      mode: sweep ? "sweep" : "off",
      memberIndex: 0,
      activeCohortId: null,
      cohortFilterTags: [],
    }),
  setMemberIndex: (memberIndex) => set({ memberIndex }),
  setActiveCohortId: (activeCohortId) => set({ activeCohortId }),
  setCohortFilterTags: (cohortFilterTags) => set({ cohortFilterTags }),
  toggleCohortFilterTag: (tag) =>
    set((s) => {
      const has = s.cohortFilterTags.includes(tag);
      return {
        cohortFilterTags: has
          ? s.cohortFilterTags.filter((t) => t !== tag)
          : [...s.cohortFilterTags, tag],
      };
    }),
  setEmphasizeAnomalies: (emphasizeAnomalies) => set({ emphasizeAnomalies }),
  setSpatialDeclutter: (spatialDeclutter) => set({ spatialDeclutter }),
  exitSweep: () =>
    set({
      mode: "off",
      sweep: null,
      memberIndex: 0,
      activeCohortId: null,
      cohortFilterTags: [],
      spatialDeclutter: "top_k",
    }),
}));

export function filteredMemberIndices(sweep: ReplayMcSweep): number[] {
  const tags = useSweepStore.getState().cohortFilterTags;
  const cohortId = useSweepStore.getState().activeCohortId;
  let indices = sweep.members.map((_, i) => i);

  if (cohortId) {
    const cohort = sweep.replay_cohorts?.find((c) => c.cohort_id === cohortId);
    if (cohort) indices = [...cohort.member_indices];
  }

  if (tags.length) {
    indices = indices.filter((i) => {
      const m = sweep.members[i];
      const mtags = m?.replay_pattern_tags ?? [];
      return tags.some((t) => mtags.includes(t));
    });
  }

  return indices.length ? indices : sweep.members.map((_, i) => i);
}
