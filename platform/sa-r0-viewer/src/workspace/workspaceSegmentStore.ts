import { create } from "zustand";
import type { WorkspaceSegment } from "./types";
import { resolveWorkspaceSegment, type SegmentRuntimeFlags } from "./resolveWorkspaceSegment";

type State = {
  userSegment: WorkspaceSegment | null;
  segmentBeforeMode: WorkspaceSegment | null;
  setUserSegment: (segment: WorkspaceSegment) => void;
  clearUserSegment: () => void;
  rememberSegmentForMode: () => void;
  restoreSegmentAfterMode: () => void;
  effectiveSegment: (flags: SegmentRuntimeFlags) => WorkspaceSegment;
};

export const useWorkspaceSegmentStore = create<State>((set, get) => ({
  userSegment: null,
  segmentBeforeMode: null,
  setUserSegment: (segment) => set({ userSegment: segment }),
  clearUserSegment: () => set({ userSegment: null, segmentBeforeMode: null }),
  rememberSegmentForMode: () => {
    const current = get().userSegment;
    if (current) set({ segmentBeforeMode: current });
  },
  restoreSegmentAfterMode: () => {
    const prev = get().segmentBeforeMode;
    if (prev) set({ userSegment: prev, segmentBeforeMode: null });
    else set({ segmentBeforeMode: null });
  },
  effectiveSegment: (flags) => resolveWorkspaceSegment(flags, get().userSegment),
}));
