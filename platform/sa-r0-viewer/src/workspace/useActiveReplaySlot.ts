import { useCompareStore } from "@/replay/compareStore";
import { useClockStore } from "@/replay/clockStore";
import type { ReplaySlotState } from "@/replay/compareStore";

/** Active replay clock for mock panes — compare focus slot or main clock (H1 Option A). */
export function useActiveReplaySlot(): Pick<
  ReplaySlotState,
  "bundle" | "currentT" | "playing" | "selectedEventId" | "highlightedTrackIds"
> {
  const compareMode = useCompareStore((s) => s.mode === "compare");
  const focusSlot = useCompareStore((s) => s.focusSlot);
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);

  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  const playing = useClockStore((s) => s.playing);
  const selectedEventId = useClockStore((s) => s.selectedEventId);
  const highlightedTrackIds = useClockStore((s) => s.highlightedTrackIds);

  if (compareMode) {
    const slot = focusSlot === "A" ? slotA : slotB;
    return {
      bundle: slot.bundle,
      currentT: slot.currentT,
      playing: slot.playing,
      selectedEventId: slot.selectedEventId,
      highlightedTrackIds: slot.highlightedTrackIds,
    };
  }

  return {
    bundle,
    currentT,
    playing,
    selectedEventId,
    highlightedTrackIds,
  };
}
