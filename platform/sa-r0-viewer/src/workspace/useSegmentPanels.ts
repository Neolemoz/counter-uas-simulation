import type { ReactNode } from "react";
import { useMemo, useState } from "react";
import type { PanelId } from "./panelRegistry";
import {
  MAX_PRIMARY_RAIL_SECTIONS,
  defaultCollapsedForPanel,
  primaryPanelsForSegment,
  segmentAllowsPanel,
} from "./panelRegistry";
import type { WorkspaceSegment } from "./types";

export type SegmentPanelItem = {
  id: PanelId;
  primary: boolean;
  defaultCollapsed: boolean;
  node: ReactNode;
};

export function useSegmentPanels(segment: WorkspaceSegment, items: SegmentPanelItem[]) {
  const allowed = useMemo(
    () => items.filter((item) => segmentAllowsPanel(segment, item.id)),
    [segment, items],
  );

  const primaryIds = useMemo(() => new Set(primaryPanelsForSegment(segment)), [segment]);

  const enriched = useMemo(
    () =>
      allowed.map((item) => ({
        ...item,
        primary: item.primary ?? primaryIds.has(item.id),
        defaultCollapsed:
          item.defaultCollapsed ?? defaultCollapsedForPanel(segment, item.id),
      })),
    [allowed, primaryIds, segment],
  );

  const primary = enriched.filter((e) => e.primary);
  const secondary = enriched.filter((e) => !e.primary);
  const [moreOpen, setMoreOpen] = useState(false);

  const showMoreDrawer = primary.length > MAX_PRIMARY_RAIL_SECTIONS;

  return {
    primary: primary.slice(0, MAX_PRIMARY_RAIL_SECTIONS),
    overflowPrimary: primary.slice(MAX_PRIMARY_RAIL_SECTIONS),
    secondary,
    moreOpen,
    setMoreOpen,
    showMoreDrawer,
  };
}
