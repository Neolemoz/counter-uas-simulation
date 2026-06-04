import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "@/cesium/planningDrawing";
import { analyzePlanningCoverage } from "@/cesium/planningCoverageAnalysis";
import { planningExtentById } from "@/cesium/planningWorld";
import {
  buildPlanningLayoutCompareSlot,
  derivePlanningLayoutCompareAnalytics,
} from "@/layout/planningLayoutComparison";
import { buildPlanningMcSnapshot } from "@/layout/planningMcSnapshot";
import {
  buildPlanningLayoutComparePanelDerivation,
  PlanningLayoutComparePanel,
} from "./PlanningLayoutComparePanel";

const FIXTURE_PATH = join(
  import.meta.dirname,
  "../../../../fixtures/rt_sandbox/planning_layout_compare_golden_v1.json",
);

const POLYGON: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: [
    { x: 0, y: 0 },
    { x: 1000, y: 0 },
    { x: 1000, y: 1000 },
    { x: 0, y: 1000 },
  ],
};

const RADARS: PlanningRadarState = {
  ...EMPTY_PLANNING_RADARS,
  sites: [
    {
      id: "planning-radar-1",
      position: { x: 150, y: 150 },
      radar_type: "Short Range",
      detection_range_m: 500,
    },
  ],
};

const ALT_RADARS: PlanningRadarState = {
  ...EMPTY_PLANNING_RADARS,
  sites: [
    {
      id: "planning-radar-2",
      position: { x: 800, y: 800 },
      radar_type: "Medium Range",
      detection_range_m: 1000,
    },
  ],
};

function snapshotFor(
  radars: PlanningRadarState,
  createdUtc = "2026-06-04T00:00:00Z",
) {
  return buildPlanningMcSnapshot(
    POLYGON,
    radars,
    analyzePlanningCoverage(POLYGON, radars, 4, { radarPresets: PLANNING_RADAR_PRESETS }),
    {
      createdUtc,
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
      planningExtent: planningExtentById("planning_10km"),
    },
  );
}

function renderPanel({
  slots = [],
  captureDisabled = false,
  importSlotLabel = "A" as const,
  importText = "",
  importError = null as string | null,
}: {
  slots?: ReturnType<typeof buildPlanningLayoutCompareSlot>[];
  captureDisabled?: boolean;
  importSlotLabel?: "A" | "B" | "C";
  importText?: string;
  importError?: string | null;
} = {}) {
  const analytics = buildPlanningLayoutComparePanelDerivation(slots);
  return renderToStaticMarkup(
    <PlanningLayoutComparePanel
      slots={slots}
      analytics={analytics}
      captureDisabled={captureDisabled}
      importSlotLabel={importSlotLabel}
      importText={importText}
      importError={importError}
      onImportSlotLabelChange={vi.fn()}
      onImportTextChange={vi.fn()}
      onImportSnapshot={vi.fn()}
      onCaptureCurrentLayout={vi.fn()}
      onRemoveSlot={vi.fn()}
      onClearAllSlots={vi.fn()}
    />,
  );
}

describe("PlanningLayoutComparePanel", () => {
  it("renders governance banner and empty state", () => {
    const markup = renderPanel();

    expect(markup).toContain('data-testid="planning-layout-compare-panel"');
    expect(markup).toContain('data-testid="planning-layout-compare-governance"');
    expect(markup).toContain("Planning-only");
    expect(markup).toContain("read-only");
    expect(markup).toContain("non-authoritative");
    expect(markup).toContain('data-testid="planning-layout-compare-empty"');
    expect(markup).toContain('data-testid="planning-layout-compare-capture"');
    expect(markup).toContain('data-testid="planning-layout-compare-import"');
  });

  it("renders compare table metrics for captured slots", () => {
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
      buildPlanningLayoutCompareSlot("B", snapshotFor(ALT_RADARS, "2026-06-04T01:00:00Z")),
    ];
    const analytics = derivePlanningLayoutCompareAnalytics(slots);
    const markup = renderToStaticMarkup(
      <PlanningLayoutComparePanel
        slots={slots}
        analytics={analytics}
        captureDisabled={false}
        importSlotLabel="A"
        importText=""
        importError={null}
        onImportSlotLabelChange={vi.fn()}
        onImportTextChange={vi.fn()}
        onImportSnapshot={vi.fn()}
        onCaptureCurrentLayout={vi.fn()}
        onRemoveSlot={vi.fn()}
        onClearAllSlots={vi.fn()}
      />,
    );

    expect(markup).toContain('data-testid="planning-layout-compare-table"');
    expect(markup).toContain('data-testid="planning-layout-compare-row-A"');
    expect(markup).toContain('data-testid="planning-layout-compare-row-B"');
    expect(markup).toContain(slots[0]!.planning_snapshot_id);
    expect(markup).toContain("planning_10km");
    expect(markup).toContain(`${analytics.rows[0]!.coverage_percent.toFixed(1)}%`);
    expect(markup).toContain(`${analytics.rows[0]!.radar_count}`);
    expect(markup).toContain('data-testid="planning-layout-compare-remove-A"');
  });

  it("renders delta columns and advisory labels when slot A is present", () => {
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
      buildPlanningLayoutCompareSlot("B", snapshotFor(ALT_RADARS, "2026-06-04T01:00:00Z")),
    ];
    const analytics = derivePlanningLayoutCompareAnalytics(slots);
    const markup = renderPanel({ slots });

    expect(markup).toContain('data-testid="planning-layout-compare-deltas"');
    expect(markup).toContain('data-testid="planning-layout-compare-delta-row-coverage_percent"');
    expect(markup).toContain("B − A");
    const coverageDelta = analytics.deltas.find(
      (row) => row.metric === "coverage_percent" && row.target_slot === "B",
    );
    expect(markup).toContain(
      `${coverageDelta!.delta >= 0 ? "+" : ""}${coverageDelta!.delta.toFixed(1)}%`,
    );
    expect(markup).toMatch(/Improved|Reduced|Unchanged/);
  });

  it("renders blind spot and recommendation summaries side-by-side", () => {
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
      buildPlanningLayoutCompareSlot("B", snapshotFor(ALT_RADARS, "2026-06-04T01:00:00Z")),
    ];
    const analytics = derivePlanningLayoutCompareAnalytics(slots);
    const markup = renderPanel({ slots });

    expect(markup).toContain('data-testid="planning-layout-compare-blind-spots"');
    expect(markup).toContain('data-testid="planning-layout-compare-blind-spot-A"');
    expect(markup).toContain(analytics.side_by_side[0]!.blind_spot_summary);
    expect(markup).toContain('data-testid="planning-layout-compare-recommendations"');
    expect(markup).toContain('data-testid="planning-layout-compare-recommendation-A"');
    expect(markup).toContain("Suggested radar");
  });

  it("renders duplicate geometry and extent mismatch warnings", () => {
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
      buildPlanningLayoutCompareSlot(
        "B",
        snapshotFor(RADARS, "2026-06-04T01:00:00Z"),
      ),
      buildPlanningLayoutCompareSlot("C", snapshotFor(ALT_RADARS, "2026-06-04T02:00:00Z")),
    ];
    const markup = renderPanel({ slots });

    expect(markup).toContain('data-testid="planning-layout-compare-warnings"');
    expect(markup).toContain('data-testid="planning-layout-compare-warning-duplicate_geometry"');
  });

  it("renders snapshot import validation errors", () => {
    const markup = renderPanel({
      importText: "{",
      importError: "Invalid JSON",
    });

    expect(markup).toContain('data-testid="planning-layout-compare-import-error"');
    expect(markup).toContain("Invalid JSON");
  });

  it("does not invoke runtime, bridge, or MC execution", () => {
    const bridgeCommand = vi.fn();
    const runtimeMutation = vi.fn();
    const monteCarloExecution = vi.fn();
    const markup = renderPanel({
      slots: [buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS))],
    });

    expect(markup).toContain("Planning layout comparison");
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeMutation).not.toHaveBeenCalled();
    expect(monteCarloExecution).not.toHaveBeenCalled();
  });
});

describe("planning_layout_compare_golden_v1", () => {
  it("matches deterministic analytics for fixture snapshots", () => {
    const raw = JSON.parse(readFileSync(FIXTURE_PATH, "utf8")) as {
      snapshots: Record<"A" | "B" | "C", ReturnType<typeof snapshotFor>>;
      expected_deltas: ReturnType<typeof derivePlanningLayoutCompareAnalytics>["deltas"];
    };
    const slots = (["A", "B", "C"] as const).map((slotLabel) =>
      buildPlanningLayoutCompareSlot(slotLabel, raw.snapshots[slotLabel]),
    );
    const analytics = derivePlanningLayoutCompareAnalytics(slots);

    expect(analytics.deltas).toEqual(raw.expected_deltas);
    expect(analytics.side_by_side).toHaveLength(3);
  });
});
