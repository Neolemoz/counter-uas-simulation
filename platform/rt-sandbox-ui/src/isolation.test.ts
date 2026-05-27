import { describe, expect, it } from "vitest";
import { readFileSync, readdirSync, existsSync } from "node:fs";
import { join } from "node:path";

const REPO_ROOT = join(process.cwd(), "..", "..");
const RT_UI_ROOT = join(REPO_ROOT, "platform", "rt-sandbox-ui");
const SA_VIEWER = "platform/sa-r0-viewer";
const FORBIDDEN_IMPORT_PATTERNS = [
  /from\s+["']@?\/?.*sa-r0/,
  /from\s+["'].*\/replay\//,
  /from\s+["']@\/replay\//,
  /CesiumReplayMap/,
];

const FORBIDDEN_BRIDGE_CALLS = [
  "capture_session",
  "rt_sa_import",
  "handoff_import_committed",
] as const;

describe("RT UI isolation", () => {
  it("rt-sandbox-ui package exists", () => {
    expect(existsSync(RT_UI_ROOT)).toBe(true);
  });

  it("does not reference sa-r0-viewer in source", () => {
    const srcDir = join(RT_UI_ROOT, "src");
    const files = collectFiles(srcDir);
    for (const file of files) {
      if (file.endsWith("templateGuards.ts")) continue;
      const content = readFileSync(file, "utf-8");
      expect(content).not.toContain(SA_VIEWER);
      for (const pattern of FORBIDDEN_IMPORT_PATTERNS) {
        expect(content).not.toMatch(pattern);
      }
    }
  });

  it("bridge client does not invoke capture or SA import commands", () => {
    const bridgeDir = join(RT_UI_ROOT, "src", "bridge");
    for (const file of collectFiles(bridgeDir)) {
      const content = readFileSync(file, "utf-8");
      for (const forbidden of FORBIDDEN_BRIDGE_CALLS) {
        expect(content).not.toContain(forbidden);
      }
    }
  });

  it("bridge client may use read-only list_capture_handoff_status", () => {
    const clientPath = join(RT_UI_ROOT, "src", "bridge", "client.ts");
    const content = readFileSync(clientPath, "utf-8");
    expect(content).toContain("list_capture_handoff_status");
    expect(content).not.toContain("capture_session");
  });

  it("includes workstation shell modules (PLAT-RT-T4)", () => {
    expect(existsSync(join(RT_UI_ROOT, "src", "workstation", "RuntimeWorkstationShell.tsx"))).toBe(
      true,
    );
    expect(existsSync(join(RT_UI_ROOT, "src", "workflow", "captureHandoffCognition.ts"))).toBe(
      true,
    );
  });

  it("includes Cesium interactive editing modules (PLAT-RT-T5)", () => {
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "cesiumEditing.ts"))).toBe(true);
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "cameraHelpers.ts"))).toBe(true);
  });

  it("includes terrain realism modules (PLAT-RT-V2)", () => {
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "rtFictionalTerrain.ts"))).toBe(true);
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "terrainMeshLayer.ts"))).toBe(true);
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "sensorDomeLayer.ts"))).toBe(true);
    expect(
      existsSync(join(RT_UI_ROOT, "src", "cesium", "fixtures", "rt_ridge_terrain_v1.json")),
    ).toBe(true);
  });

  it("includes runtime realism expansion modules (PLAT-RT-F4)", () => {
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "terrainContourLayer.ts"))).toBe(true);
    expect(existsSync(join(RT_UI_ROOT, "src", "cesium", "losSegmentLayer.ts"))).toBe(true);
    const banners = readFileSync(join(RT_UI_ROOT, "src", "governance", "banners.ts"), "utf-8");
    expect(banners).toContain("BANNER_REALISM_F4");
  });

  it("includes fidelity truth cognition modules (PLAT-RT-F5b P1)", () => {
    expect(existsSync(join(RT_UI_ROOT, "src", "fidelity", "fidelityCognition.ts"))).toBe(
      true,
    );
    expect(
      existsSync(join(RT_UI_ROOT, "src", "components", "FidelityTruthCognitionStrip.tsx")),
    ).toBe(true);
    const banners = readFileSync(join(RT_UI_ROOT, "src", "governance", "banners.ts"), "utf-8");
    expect(banners).toContain("BANNER_FIDELITY_TRUTH");
  });

  it("includes platform hardening modules (PLAT-RT-F2)", () => {
    const expDir = join(RT_UI_ROOT, "src", "experiment");
    expect(existsSync(join(expDir, "experimentImportGuards.ts"))).toBe(true);
    expect(existsSync(join(expDir, "experimentIds.ts"))).toBe(true);
    const guards = readFileSync(join(expDir, "experimentImportGuards.ts"), "utf-8");
    expect(guards).not.toContain("capture_session");
  });

  it("includes experiment workbench modules (PLAT-RT-X1)", () => {
    const expDir = join(RT_UI_ROOT, "src", "experiment");
    expect(existsSync(join(expDir, "experimentSchema.ts"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentWorkbenchPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentBatchPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "analyticsDerive.ts"))).toBe(true);
    expect(existsSync(join(expDir, "sweepCompile.ts"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentAnalyticsPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "SweepCatalogBrowser.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "tacticalAnnexSchema.ts"))).toBe(true);
    expect(existsSync(join(expDir, "TacticalAnnexReviewPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentContinuityReviewPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "experimentImportGuards.ts"))).toBe(true);
    expect(existsSync(join(expDir, "experimentIds.ts"))).toBe(true);
    expect(existsSync(join(expDir, "annexReviewStore.ts"))).toBe(true);
    expect(existsSync(join(expDir, "experimentSpecCompile.ts"))).toBe(true);
    expect(existsSync(join(expDir, "metricsDerive.ts"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentFilterBar.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentMatrixPanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentExtendedComparePanel.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentHandoffEligibilityStrip.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "experimentF5UiHelpers.ts"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentRepeatabilityTrendStrip.tsx"))).toBe(true);
    expect(existsSync(join(expDir, "experimentRepeatabilityTrend.ts"))).toBe(true);
    expect(existsSync(join(expDir, "fidelityMetricsDerive.ts"))).toBe(true);
    expect(existsSync(join(expDir, "ExperimentFidelityCompareStrip.tsx"))).toBe(true);
    expect(existsSync(join(REPO_ROOT, "scripts", "rt", "rt_experiment_metrics.py"))).toBe(true);
    expect(
      existsSync(join(REPO_ROOT, "scripts", "rt", "rt_experiment_fidelity_metrics.py")),
    ).toBe(true);
    expect(existsSync(join(expDir, "templateGuards.ts"))).toBe(true);
    const banners = readFileSync(join(RT_UI_ROOT, "src", "governance", "banners.ts"), "utf-8");
    expect(banners).toContain("BANNER_EXPERIMENT_F5");
    const saViewerImport = /from\s+["'].*sa-r0-viewer/;
    for (const file of collectFiles(expDir)) {
      if (!file.endsWith(".ts") && !file.endsWith(".tsx")) continue;
      if (file.endsWith(".test.ts") || file.endsWith(".test.tsx")) continue;
      expect(readFileSync(file, "utf-8")).not.toMatch(saViewerImport);
    }
    const forbiddenCaptureInvoke = [
      /sendRtCommand\s*\(\s*["']capture_session/,
      /command_type:\s*["']capture_session/,
      /["']capture_session["']\s*\)/,
    ];
    for (const file of collectFiles(expDir)) {
      if (!file.endsWith(".ts") && !file.endsWith(".tsx")) continue;
      if (file.endsWith(".test.ts") || file.endsWith(".test.tsx")) continue;
      const content = readFileSync(file, "utf-8");
      for (const pattern of forbiddenCaptureInvoke) {
        expect(content).not.toMatch(pattern);
      }
    }
  });
});

function collectFiles(dir: string): string[] {
  const out: string[] = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const full = join(dir, entry.name);
    if (entry.isDirectory()) {
      out.push(...collectFiles(full));
    } else if (/\.(tsx?|json|html)$/.test(entry.name) && !entry.name.endsWith(".test.ts")) {
      out.push(full);
    }
  }
  return out;
}
