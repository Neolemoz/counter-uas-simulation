import { describe, expect, it } from "vitest";
import { readFileSync, readdirSync } from "node:fs";
import { join } from "node:path";
import {
  ALL_BANNERS,
  BANNER_ANALYTICS,
  BANNER_ANNEX_REVIEW,
  BANNER_CESIUM,
  BANNER_EXPERIMENT,
  BANNER_EXPERIMENT_F5,
  BANNER_FIDELITY_TRUTH,
  BANNER_INTERACTIVE_EDITING,
  BANNER_MANUAL_HANDOFF_ONLY,
  BANNER_MULTI_SESSION,
  BANNER_SA_WORKFLOW_ADVISORY,
  BANNER_TERRAIN,
  BANNER_VISIBILITY_V3,
  BANNER_WORLD_EDITING,
  bannersForSession,
  ADVISORY_FORBIDDEN_LEXICON,
  FORBIDDEN_LEXICON,
} from "@/governance/banners";
import { containsForbiddenLexicon } from "@/cesium/cognition";

describe("governance banners", () => {
  it("includes all required banners including world editing and Cesium", () => {
    expect(ALL_BANNERS).toContain(
      "RT SANDBOX — experimental simulation; not operational state",
    );
    expect(ALL_BANNERS).toContain(
      "TRANSIENT RUNTIME ONLY — not replay authority",
    );
    expect(ALL_BANNERS).toContain("NOT SA REPLAY AUTHORITY");
    expect(bannersForSession(false)).toContain(BANNER_MANUAL_HANDOFF_ONLY);
    expect(ALL_BANNERS).toContain(BANNER_WORLD_EDITING);
    expect(ALL_BANNERS).toContain(BANNER_CESIUM);
    expect(ALL_BANNERS).toContain(BANNER_INTERACTIVE_EDITING);
  });

  it("shows connected banners only when connected", () => {
    expect(bannersForSession(false)).not.toContain(BANNER_WORLD_EDITING);
    expect(bannersForSession(false)).not.toContain(BANNER_CESIUM);
    expect(bannersForSession(false)).not.toContain(BANNER_INTERACTIVE_EDITING);
    expect(bannersForSession(true)).toContain(BANNER_WORLD_EDITING);
    expect(bannersForSession(true)).toContain(BANNER_CESIUM);
    expect(bannersForSession(true)).toContain(BANNER_INTERACTIVE_EDITING);
  });

  it("shows multi-session banner when two or more sessions connected", () => {
    expect(bannersForSession(true, false)).not.toContain(BANNER_MULTI_SESSION);
    expect(bannersForSession(true, true)).toContain(BANNER_MULTI_SESSION);
  });

  it("terrain banner avoids forbidden lexicon", () => {
    expect(containsForbiddenLexicon(BANNER_TERRAIN)).toBe(false);
  });

  it("experiment banner is defined and avoids forbidden lexicon (PLAT-RT-X1)", () => {
    expect(BANNER_EXPERIMENT).toContain("explanatory compare");
    expect(containsForbiddenLexicon(BANNER_EXPERIMENT)).toBe(false);
  });

  it("analytics banner is defined and avoids forbidden lexicon (PLAT-RT-F1)", () => {
    expect(BANNER_ANALYTICS).toContain("derived summaries");
    expect(containsForbiddenLexicon(BANNER_ANALYTICS)).toBe(false);
  });

  it("experiment import guards avoid forbidden lexicon (PLAT-RT-F2)", () => {
    const path = join(process.cwd(), "src", "experiment", "experimentImportGuards.ts");
    const content = readFileSync(path, "utf-8").toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      const re = new RegExp(`\\b${term.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}\\b`, "i");
      expect(content).not.toMatch(re);
    }
  });

  it("annex review banner is defined and avoids forbidden lexicon (PLAT-RT-F3)", () => {
    expect(BANNER_ANNEX_REVIEW).toContain("replay-boundary");
    expect(containsForbiddenLexicon(BANNER_ANNEX_REVIEW)).toBe(false);
  });

  it("F5 experiment banner is defined and avoids forbidden lexicon (PLAT-RT-F5 P1)", () => {
    expect(BANNER_EXPERIMENT_F5).toContain("derived summaries");
    expect(containsForbiddenLexicon(BANNER_EXPERIMENT_F5)).toBe(false);
  });

  it("fidelity truth banner is defined and avoids forbidden lexicon (PLAT-RT-F5b P1)", () => {
    expect(BANNER_FIDELITY_TRUTH).toContain("sim-scoped attestation");
    expect(containsForbiddenLexicon(BANNER_FIDELITY_TRUTH)).toBe(false);
  });

  it("visibility v3 banner is defined and avoids forbidden lexicon (PLAT-RT-V3 P1)", () => {
    expect(BANNER_VISIBILITY_V3).toContain("heuristic");
    expect(containsForbiddenLexicon(BANNER_VISIBILITY_V3)).toBe(false);
  });

  it("registry budget summary is advisory only (PLAT-RT-V3 P2)", async () => {
    const { registryBudgetSummaryLine, defaultVisibilityFromRegistry } = await import(
      "@/cesium/visualLayerRegistry"
    );
    const line = registryBudgetSummaryLine(defaultVisibilityFromRegistry());
    expect(line.toLowerCase()).toContain("advisory");
    expect(containsForbiddenLexicon(line)).toBe(false);
  });

  it("SA workflow advisory banner is defined (PLAT-RT-F6 P1)", () => {
    expect(BANNER_SA_WORKFLOW_ADVISORY).toContain("maintainer CLIs are authority");
    expect(containsForbiddenLexicon(BANNER_SA_WORKFLOW_ADVISORY)).toBe(false);
    for (const term of ADVISORY_FORBIDDEN_LEXICON) {
      const re = new RegExp(`\\b${term.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}\\b`, "i");
      expect(BANNER_SA_WORKFLOW_ADVISORY.toLowerCase()).not.toMatch(re);
    }
  });

  it("UI source avoids forbidden lexicon", () => {
    const srcDir = join(process.cwd(), "src");
    const files = collectTsFiles(srcDir);
    const combined = files.map((f) => readFileSync(f, "utf-8")).join("\n");
    const lower = combined.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      const re = new RegExp(`\\b${term.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}\\b`, "i");
      expect(lower).not.toMatch(re);
    }
  });
});

function collectTsFiles(dir: string): string[] {
  const out: string[] = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const full = join(dir, entry.name);
    if (entry.isDirectory()) {
      out.push(...collectTsFiles(full));
    } else if (
      /\.(tsx?)$/.test(entry.name) &&
      !/\.test\.(tsx?)$/.test(entry.name)
    ) {
      if (entry.name === "banners.ts") continue;
      out.push(full);
    }
  }
  return out;
}
