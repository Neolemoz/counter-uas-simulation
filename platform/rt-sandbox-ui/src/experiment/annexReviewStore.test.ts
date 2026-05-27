import { beforeEach, describe, expect, it, vi } from "vitest";
import demoAnnex from "./fixtures/tactical_annex_demo_v1.json";
import {
  clearAnnexForRun,
  exportAnnexBundle,
  getAnnexForRun,
  importAnnexBundle,
  importAnnexBundleDetailed,
  importAnnexForRun,
  pruneAnnexCacheForManifest,
} from "./annexReviewStore";
import { createEmptyManifest } from "./experimentStore";

const RUN = "run-demo";
const memory: Record<string, string> = {};

vi.stubGlobal("localStorage", {
  getItem: (key: string) => memory[key] ?? null,
  setItem: (key: string, value: string) => {
    memory[key] = value;
  },
  removeItem: (key: string) => {
    delete memory[key];
  },
  clear: () => {
    for (const k of Object.keys(memory)) delete memory[k];
  },
});

describe("annexReviewStore", () => {
  beforeEach(() => {
    for (const k of Object.keys(memory)) delete memory[k];
    clearAnnexForRun(RUN);
  });

  it("imports annex for run", () => {
    importAnnexForRun(RUN, JSON.stringify(demoAnnex));
    const annex = getAnnexForRun(RUN);
    expect(annex?.selected_id).toBe("interceptor_0");
  });

  it("imports bundle", () => {
    const bundle = {
      schema: "rt_experiment_annex_bundle_v1",
      experiment_id: "exp-1",
      entries: [{ run_id: RUN, annex: demoAnnex }],
    };
    const n = importAnnexBundle(JSON.stringify(bundle));
    expect(n).toBe(1);
    expect(getAnnexForRun(RUN)?.assigned_target).toBe("threat_uav_0");
  });

  it("importAnnexBundleDetailed skips invalid entries", () => {
    const bundle = {
      schema: "rt_experiment_annex_bundle_v1",
      experiment_id: "exp-1",
      entries: [
        { run_id: RUN, annex: demoAnnex },
        { run_id: "run-bad", annex: { schema: "wrong" } },
      ],
    };
    const result = importAnnexBundleDetailed(JSON.stringify(bundle));
    expect(result.imported).toBe(1);
    expect(result.skipped).toBe(1);
    expect(result.errors.length).toBeGreaterThan(0);
    expect(getAnnexForRun(RUN)).not.toBeNull();
    expect(getAnnexForRun("run-bad")).toBeNull();
  });

  it("readCache skips corrupt per-run entries", () => {
    importAnnexForRun(RUN, JSON.stringify(demoAnnex));
    const key = "rt_experiment_annex_cache_v1";
    const cache = JSON.parse(memory[key] ?? "{}") as Record<string, unknown>;
    cache["orphan-bad"] = { not: "annex" };
    memory[key] = JSON.stringify(cache);
    expect(getAnnexForRun(RUN)?.selected_id).toBe("interceptor_0");
    expect(getAnnexForRun("orphan-bad")).toBeNull();
  });

  it("pruneAnnexCacheForManifest removes stale run keys", () => {
    importAnnexForRun(RUN, JSON.stringify(demoAnnex));
    importAnnexForRun("run-stale", JSON.stringify(demoAnnex));
    const manifest = createEmptyManifest("exp-1");
    manifest.runs.push({
      run_id: RUN,
      label: "demo",
      session_id: "sess-1",
      recorded_at_utc: "2026-05-26T12:00:00Z",
      snapshot: {},
    });
    const removed = pruneAnnexCacheForManifest(manifest);
    expect(removed).toBe(1);
    expect(getAnnexForRun(RUN)).not.toBeNull();
    expect(getAnnexForRun("run-stale")).toBeNull();
  });

  it("exportAnnexBundle includes cached runs", () => {
    importAnnexForRun(RUN, JSON.stringify(demoAnnex));
    const manifest = createEmptyManifest("exp-1");
    manifest.runs.push({
      run_id: RUN,
      label: "demo",
      session_id: "sess-1",
      recorded_at_utc: "2026-05-26T12:00:00Z",
      snapshot: {},
    });
    const text = exportAnnexBundle(manifest);
    const parsed = JSON.parse(text);
    expect(parsed.schema).toBe("rt_experiment_annex_bundle_v1");
    expect(parsed.entries).toHaveLength(1);
  });
});
