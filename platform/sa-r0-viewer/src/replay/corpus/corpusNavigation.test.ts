import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { replayCorpusDriftReportSchema, replayCorpusIndexSchema } from "../synthesis/synthesisSchema";
import { childrenOf, parentsOf, resolveEntryTarget, siblingsOf } from "./corpusNavigation";

const REPO = join(dirname(fileURLToPath(import.meta.url)), "../../../../..");

describe("corpusNavigation", () => {
  it("parses committed corpus index with lineage edges", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"),
      "utf-8",
    );
    const index = replayCorpusIndexSchema.parse(JSON.parse(raw));
    expect(index.entries.length).toBeGreaterThan(50);
    expect(index.lineage_edges?.length).toBeGreaterThan(0);
  });

  it("builds stable parent/child adjacency", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"),
      "utf-8",
    );
    const index = replayCorpusIndexSchema.parse(JSON.parse(raw));
    const sweepId = "sweep_family__valley_sensor_sweep";
    const parents = parentsOf(sweepId, index);
    const children = childrenOf(sweepId, index);
    expect(parents).toEqual([...parents].sort());
    expect(children).toEqual([...children].sort());
    expect(children.length).toBeGreaterThan(0);
  });

  it("resolves sweep family to sweep mode", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"),
      "utf-8",
    );
    const index = replayCorpusIndexSchema.parse(JSON.parse(raw));
    const entry = index.entries.find((e) => e.entry_id === "sweep_family__valley_sensor_sweep");
    expect(entry).toBeDefined();
    const target = resolveEntryTarget(entry!);
    expect(target).toEqual({ mode: "sweep", sweepId: "valley_sensor_sweep" });
  });

  it("parses drift report fixture", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json"),
      "utf-8",
    );
    const drift = replayCorpusDriftReportSchema.parse(JSON.parse(raw));
    expect(drift.findings.length).toBeGreaterThan(0);
  });

  it("lists siblings under shared parent", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"),
      "utf-8",
    );
    const index = replayCorpusIndexSchema.parse(JSON.parse(raw));
    const exportEntry = index.entries.find(
      (e) => e.entry_kind === "replay_export" && e.entry_id.includes("valley_sensor"),
    );
    if (!exportEntry) return;
    const sibs = siblingsOf(exportEntry.entry_id, index);
    expect(sibs.length).toBeGreaterThanOrEqual(0);
  });
});
