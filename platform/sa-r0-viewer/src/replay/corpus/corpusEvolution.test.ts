import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import {
  replayCorpusEvolutionManifestSchema,
  replayCorpusEvolutionSummarySchema,
} from "../synthesis/synthesisSchema";

const REPO = join(dirname(fileURLToPath(import.meta.url)), "../../../../..");

describe("replay corpus evolution fixtures", () => {
  it("parses evolution manifest with stable chronology tier order", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json"),
      "utf-8",
    );
    const manifest = replayCorpusEvolutionManifestSchema.parse(JSON.parse(raw) as unknown);
    const tierIds = manifest.chronology_tiers.map((t) => t.tier_id);
    expect(tierIds).toEqual([
      "topology_baseline",
      "topology_demo",
      "sweep_wave_d2",
      "presentation_e1",
      "export_e1",
      "synthesis_e2",
      "corpus_release",
    ]);
    expect(manifest.releases.length).toBeGreaterThanOrEqual(2);
  });

  it("parses evolution summary fixture", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_evolution_summary_v1.json"),
      "utf-8",
    );
    const summary = replayCorpusEvolutionSummarySchema.parse(JSON.parse(raw) as unknown);
    expect(summary.corpus_id).toBe("sa_r0_corpus_r1");
    expect((summary.divergence_chronology ?? []).length).toBeGreaterThan(0);
  });
});
