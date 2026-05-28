import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { buildReviewPacketPreview } from "./reviewPacketPreview";
import { reviewPacketSchema } from "./reviewPacketSchema";
import {
  buildPacketSectionsPreview,
  PACKET_SECTION_CATALOG,
} from "./reviewPacketSections";
import { defaultWorkbenchV2State } from "./workbenchV2State";
import type { ExperimentManifest } from "./experimentSchema";

const REPO_ROOT = join(import.meta.dirname, "../../../..");

describe("reviewPacketSections", () => {
  it("catalog has five section ids", () => {
    expect(PACKET_SECTION_CATALOG).toHaveLength(5);
  });

  it("builds preview sections without affecting packet export", () => {
    const manifest: ExperimentManifest = {
      schema: "rt_experiment_manifest_v1",
      experiment_id: "exp-test",
      created_at_utc: "2026-05-28T12:00:00Z",
      governance_banner: "RT EXPERIMENT — explanatory compare only; not operational authority",
      runs: [],
    };
    const v2State = defaultWorkbenchV2State();
    const sections = buildPacketSectionsPreview({
      v2State,
      manifest,
      presence: { f1_analytics: true, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(sections.map((s) => s.section_id)).toEqual([
      "scope",
      "reports",
      "compare_summary",
      "advisory_refs",
      "cli_hints",
    ]);
    const packet = buildReviewPacketPreview({
      v2State,
      manifest,
      presence: { f1_analytics: true, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(packet).not.toHaveProperty("sections");
  });

  it("parses x3 fixture with optional sections", () => {
    const raw = JSON.parse(
      readFileSync(
        join(REPO_ROOT, "fixtures/rt_experiments/x3_review_packet_sections_example.json"),
        "utf-8",
      ),
    );
    const parsed = reviewPacketSchema.parse(raw);
    expect(parsed.sections).toHaveLength(5);
  });
});
