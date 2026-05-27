import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  backgroundFidelityLabel,
  domeTruthSummary,
  extractFidelityContext,
  fidelityHubLine,
  losDivergenceBadge,
  losTruthSummary,
  mergeFidelityContextFromPayloads,
  partialTruthFlags,
  poseTruthDriftRows,
  poseTruthDriftSummary,
  truthFreshnessSummary,
  truthStaleBadge,
} from "./fidelityCognition";

const FIXTURES = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5b_fidelity_examples",
);

function loadJson(name: string): Record<string, unknown> {
  return JSON.parse(readFileSync(join(FIXTURES, name), "utf-8")) as Record<
    string,
    unknown
  >;
}

describe("fidelityCognition", () => {
  it("extracts default-off context", () => {
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: false,
      fidelity_attestation_status: "unavailable",
    });
    expect(ctx.enableFidelityCoupling).toBe(false);
    expect(fidelityHubLine(ctx)).toBe("Fidelity: off (stub)");
    expect(truthStaleBadge(ctx)).toBeNull();
  });

  it("extracts coupling-on context from clear snapshot fixture", () => {
    const truth = loadJson("truth_snapshot_clear.json");
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "available",
      fidelity_label: "truth_attested",
      fidelity_truth: truth,
      source: "bridge_registry",
      authority_label: "command_authoritative",
    });
    expect(fidelityHubLine(ctx)).toBe("Fidelity: truth_attested (sim)");
    expect(losTruthSummary(ctx)).toContain("clear");
    expect(domeTruthSummary(ctx)).toContain("radar_north");
    expect(truthFreshnessSummary(ctx)).toContain("available");
  });

  it("detects stale truth badge", () => {
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "stale",
      fidelity_truth: loadJson("truth_snapshot_clear.json"),
    });
    expect(truthStaleBadge(ctx)?.label).toBe("stale truth");
    expect(backgroundFidelityLabel(ctx)).toBe("on · stale");
  });

  it("computes pose drift rows from pose block fixture", () => {
    const blockFixture = loadJson("fidelity_pose_block_example.json");
    const block = blockFixture.fidelity_pose_block as Record<string, unknown>;
    const perEntity = (block.per_entity as Array<Record<string, unknown>>) ?? [];
    const entityTruth = perEntity.map((row) => ({
      entity_id: row.entity_id,
      truth_attested_pose: row.truth_attested_pose,
      sim_agl_m: row.sim_agl_m,
    }));
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "available",
      fidelity_truth: {
        ...loadJson("truth_snapshot_clear.json"),
        entity_truth: entityTruth,
      },
    });
    const worldSummary = {
      entities: perEntity.map((row) => ({
        entity_id: row.entity_id,
        pose: row.command_pose,
      })),
    };
    const rows = poseTruthDriftRows(ctx, worldSummary);
    expect(rows.length).toBe(2);
    const summary = poseTruthDriftSummary(rows);
    expect(summary.count).toBe(2);
    expect(summary.maxDriftM).toBeGreaterThan(0.3);
  });

  it("flags LOS divergence against heuristic label", () => {
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "available",
      fidelity_truth: loadJson("truth_snapshot_divergent.json"),
    });
    expect(losDivergenceBadge(ctx, "clear")?.label).toBe(
      "cognition_truth_divergence",
    );
    expect(losDivergenceBadge(ctx, "terrain_blocked")).toBeNull();
  });

  it("merges fidelity context preferring coupling-on payload", () => {
    const offPayload = { enable_fidelity_coupling: false };
    const onPayload = {
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "available",
      fidelity_truth: loadJson("truth_snapshot_clear.json"),
    };
    expect(
      mergeFidelityContextFromPayloads(offPayload, onPayload).enableFidelityCoupling,
    ).toBe(true);
  });

  it("surfaces partial truth when coupling on without snapshot", () => {
    const ctx = extractFidelityContext({
      enable_fidelity_coupling: true,
      fidelity_attestation_status: "unavailable",
    });
    expect(partialTruthFlags(ctx)).toContain("partial_truth");
  });
});
