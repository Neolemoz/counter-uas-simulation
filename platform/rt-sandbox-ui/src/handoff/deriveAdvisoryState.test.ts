import { readFileSync, readdirSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { deriveAdvisoryState } from "./deriveAdvisoryState";
import type { AdvisoryDeriveInput } from "./advisoryTypes";

const FIXTURE_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_handoff/f6_advisory_examples",
);
const EXPECTED_DIR = join(FIXTURE_DIR, "expected");

function loadFixtureInput(raw: Record<string, unknown>): AdvisoryDeriveInput {
  const inputs = raw.inputs as Record<string, unknown>;
  if (inputs.capture_advisory_inputs) {
    return {
      ...(inputs.capture_advisory_inputs as AdvisoryDeriveInput),
      capture_candidate_id: "test-cap",
    };
  }
  return {
    ...(inputs as AdvisoryDeriveInput),
    capture_candidate_id:
      (inputs.capture_candidate_id as string | undefined) ?? "test-cap",
  };
}

describe("deriveAdvisoryState", () => {
  const fixtureFiles = readdirSync(FIXTURE_DIR).filter(
    (f) => f.endsWith(".json") && f !== "README.md",
  );

  for (const file of fixtureFiles) {
    it(`matches golden expected for ${file}`, () => {
      const raw = JSON.parse(readFileSync(join(FIXTURE_DIR, file), "utf8")) as {
        inputs: Record<string, unknown>;
      };
      const input = loadFixtureInput(raw);
      const expected = JSON.parse(
        readFileSync(join(EXPECTED_DIR, file.replace(".json", ".expected.json")), "utf8"),
      );
      const got = deriveAdvisoryState(input);
      expect(got.advisory_state).toBe(expected.advisory_state);
      expect(got.blocked).toBe(expected.blocked);
      expect(got.advisory_state_label).toBe(expected.advisory_state_label);
      if (expected.terminal) {
        expect(got.terminal).toBe(expected.terminal);
      }
    });
  }

  it("is deterministic", () => {
    const raw = JSON.parse(
      readFileSync(join(FIXTURE_DIR, "approval_ready.json"), "utf8"),
    ) as { inputs: Record<string, unknown> };
    const input = loadFixtureInput(raw);
    expect(deriveAdvisoryState(input)).toEqual(deriveAdvisoryState(input));
  });

  it("export handoff_ready event is not advisory handoff_ready", () => {
    const raw = JSON.parse(
      readFileSync(join(FIXTURE_DIR, "export_handoff_ready_pre_approve.json"), "utf8"),
    ) as { inputs: Record<string, unknown> };
    const got = deriveAdvisoryState(loadFixtureInput(raw));
    expect(got.advisory_state).toBe("capture_ready");
    expect(got.advisory_state).not.toBe("handoff_ready");
  });
});
