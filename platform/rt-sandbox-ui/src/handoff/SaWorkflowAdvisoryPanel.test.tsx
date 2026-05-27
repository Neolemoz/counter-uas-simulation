import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ADVISORY_FORBIDDEN_LEXICON } from "@/governance/banners";
import { deriveAdvisoryState } from "./deriveAdvisoryState";
import type { AdvisoryDeriveInput } from "./advisoryTypes";
import { SaWorkflowAdvisoryPanel } from "./SaWorkflowAdvisoryPanel";

const FIXTURE_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_handoff/f6_advisory_examples",
);

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

describe("SaWorkflowAdvisoryPanel", () => {
  it("renders checklist and avoids forbidden lexicon", () => {
    const raw = JSON.parse(
      readFileSync(join(FIXTURE_DIR, "approval_ready.json"), "utf8"),
    ) as { inputs: Record<string, unknown> };
    const status = deriveAdvisoryState(loadFixtureInput(raw));
    const markup = renderToStaticMarkup(
      <SaWorkflowAdvisoryPanel status={status} selectedCaptureId="test-cap" />,
    );
    const text = markup.toLowerCase();
    for (const term of ADVISORY_FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="sa-workflow-advisory-panel"');
    expect(markup).toContain("Normalization");
    expect(markup).not.toMatch(/type="submit"/);
  });
});
