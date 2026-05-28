import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import {
  AdvisoryStandupPassSelector,
  filterPresetForStandupPass,
} from "./AdvisoryStandupPassSelector";
import { STANDUP_PASSES } from "./advisoryAggregationV2";

describe("AdvisoryStandupPassSelector", () => {
  it("renders pass selector banner", () => {
    const markup = renderToStaticMarkup(
      <AdvisoryStandupPassSelector
        activePassId={null}
        onSelectPass={() => {}}
      />,
    );
    expect(markup).toContain("Pass selector ≠ CLI invocation");
    expect(markup).toContain("Pass A — Firefight");
  });

  it("maps pass E to import_advisory_only preset", () => {
    const passE = STANDUP_PASSES.find((p) => p.pass_id === "pass_e_import_advisory");
    expect(passE).toBeTruthy();
    const { preset, cohortHint } = filterPresetForStandupPass(passE!);
    expect(preset).toBe("import_advisory_only");
    expect(cohortHint).toBeNull();
  });

  it("maps pass D to needs_prepare cohort hint", () => {
    const passD = STANDUP_PASSES.find((p) => p.pass_id === "pass_d_package");
    expect(passD).toBeTruthy();
    const { cohortHint } = filterPresetForStandupPass(passD!);
    expect(cohortHint).toBe("needs_prepare");
  });
});
