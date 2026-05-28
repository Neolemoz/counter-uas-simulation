import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ExperimentUnifiedReviewPanel } from "./ExperimentUnifiedReviewPanel";
import { defaultWorkbenchV2State } from "./workbenchV2State";
import type { ExperimentRun } from "./experimentSchema";

const runs: ExperimentRun[] = [
  {
    run_id: "run-a",
    label: "A",
    session_id: "s1",
    recorded_at_utc: "2026-05-28T12:00:00Z",
    snapshot: {},
  },
];

describe("ExperimentUnifiedReviewPanel", () => {
  it("renders unified review lane", () => {
    const markup = renderToStaticMarkup(
      <ExperimentUnifiedReviewPanel
        v2State={defaultWorkbenchV2State()}
        onV2StateChange={() => {}}
        runs={runs}
        onActivateStep={() => {}}
        onContinuityRunId={() => {}}
        onSyncComparePinned={() => {}}
      />,
    );
    expect(markup).toContain("Unified review lane");
    expect(markup).toContain("F1 analytics");
  });
});
