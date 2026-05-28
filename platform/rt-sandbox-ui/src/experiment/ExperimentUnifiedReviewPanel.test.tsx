import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ExperimentUnifiedReviewPanel } from "./ExperimentUnifiedReviewPanel";
import { createEmptyManifest } from "./experimentStore";
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
        manifest={createEmptyManifest("exp-review")}
        presence={{
          f1_analytics: false,
          f3_annex: false,
          f5_metrics: false,
          f5b_fidelity: false,
        }}
        runs={runs}
        onActivateStep={() => {}}
        onContinuityRunId={() => {}}
        onSyncComparePinned={() => {}}
      />,
    );
    expect(markup).toContain("Unified review lane");
    expect(markup).toContain("F1 analytics");
    expect(markup).toContain('data-completion="missing"');
  });
});
