import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ExperimentCompareStagePanel } from "./ExperimentCompareStagePanel";
import { defaultWorkbenchV2State } from "./workbenchV2State";

describe("ExperimentCompareStagePanel", () => {
  it("renders compare mode select", () => {
    const markup = renderToStaticMarkup(
      <ExperimentCompareStagePanel
        v2State={defaultWorkbenchV2State()}
        onV2StateChange={() => {}}
        onApplyCompareMode={() => {}}
      />,
    );
    expect(markup).toContain("Compare mode");
    expect(markup).toContain("Pinned");
    expect(markup).toContain("Side-by-side");
    expect(markup).toContain("Multi-manifest");
  });
});
