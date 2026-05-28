import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ExperimentReportDockPanel } from "./ExperimentReportDockPanel";
import { createEmptyManifest } from "./experimentStore";
import { defaultWorkbenchV2State } from "./workbenchV2State";

describe("ExperimentReportDockPanel", () => {
  it("renders report dock and packet tab labels", () => {
    const markup = renderToStaticMarkup(
      <ExperimentReportDockPanel
        v2State={defaultWorkbenchV2State()}
        manifest={createEmptyManifest("exp-dock")}
        presence={{
          f1_analytics: false,
          f3_annex: false,
          f5_metrics: false,
          f5b_fidelity: false,
        }}
        previews={{}}
        onImportSlot={() => null}
        onExportSlot={() => null}
      />,
    );
    expect(markup).toContain("Report dock");
    expect(markup).toContain("Analytics");
    expect(markup).toContain("Continuity");
    expect(markup).toContain("Review packet");
    expect(markup).toContain("Import JSON");
    expect(markup).toContain("report-dock-packet-tab");
  });
});
