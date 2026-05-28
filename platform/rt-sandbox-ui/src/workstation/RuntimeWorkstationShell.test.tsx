import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { RuntimeWorkstationShell } from "./RuntimeWorkstationShell";

describe("RuntimeWorkstationShell", () => {
  it("renders cognition rail region when cognitionColumn provided", () => {
    const markup = renderToStaticMarkup(
      <RuntimeWorkstationShell
        header={<h1>RT</h1>}
        sessionRail={<div>tabs</div>}
        workflowStrip={<div>workflow</div>}
        cognitionColumn={<div data-testid="cognition-rail-content">hub</div>}
        vizColumn={<div>cesium</div>}
        worldColumn={<div>world</div>}
        globeFooter={<div>compact-diag</div>}
        mirrorsColumn={<div>mirrors</div>}
        pipelineFooter={<div>pipeline</div>}
        diagnostics={<div>diag</div>}
      />,
    );
    expect(markup).toContain('aria-label="Cognition rail"');
    expect(markup).toContain("cognition-rail-content");
    expect(markup).toContain("xl:sticky");
    expect(markup).toContain('aria-label="Background session summary"');
    expect(markup).toContain("compact-diag");
  });

  it("omits cognition rail when not provided", () => {
    const markup = renderToStaticMarkup(
      <RuntimeWorkstationShell
        header={<h1>RT</h1>}
        sessionRail={<div>tabs</div>}
        workflowStrip={<div>workflow</div>}
        vizColumn={<div>cesium</div>}
        worldColumn={<div>world</div>}
        mirrorsColumn={<div>mirrors</div>}
        pipelineFooter={<div>pipeline</div>}
        diagnostics={<div>diag</div>}
      />,
    );
    expect(markup).not.toContain('aria-label="Cognition rail"');
    expect(markup).toContain('aria-label="Globe and layers"');
  });
});
