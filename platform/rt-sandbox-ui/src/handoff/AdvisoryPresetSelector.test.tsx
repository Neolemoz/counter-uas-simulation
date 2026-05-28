import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { AdvisoryPresetSelector } from "./AdvisoryPresetSelector";

describe("AdvisoryPresetSelector", () => {
  it("shows preset does not invoke CLI banner", () => {
    const markup = renderToStaticMarkup(
      <AdvisoryPresetSelector value="all_staged" onChange={() => {}} />,
    );
    expect(markup).toContain("Filter preset ≠ CLI invocation");
    expect(markup).toContain('aria-label="Advisory filter preset"');
  });
});
