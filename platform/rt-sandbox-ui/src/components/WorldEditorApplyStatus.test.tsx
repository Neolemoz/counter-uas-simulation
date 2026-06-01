import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { WorldEditorApplyStatus } from "./WorldEditorApplyStatus";

describe("WorldEditorApplyStatus", () => {
  it("renders applying state", () => {
    const html = renderToStaticMarkup(
      <WorldEditorApplyStatus status={{ phase: "applying" }} />,
    );
    expect(html).toContain("Applying");
    expect(html).toContain('data-phase="applying"');
  });

  it("renders applied with entity count", () => {
    const html = renderToStaticMarkup(
      <WorldEditorApplyStatus status={{ phase: "applied", entityCount: 3 }} />,
    );
    expect(html).toContain("Applied");
    expect(html).toContain("3 entities");
  });

  it("renders nothing when idle", () => {
    const html = renderToStaticMarkup(
      <WorldEditorApplyStatus status={{ phase: "idle" }} />,
    );
    expect(html).toBe("");
  });
});
