import { describe, expect, it } from "vitest";
import { resolveWorkspaceSegment } from "./resolveWorkspaceSegment";

describe("resolveWorkspaceSegment", () => {
  it("forces compare when compare mode active", () => {
    expect(
      resolveWorkspaceSegment(
        {
          compareMode: true,
          presentationMode: false,
          filmstripMode: false,
          sweepMode: false,
          hasBundle: true,
        },
        "replay",
      ),
    ).toBe("compare");
  });

  it("forces report when presentation mode active", () => {
    expect(
      resolveWorkspaceSegment(
        {
          compareMode: false,
          presentationMode: true,
          filmstripMode: false,
          sweepMode: false,
          hasBundle: true,
        },
        "scenario",
      ),
    ).toBe("report");
  });

  it("respects user segment when not mode-locked", () => {
    expect(
      resolveWorkspaceSegment(
        {
          compareMode: false,
          presentationMode: false,
          filmstripMode: false,
          sweepMode: false,
          hasBundle: true,
        },
        "corpus",
      ),
    ).toBe("corpus");
  });

  it("defaults to replay", () => {
    expect(
      resolveWorkspaceSegment(
        {
          compareMode: false,
          presentationMode: false,
          filmstripMode: false,
          sweepMode: false,
          hasBundle: true,
        },
        null,
      ),
    ).toBe("replay");
  });
});
