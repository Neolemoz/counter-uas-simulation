import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TacticalAbCompareTable } from "./TacticalAbCompareTable";
import type { CompareSide } from "./experimentCompare";
import { FORBIDDEN_LEXICON } from "@/governance/banners";

function side(label: string, mode: string): CompareSide {
  return {
    label,
    sessionId: "abcdef12-3456",
    source: "live",
    tactical: { tactical_mode: mode, schema: "rt_tactical_state_v1" },
    worldSummary: null,
    lifecycle: null,
    annexSummary: null,
  };
}

describe("TacticalAbCompareTable", () => {
  it("renders A/B columns without forbidden lexicon", () => {
    const markup = renderToStaticMarkup(
      <TacticalAbCompareTable sideA={side("A", "manual")} sideB={side("B", "assisted")} />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="tactical-ab-compare"');
    expect(markup).toContain("manual");
    expect(markup).toContain("assisted");
  });
});
