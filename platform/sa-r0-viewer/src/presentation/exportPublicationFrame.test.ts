import { describe, expect, it } from "vitest";
import { bundleChapterExportText, snapshotFilename } from "./exportPublicationFrame";
import type { ReplaySaBundle } from "@/replay/bundleSchema";

describe("exportPublicationFrame", () => {
  it("builds deterministic snapshot filename", () => {
    const bundle = {
      scenario: { catalog_pack_id: "valley_ingress", scenario_id: "v", title: "T" },
    } as ReplaySaBundle;
    expect(snapshotFilename(bundle, 2)).toBe("replay_valley_ingress_ch2.png");
  });

  it("exports chapter markdown with governance footer", () => {
    const bundle = {
      presentation: {
        chapters: [{ chapter_id: "c1", title: "Ingress", summary: "Review ingress.", t_start: 0, t_end: 10 }],
      },
    } as ReplaySaBundle;
    const text = bundleChapterExportText(bundle, 0);
    expect(text).toContain("# Ingress");
    expect(text).toContain("not operational assessment");
  });
});
