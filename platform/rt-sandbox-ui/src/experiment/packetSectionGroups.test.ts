import { describe, expect, it } from "vitest";
import {
  groupPacketSections,
  PACKET_SECTION_GROUPS,
  sectionOrganizationHint,
} from "./packetSectionGroups";
import type { ReviewPacketSectionEntry } from "./reviewPacketSections";

const sampleSections: ReviewPacketSectionEntry[] = [
  {
    section_id: "scope",
    title: "Review scope",
    body_markdown: "scope",
    refs: [],
  },
  {
    section_id: "compare_summary",
    title: "Compare snapshot",
    body_markdown: "compare",
    refs: [],
  },
];

describe("packetSectionGroups", () => {
  it("defines three groups", () => {
    expect(PACKET_SECTION_GROUPS).toHaveLength(3);
  });

  it("groupPacketSections buckets by catalog", () => {
    const grouped = groupPacketSections(sampleSections);
    expect(grouped).toHaveLength(2);
    expect(grouped[0]!.group.id).toBe("navigation");
    expect(grouped[1]!.group.id).toBe("compare");
  });

  it("sectionOrganizationHint returns labels", () => {
    expect(sectionOrganizationHint("scope")).toContain("Cohort");
  });
});
