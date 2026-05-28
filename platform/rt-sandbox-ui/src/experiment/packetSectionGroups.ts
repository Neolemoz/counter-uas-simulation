import type { ReviewPacketSectionEntry, ReviewPacketSectionId } from "./reviewPacketSections";

export type PacketSectionGroupId = "navigation" | "compare" | "export";

export const PACKET_SECTION_GROUPS: ReadonlyArray<{
  id: PacketSectionGroupId;
  label: string;
  sectionIds: readonly ReviewPacketSectionId[];
}> = [
  {
    id: "navigation",
    label: "Navigation",
    sectionIds: ["scope", "reports"],
  },
  {
    id: "compare",
    label: "Compare and advisory",
    sectionIds: ["compare_summary", "advisory_refs"],
  },
  {
    id: "export",
    label: "Export hints",
    sectionIds: ["cli_hints"],
  },
];

const SECTION_ORGANIZATION_HINT: Partial<Record<ReviewPacketSectionId, string>> = {
  scope: "Cohort and manifest focus",
  reports: "Imported dock slots",
  compare_summary: "Compare mode snapshot",
  advisory_refs: "Handoff refs — display only",
  cli_hints: "Maintainer CLI strings",
};

export function sectionOrganizationHint(sectionId: ReviewPacketSectionId): string | undefined {
  return SECTION_ORGANIZATION_HINT[sectionId];
}

export function groupPacketSections(
  sections: ReviewPacketSectionEntry[],
): Array<{
  group: (typeof PACKET_SECTION_GROUPS)[number];
  sections: ReviewPacketSectionEntry[];
}> {
  const byId = new Map(sections.map((s) => [s.section_id, s]));
  return PACKET_SECTION_GROUPS.map((group) => ({
    group,
    sections: group.sectionIds
      .map((id) => byId.get(id))
      .filter((s): s is ReviewPacketSectionEntry => s != null),
  })).filter((g) => g.sections.length > 0);
}
