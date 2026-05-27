import type { CaptureHandoffRow } from "@/bridge/types";

export const HANDOFF_AUTHORITY_STATIC = [
  "RT authority ends at handoff staging — not SA replay corpus.",
  "SA lineage begins only on explicit maintainer: scripts/rt/rt_sa_import.py commit",
  "Mirrors and session_id are correlation only — not lineage parents.",
] as const;

export function handoffAuthorityDetail(row: CaptureHandoffRow | null): string {
  if (!row) {
    return "Select a capture row or run maintainer CLIs after capture_session.";
  }
  const parts = [
    `Source: ${row.source_origin}`,
    row.lineage_note,
    `Validation: ${row.validation_ok ? "ok" : "issues — inspect with rt_capture_inspect"}`,
  ];
  if (row.has_import_record) {
    parts.push("Import record present — corpus commit completed (maintainer CLI).");
  } else if (row.has_handoff_manifest) {
    parts.push("Handoff manifest present — SA import steps remain maintainer-only.");
  }
  return parts.join(" ");
}
