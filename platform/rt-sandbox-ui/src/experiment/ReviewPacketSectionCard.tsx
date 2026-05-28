import { ReviewStepCompletionBadge } from "./ReviewStepCompletionBadge";
import type { ReviewStepCompletionState } from "./reviewStepCompletion";
import type { ReviewPacketSectionEntry } from "./reviewPacketSections";

export function ReviewPacketSectionCard({
  section,
  completionHint,
  organizationHint,
}: {
  section: ReviewPacketSectionEntry;
  completionHint?: ReviewStepCompletionState;
  organizationHint?: string;
}) {
  return (
    <div
      className="rounded border border-slate-800 bg-slate-950/60 p-2"
      data-testid={`packet-section-${section.section_id}`}
    >
      <div className="flex flex-wrap items-center gap-2">
        <p className="text-[10px] font-medium text-slate-300">{section.title}</p>
        {completionHint && <ReviewStepCompletionBadge state={completionHint} />}
      </div>
      {organizationHint && (
        <p className="text-[9px] text-slate-600">{organizationHint}</p>
      )}
      <p className="mt-1 whitespace-pre-wrap text-[10px] text-slate-500">
        {section.body_markdown}
      </p>
    </div>
  );
}
