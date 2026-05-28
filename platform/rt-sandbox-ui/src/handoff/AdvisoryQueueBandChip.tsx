import { StatusBadge } from "@/workstation/StatusBadge";
import { queueBandLabel } from "./advisoryQueue";
import type { QueuePriority } from "./advisoryTypes";

export function AdvisoryQueueBandChip({ priority }: { priority: QueuePriority }) {
  const tone =
    priority.band === "P0_block"
      ? "error"
      : priority.band === "P6_import"
        ? "warn"
        : "neutral";
  return (
    <StatusBadge
      label={queueBandLabel(priority.band)}
      tone={tone}
      title={`Queue band ${priority.band} (rank ${priority.rank}) — advisory only`}
    />
  );
}
