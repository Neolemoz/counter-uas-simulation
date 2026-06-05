import { getSelectedEntityAdvisory } from "@/intelligence/intelligenceSelectors";
import type { RtIntelligenceAdvisoryTransportV1 } from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  buildLiveTraceabilityAssemblyInput,
  type LiveTraceabilityAdapterInput,
} from "./liveTraceabilityAdapter";
import type { LiveMirrorFreshness } from "./liveTraceabilityFreshness";
import {
  assembleTraceabilityWorkbenchModel,
  type TraceabilityAssemblyInput,
} from "./traceabilitySelectors";
import type { TraceabilityWorkbenchModel } from "./traceabilityWorkbenchTypes";

export type LiveTraceabilitySelectorInput = {
  selectedEntityId: string | null | undefined;
  entityPoseMirror: ChannelSnapshot<"entity_pose_mirror"> | null | undefined;
  intelligenceAdvisory: RtIntelligenceAdvisoryTransportV1 | null | undefined;
  mirrorFreshness: LiveMirrorFreshness | null | undefined;
};

function findSelectedEntity(
  selectedEntityId: string | null | undefined,
  entityPoseMirror: ChannelSnapshot<"entity_pose_mirror"> | null | undefined,
): Record<string, unknown> | null {
  if (!selectedEntityId || selectedEntityId.trim().length === 0) return null;
  const entities = entityPoseMirror?.payload.entities;
  if (!Array.isArray(entities)) return null;
  return (
    (entities as Array<Record<string, unknown>>).find(
      (entity) => entity.entity_id === selectedEntityId,
    ) ?? null
  );
}

export function getLiveTraceabilityAssemblyInput({
  selectedEntityId,
  entityPoseMirror,
  intelligenceAdvisory,
  mirrorFreshness,
}: LiveTraceabilitySelectorInput): TraceabilityAssemblyInput | null {
  const entity = findSelectedEntity(selectedEntityId, entityPoseMirror);
  if (!entity) return null;

  const adapterInput: LiveTraceabilityAdapterInput = {
    selectedEntityId,
    entity,
    advisory: getSelectedEntityAdvisory(intelligenceAdvisory, selectedEntityId, {
      includeStale: true,
    }),
    mirrorSnapshot: entityPoseMirror,
    mirrorFreshness,
    advisoryStale: intelligenceAdvisory?.stale ?? false,
    advisoryStaleReason: intelligenceAdvisory?.stale_reason ?? null,
  };
  return buildLiveTraceabilityAssemblyInput(adapterInput);
}

export function getSelectedLiveTraceabilityModel(
  input: LiveTraceabilitySelectorInput,
): TraceabilityWorkbenchModel | null {
  const assemblyInput = getLiveTraceabilityAssemblyInput(input);
  return assemblyInput === null ? null : assembleTraceabilityWorkbenchModel(assemblyInput);
}
