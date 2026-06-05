import type { RtIntelligenceAdvisoryTransportV1 } from "@/intelligence/intelligenceAdvisory";
import { getSelectedEntityAdvisory } from "@/intelligence/intelligenceSelectors";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  buildLiveTrackSensorWorkbenchModel,
  type LiveTrackSensorAdapterInput,
} from "./liveTrackSensorAdapter";
import type { LiveTrackMirrorFreshness } from "./liveTrackFreshness";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

export type LiveTrackSensorWorkbenchSelectorInput = {
  selectedEntityId: string | null | undefined;
  entityPoseMirror: ChannelSnapshot<"entity_pose_mirror"> | null | undefined;
  intelligenceAdvisory: RtIntelligenceAdvisoryTransportV1 | null | undefined;
  mirrorFreshness: LiveTrackMirrorFreshness | null | undefined;
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

export function getLiveTrackSensorWorkbenchModel({
  selectedEntityId,
  entityPoseMirror,
  intelligenceAdvisory,
  mirrorFreshness,
}: LiveTrackSensorWorkbenchSelectorInput): TrackSensorWorkbenchModel | null {
  const entity = findSelectedEntity(selectedEntityId, entityPoseMirror);
  if (!entity) return null;

  const adapterInput: LiveTrackSensorAdapterInput = {
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

  return buildLiveTrackSensorWorkbenchModel(adapterInput);
}

export function getSelectedLiveTrackSensorWorkbenchModel(
  input: LiveTrackSensorWorkbenchSelectorInput,
): TrackSensorWorkbenchModel | null {
  return getLiveTrackSensorWorkbenchModel(input);
}
