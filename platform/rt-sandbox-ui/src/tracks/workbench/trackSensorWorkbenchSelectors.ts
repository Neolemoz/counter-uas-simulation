import { TRACK_SENSOR_WORKBENCH_FIXTURES } from "./trackSensorWorkbenchFixtures";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

export function listTrackSensorWorkbenchFixtures(
  models: readonly TrackSensorWorkbenchModel[] = TRACK_SENSOR_WORKBENCH_FIXTURES,
): readonly TrackSensorWorkbenchModel[] {
  return models;
}

export function getSelectedTrackSensorWorkbenchModel(
  selectedTrackId: string | null,
  models: readonly TrackSensorWorkbenchModel[] = TRACK_SENSOR_WORKBENCH_FIXTURES,
): TrackSensorWorkbenchModel | null {
  if (selectedTrackId === null || selectedTrackId.trim().length === 0) return null;
  return models.find((model) => model.track.track_id === selectedTrackId) ?? null;
}

export function isSelectedTrackStale(
  selectedTrackId: string | null,
  models: readonly TrackSensorWorkbenchModel[] = TRACK_SENSOR_WORKBENCH_FIXTURES,
): boolean {
  return getSelectedTrackSensorWorkbenchModel(selectedTrackId, models)?.track.staleness === "stale";
}
