import { TrackSensorWorkbench } from "./TrackSensorWorkbench";
import { getSelectedTrackSensorWorkbenchModel } from "./trackSensorWorkbenchSelectors";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

export function SelectedTrackSensorWorkbench({
  selectedTrackId,
  models,
}: {
  selectedTrackId: string | null;
  models?: readonly TrackSensorWorkbenchModel[];
}) {
  const selectedModel = getSelectedTrackSensorWorkbenchModel(selectedTrackId, models);

  if (selectedModel === null) {
    return (
      <section
        className="rounded border border-slate-800 bg-slate-950/45 p-4 text-xs text-slate-400"
        data-testid="selected-track-sensor-empty"
      >
        <p className="font-semibold uppercase tracking-wide text-slate-300">Track & sensor workbench</p>
        <p className="mt-2">Select a track to inspect sensor and track details.</p>
      </section>
    );
  }

  return <TrackSensorWorkbench model={selectedModel} />;
}
