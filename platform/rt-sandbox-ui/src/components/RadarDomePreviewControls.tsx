export interface RadarDomePreviewControlState {
  layerEnabled: boolean;
  showVolume: boolean;
  showRing: boolean;
  selectedOnly: boolean;
  showLabels?: boolean;
}

export interface RadarDomePreviewControlHandlers {
  onShowVolumeChange: (visible: boolean) => void;
  onShowRingChange: (visible: boolean) => void;
  onSelectedOnlyChange: (selectedOnly: boolean) => void;
  onShowLabelsChange?: (visible: boolean) => void;
}

export function radarDomeEffectiveRingVisible(state: RadarDomePreviewControlState): boolean {
  return state.layerEnabled && state.showRing;
}

export function radarDomeEffectiveVolumeVisible(state: RadarDomePreviewControlState): boolean {
  return state.layerEnabled && state.showVolume;
}

function statusTone(on: boolean, layerEnabled: boolean): string {
  if (!layerEnabled) return "border-amber-700/50 bg-amber-950/40 text-amber-200";
  return on
    ? "border-cyan-600/60 bg-cyan-950/50 text-cyan-100"
    : "border-slate-700 bg-slate-900/80 text-slate-400";
}

export function RadarDomeStatusChips({
  state,
  className = "",
}: {
  state: RadarDomePreviewControlState;
  className?: string;
}) {
  const ringOn = radarDomeEffectiveRingVisible(state);
  const domeOn = radarDomeEffectiveVolumeVisible(state);
  return (
    <div className={`flex flex-wrap gap-1.5 ${className}`}>
      <span
        className={`rounded-full border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-wide ${statusTone(domeOn, state.layerEnabled)}`}
        data-testid="radar-dome-status-volume"
      >
        Dome {domeOn ? "ON" : "OFF"}
      </span>
      <span
        className={`rounded-full border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-wide ${statusTone(ringOn, state.layerEnabled)}`}
        data-testid="radar-dome-status-ring"
      >
        Ring {ringOn ? "ON" : "OFF"}
      </span>
    </div>
  );
}

const CHECKBOX_CLASS =
  "h-3.5 w-3.5 rounded border-slate-600 bg-slate-950 text-cyan-500 focus:ring-cyan-700/40";

export function RadarDomePreviewPanel({
  state,
  handlers,
}: {
  state: RadarDomePreviewControlState;
  handlers: RadarDomePreviewControlHandlers;
}) {
  return (
    <div
      className="mt-3 rounded border border-cyan-800/50 bg-cyan-950/20 p-3"
      data-testid="radar-dome-preview-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h4 className="text-[11px] font-semibold uppercase tracking-wide text-cyan-100">
          Dome preview
        </h4>
        <RadarDomeStatusChips state={state} />
      </div>
      {!state.layerEnabled && (
        <p className="mb-2 text-[10px] text-amber-200">
          Enable Sensor domes in map Layers to show preview on globe.
        </p>
      )}
      <div className="grid gap-2 text-xs text-slate-300">
        <label className="flex cursor-pointer items-center gap-2">
          <input
            type="checkbox"
            className={CHECKBOX_CLASS}
            checked={state.showVolume}
            disabled={!state.layerEnabled}
            onChange={(e) => handlers.onShowVolumeChange(e.target.checked)}
          />
          Show radar dome (3D)
        </label>
        <label className="flex cursor-pointer items-center gap-2">
          <input
            type="checkbox"
            className={CHECKBOX_CLASS}
            checked={state.showRing}
            disabled={!state.layerEnabled}
            onChange={(e) => handlers.onShowRingChange(e.target.checked)}
          />
          Show detection ring
        </label>
        <label className="flex cursor-pointer items-center gap-2">
          <input
            type="checkbox"
            className={CHECKBOX_CLASS}
            checked={state.selectedOnly}
            disabled={!state.layerEnabled}
            onChange={(e) => handlers.onSelectedOnlyChange(e.target.checked)}
          />
          Selected radar only
        </label>
        {handlers.onShowLabelsChange && (
          <label className="flex cursor-pointer items-center gap-2">
            <input
              type="checkbox"
              className={CHECKBOX_CLASS}
              checked={state.showLabels !== false}
              disabled={!state.layerEnabled}
              onChange={(e) => handlers.onShowLabelsChange?.(e.target.checked)}
            />
            Show range labels
          </label>
        )}
      </div>
    </div>
  );
}

export function RadarDomeMapQuickControls({
  state,
  handlers,
}: {
  state: RadarDomePreviewControlState;
  handlers: Pick<
    RadarDomePreviewControlHandlers,
    "onShowVolumeChange" | "onShowRingChange"
  >;
}) {
  const domeOn = radarDomeEffectiveVolumeVisible(state);
  const ringOn = radarDomeEffectiveRingVisible(state);
  const disabled = !state.layerEnabled;

  const chipClass = (on: boolean) =>
    `rounded border px-2 py-1 text-[10px] font-semibold transition-colors duration-150 disabled:opacity-40 ${
      on
        ? "border-cyan-600/70 bg-cyan-950/70 text-cyan-100"
        : "border-slate-700 bg-slate-950/80 text-slate-400 hover:border-slate-600"
    }`;

  return (
    <div
      className="pointer-events-auto flex flex-wrap items-center gap-1 rounded border border-slate-800/90 bg-slate-950/85 p-1 shadow-lg shadow-black/30 backdrop-blur-sm"
      data-testid="radar-dome-map-quick-controls"
    >
      <span className="px-1 text-[9px] font-medium uppercase tracking-wide text-slate-500">
        Radar
      </span>
      <button
        type="button"
        className={chipClass(domeOn)}
        disabled={disabled}
        title="Toggle 3D dome preview"
        onClick={() => handlers.onShowVolumeChange(!state.showVolume)}
      >
        Dome {domeOn ? "ON" : "OFF"}
      </button>
      <button
        type="button"
        className={chipClass(ringOn)}
        disabled={disabled}
        title="Toggle detection ring"
        onClick={() => handlers.onShowRingChange(!state.showRing)}
      >
        Ring {ringOn ? "ON" : "OFF"}
      </button>
    </div>
  );
}
