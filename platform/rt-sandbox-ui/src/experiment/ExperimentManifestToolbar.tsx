export function ExperimentManifestToolbar({
  experimentId,
  onExperimentIdChange,
  connected,
  activeSessionId,
  onPinActive,
  onImportManifest,
  onExportManifest,
  onImportSpec,
  compareModeActive,
  onCompareModeChange,
  analyticsActive,
  onAnalyticsActiveChange,
  continuityReviewActive,
  onContinuityReviewActiveChange,
  f5Active,
  onF5ActiveChange,
}: {
  experimentId: string;
  onExperimentIdChange: (value: string) => void;
  connected: boolean;
  activeSessionId: string | null;
  onPinActive: () => void;
  onImportManifest: () => void;
  onExportManifest: () => void;
  onImportSpec: () => void;
  compareModeActive: boolean;
  onCompareModeChange: (active: boolean) => void;
  analyticsActive: boolean;
  onAnalyticsActiveChange: (active: boolean) => void;
  continuityReviewActive: boolean;
  onContinuityReviewActiveChange: (active: boolean) => void;
  f5Active: boolean;
  onF5ActiveChange: (active: boolean) => void;
}) {
  return (
    <div className="mb-3 flex flex-wrap gap-2">
      <input
        className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-xs font-mono"
        value={experimentId}
        onChange={(e) => onExperimentIdChange(e.target.value)}
      />
      <button
        type="button"
        className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 disabled:opacity-40"
        disabled={!connected || !activeSessionId}
        onClick={onPinActive}
      >
        Pin active session
      </button>
      <button
        type="button"
        className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
        onClick={onImportManifest}
      >
        Import manifest
      </button>
      <button
        type="button"
        className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
        onClick={onExportManifest}
      >
        Export manifest
      </button>
      <button
        type="button"
        className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
        onClick={onImportSpec}
      >
        Import spec
      </button>
      <label className="flex items-center gap-1 text-xs text-slate-400">
        <input
          type="checkbox"
          checked={compareModeActive}
          onChange={(e) => onCompareModeChange(e.target.checked)}
        />
        Compare mode
      </label>
      <label className="flex items-center gap-1 text-xs text-slate-400">
        <input
          type="checkbox"
          checked={analyticsActive}
          onChange={(e) => onAnalyticsActiveChange(e.target.checked)}
        />
        Analytics
      </label>
      <label className="flex items-center gap-1 text-xs text-slate-400">
        <input
          type="checkbox"
          checked={continuityReviewActive}
          onChange={(e) => onContinuityReviewActiveChange(e.target.checked)}
        />
        Continuity review
      </label>
      <label className="flex items-center gap-1 text-xs text-slate-400">
        <input
          type="checkbox"
          checked={f5Active}
          onChange={(e) => onF5ActiveChange(e.target.checked)}
        />
        Advanced metrics (F5)
      </label>
    </div>
  );
}
