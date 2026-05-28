import { shortCaptureId } from "./captureIdDisplay";

export function AdvisoryFocusChips({
  captureIds,
  focusSet,
  onToggle,
  onClear,
}: {
  captureIds: string[];
  focusSet: ReadonlySet<string>;
  onToggle: (id: string) => void;
  onClear: () => void;
}) {
  if (!captureIds.length) return null;
  return (
    <div className="mb-3 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Focus set (cognition only)
      </h3>
      <p className="mb-2 text-[10px] text-slate-500">
        Highlight captures for stand-up — does not merge advisory authority.
      </p>
      <div className="flex flex-wrap gap-1">
        {captureIds.map((id) => {
          const active = focusSet.has(id);
          return (
            <button
              key={id}
              type="button"
              className={`rounded border px-2 py-0.5 font-mono text-[10px] ${
                active
                  ? "border-amber-700/60 bg-amber-950/40 text-amber-100"
                  : "border-slate-700 bg-slate-900 text-slate-400 hover:bg-slate-800"
              }`}
              onClick={() => onToggle(id)}
              aria-pressed={active}
            >
              {shortCaptureId(id)}
            </button>
          );
        })}
        {focusSet.size > 0 && (
          <button
            type="button"
            className="rounded border border-slate-600 px-2 py-0.5 text-[10px] text-slate-500"
            onClick={onClear}
          >
            Clear focus
          </button>
        )}
      </div>
    </div>
  );
}
