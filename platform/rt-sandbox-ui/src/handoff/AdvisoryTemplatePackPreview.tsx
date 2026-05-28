import { useMemo, useState } from "react";
import type { SessionAdvisorySummary, TemplatePackId } from "./advisoryTypes";
import { renderTemplatePackPreview } from "./advisoryAggregationV2";

const PACK_OPTIONS: { id: TemplatePackId; label: string }[] = [
  { id: "standup_json_v2", label: "JSON v2" },
  { id: "standup_md_daily", label: "Markdown daily" },
  { id: "standup_md_minimal", label: "Markdown minimal" },
];

export function AdvisoryTemplatePackPreview({
  summary,
}: {
  summary: SessionAdvisorySummary | null;
}) {
  const [packId, setPackId] = useState<TemplatePackId>("standup_md_minimal");
  const content = useMemo(() => {
    if (!summary) return "";
    return renderTemplatePackPreview(summary, packId);
  }, [summary, packId]);

  if (!summary) return null;

  return (
    <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Template pack preview (render-only)
      </h3>
      <p className="mb-2 text-[10px] text-slate-500">
        Preview only — does not run maintainer CLIs or commit.
      </p>
      <label className="mb-2 flex flex-wrap items-center gap-2 text-xs text-slate-400">
        <span>Pack</span>
        <select
          className="rounded border border-slate-600 bg-slate-900 px-2 py-1 text-slate-200"
          value={packId}
          onChange={(e) => setPackId(e.target.value as TemplatePackId)}
        >
          {PACK_OPTIONS.map((o) => (
            <option key={o.id} value={o.id}>
              {o.label}
            </option>
          ))}
        </select>
      </label>
      <pre className="max-h-40 overflow-auto rounded border border-slate-800 bg-slate-900/80 p-2 text-[10px] text-slate-400 whitespace-pre-wrap">
        {content}
      </pre>
    </div>
  );
}
