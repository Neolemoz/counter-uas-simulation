type Props = {
  title: string;
  rows: Record<string, unknown>[];
  columns: { key: string; label: string }[];
};

function formatCell(value: unknown): string {
  if (value === null || value === undefined) return "—";
  if (typeof value === "object") return JSON.stringify(value);
  return String(value);
}

export function TacticalAnnexTimelineTable({ title, rows, columns }: Props) {
  if (rows.length === 0) {
    return (
      <div className="text-xs text-slate-500">
        <span className="font-medium text-slate-400">{title}</span>: (empty)
      </div>
    );
  }
  return (
    <div className="overflow-x-auto">
      <p className="mb-1 text-xs font-medium text-slate-400">
        {title} <span className="text-slate-600">({rows.length})</span>
      </p>
      <table className="w-full border-collapse text-left text-[10px] text-slate-400">
        <thead>
          <tr className="border-b border-slate-700 text-slate-500">
            {columns.map((c) => (
              <th key={c.key} className="px-1 py-0.5 font-medium">
                {c.label}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {rows.map((row, i) => (
            <tr key={i} className="border-b border-slate-800/80">
              {columns.map((c) => (
                <td key={c.key} className="max-w-[8rem] truncate px-1 py-0.5 font-mono">
                  {formatCell(row[c.key])}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
