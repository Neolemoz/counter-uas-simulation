import type { CompareSide } from "./experimentCompare";

function shortId(id: string | null | undefined): string {
  if (!id) return "—";
  return id.length > 8 ? id.slice(0, 8) : id;
}

function Row({
  label,
  left,
  right,
}: {
  label: string;
  left: string;
  right: string;
}) {
  return (
    <tr className="border-t border-slate-800">
      <th className="py-1 pr-2 text-left font-medium text-slate-500">{label}</th>
      <td className="py-1 font-mono text-slate-300">{left}</td>
      <td className="py-1 font-mono text-slate-300">{right}</td>
    </tr>
  );
}

export function TacticalAbCompareTable({ sideA, sideB }: { sideA: CompareSide; sideB: CompareSide }) {
  const ta = sideA.tactical;
  const tb = sideB.tactical;
  return (
    <table className="w-full text-xs" data-testid="tactical-ab-compare">
      <thead>
        <tr className="text-slate-500">
          <th className="pb-1 text-left">field</th>
          <th className="pb-1 text-left">{sideA.label}</th>
          <th className="pb-1 text-left">{sideB.label}</th>
        </tr>
      </thead>
      <tbody>
        <Row label="source" left={sideA.source} right={sideB.source} />
        <Row label="session" left={shortId(sideA.sessionId)} right={shortId(sideB.sessionId)} />
        <Row label="mode" left={ta?.tactical_mode ?? "—"} right={tb?.tactical_mode ?? "—"} />
        <Row
          label="selected"
          left={shortId(ta?.selected_target_id ?? ta?.selected_interceptor_id)}
          right={shortId(tb?.selected_target_id ?? tb?.selected_interceptor_id)}
        />
        <Row
          label="assigned"
          left={shortId(ta?.assigned_target_id ?? ta?.assigned_interceptor_id)}
          right={shortId(tb?.assigned_target_id ?? tb?.assigned_interceptor_id)}
        />
        <Row
          label="TTI (s)"
          left={typeof ta?.tti_s === "number" ? ta.tti_s.toFixed(1) : "—"}
          right={typeof tb?.tti_s === "number" ? tb.tti_s.toFixed(1) : "—"}
        />
        <Row
          label="autonomous"
          left={ta?.autonomous_loop_status ?? "—"}
          right={tb?.autonomous_loop_status ?? "—"}
        />
      </tbody>
    </table>
  );
}
