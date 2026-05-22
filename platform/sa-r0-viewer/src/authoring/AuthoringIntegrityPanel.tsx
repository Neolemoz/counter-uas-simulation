import { useEffect, useMemo, useState } from "react";
import { loadAuthoringIntegrityReport } from "./loadAuthoring";
import type { AuthoringIntegrityReport } from "./authoringSchema";

type Props = {
  packId: string | null;
};

export function AuthoringIntegrityPanel({ packId }: Props) {
  const [report, setReport] = useState<AuthoringIntegrityReport | null>(null);

  useEffect(() => {
    loadAuthoringIntegrityReport()
      .then(setReport)
      .catch(() => setReport(null));
  }, []);

  const packResult = useMemo(() => {
    if (!packId || !report?.per_pack) return null;
    return report.per_pack[packId] ?? null;
  }, [packId, report]);

  if (!report) {
    return (
      <p className="text-xs text-slate-500">
        No integrity report mirror. Run{" "}
        <span className="font-mono">sync_authoring_mirrors.py</span> (CLI).
      </p>
    );
  }

  const packErrors = packResult?.errors ?? [];
  const packWarnings = packResult?.warnings ?? [];

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-teal-200/90">
        {report.governance_banner ?? "INTEGRITY MIRROR — explanatory audit only"}
      </p>
      <p className={report.ok ? "text-emerald-400/90" : "text-amber-300/80"}>
        Corpus: {report.manifest_count ?? "?"}/{report.pack_count ?? "?"} manifests ·{" "}
        {report.ok ? "pass" : "issues"}
      </p>
      {report.checked_at && (
        <p className="text-[10px] text-slate-600">Checked: {report.checked_at}</p>
      )}
      {packId && (
        <>
          <p className="text-slate-400">
            Pack <span className="font-mono text-slate-200">{packId}</span>
          </p>
          {packErrors.length === 0 && packWarnings.length === 0 && (
            <p className="text-emerald-400/80">No pack-specific issues in last audit.</p>
          )}
          {packErrors.map((e) => (
            <p key={e} className="text-red-300/90">
              {e}
            </p>
          ))}
          {packWarnings.map((w) => (
            <p key={w} className="text-amber-200/80">
              {w}
            </p>
          ))}
        </>
      )}
      {!packId && (report.errors?.length ?? 0) > 0 && (
        <ul className="list-inside list-disc text-red-300/90">
          {(report.errors ?? []).slice(0, 5).map((e) => (
            <li key={e}>{e}</li>
          ))}
        </ul>
      )}
    </div>
  );
}
