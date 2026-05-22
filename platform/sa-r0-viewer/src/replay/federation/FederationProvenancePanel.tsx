import { useEffect, useState } from "react";
import { loadFederationIntegrityReport } from "./loadFederationArtifacts";
import type { ReplayFederationIntegrityReport } from "./federationSchema";

export function FederationProvenancePanel() {
  const [report, setReport] = useState<ReplayFederationIntegrityReport | null>(null);

  useEffect(() => {
    loadFederationIntegrityReport().then(setReport).catch(() => setReport(null));
  }, []);

  if (!report) {
    return <p className="text-xs text-slate-500">No federation integrity report.</p>;
  }

  const errors = (report.findings ?? []).filter((f) => f.severity === "error");
  const warnings = (report.findings ?? []).filter((f) => f.severity === "warning");

  return (
    <div className="space-y-2 text-xs">
      <p className={report.integrity_ok ? "text-emerald-400/80" : "text-amber-300/80"}>
        Integrity: {report.integrity_ok ? "ok" : "issues"} · {report.findings?.length ?? 0}{" "}
        findings
      </p>
      {errors.length > 0 && (
        <ul className="space-y-0.5 text-red-300/80">
          {errors.slice(0, 8).map((f, i) => (
            <li key={`e-${i}`}>
              {f.kind}: {f.message}
            </li>
          ))}
        </ul>
      )}
      {warnings.length > 0 && (
        <ul className="space-y-0.5 text-amber-300/70">
          {warnings.slice(0, 6).map((f, i) => (
            <li key={`w-${i}`}>
              {f.kind}
              {f.supersession_note ? ` (${f.supersession_note})` : ""}: {f.message}
            </li>
          ))}
          {warnings.length > 6 && (
            <li className="text-slate-600">…and {warnings.length - 6} more warnings</li>
          )}
        </ul>
      )}
    </div>
  );
}
