import { PanelShell } from "@/components/GovernanceChrome";

export function ConnectPlaceholder() {
  return (
    <PanelShell title="Runtime workstation">
      <p className="text-sm text-slate-400">
        Start a loopback session to enable world editing, Cesium mirror view, and telemetry
        mirrors. Capture and handoff staging remain maintainer-CLI authority — not browser
        authority.
      </p>
      <p className="mt-2 text-xs text-slate-500">
        Workflow order: connect → edit (SVG) → inspect (Cesium + mirrors) → stop → maintainer
        capture pipeline.
      </p>
    </PanelShell>
  );
}
