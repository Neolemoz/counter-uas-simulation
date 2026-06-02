import { PanelShell } from "@/components/GovernanceChrome";
import { RuntimeProfileSelector } from "@/components/RuntimeProfileSelector";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";

export function ConnectPlaceholder({
  sessionRuntimeProfile,
  onSessionRuntimeProfileChange,
}: {
  sessionRuntimeProfile: SessionRuntimeProfile;
  onSessionRuntimeProfileChange: (profile: SessionRuntimeProfile) => void;
}) {
  return (
    <PanelShell title="Runtime workstation">
      <p className="text-sm text-slate-400">
        Start a loopback session to enable world editing, Cesium mirror view, and telemetry
        mirrors. Capture and handoff staging remain maintainer-CLI authority — not browser
        authority.
      </p>
      <div className="mt-3">
        <RuntimeProfileSelector
          value={sessionRuntimeProfile}
          onChange={onSessionRuntimeProfileChange}
        />
      </div>
      <p className="mt-2 text-xs text-slate-500">
        Workflow order: choose runtime → connect → edit (SVG) → inspect (Cesium + mirrors) →
        stop → maintainer capture pipeline.
      </p>
    </PanelShell>
  );
}
