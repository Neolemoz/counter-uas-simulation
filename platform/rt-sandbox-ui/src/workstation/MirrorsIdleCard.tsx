import { PanelShell } from "@/components/GovernanceChrome";

export function MirrorsIdleCard() {
  return (
    <PanelShell title="Telemetry mirrors (idle)">
      <p className="text-sm text-slate-500">
        Connect and subscribe to surface lifecycle, session health, world summary, entity pose
        mirror, and clock mirror channels via pull-only telemetry.
      </p>
    </PanelShell>
  );
}
